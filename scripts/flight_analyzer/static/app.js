/* Flight Analyzer — uPlot 차트, 커서/줌 연동, 기록 목록 사이드바. */
"use strict";

const T = "/pl/control/status";
const D = "/pl/detection/status";
const M = "/pl/monitor/status";

function panel(title, topic, fields, note) {
  return { title, note, series: fields.map((field) => {
    const [f, label] = Array.isArray(field) ? field : [field, field];
    return { t: topic, f, label };
  }) };
}

const PANELS = [
  panel("Landing state", T, [["active", "Mode active"], ["target_lost", "Target lost"]], "0 / 1"),
  panel("ArUco detection", D, [["detected", "Marker detected"], ["pose_valid", "Pose valid"]], "0 / 1"),
  panel("Marker relative to drone", M,
    [["detection_value_body_position_m_0", "Forward"], ["detection_value_body_position_m_1", "Right"],
     ["detection_value_body_position_m_2", "Down"]], "Body FRD [m]"),
  panel("Estimated target position", "/target_pose_world",
    [["pose_position_x", "North"], ["pose_position_y", "East"], ["pose_position_z", "Down"]], "Local NED [m]"),
  panel("Ground and vertical speed", M,
    [["velocity_value_horizontal_m_s", "Horizontal"], ["velocity_value_vertical_m_s", "Down"]], "[m/s]"),
  panel("GPS", M, [["gps_value_fix_type", "Fix type"], ["gps_value_satellites", "Satellites"]]),
  panel("Battery voltage", M, [["battery_value_voltage_v", "Voltage"]], "[V]"),
  panel("Battery remaining", M, [["battery_value_remaining_pct", "Remaining"]], "[%]"),
];

const PALETTE = ["#60a5fa", "#f87171", "#4ade80", "#fbbf24", "#c084fc", "#22d3ee", "#f472b6", "#a3e635"];
const STATE_COLORS = {
  SEARCH: "rgba(148,163,184,0.10)",
  ALIGN: "rgba(234,179,8,0.18)",
  APPROACH: "rgba(34,197,94,0.16)",
  FINAL_APPROACH: "rgba(59,130,246,0.20)",
  FINISHED: "rgba(168,85,247,0.18)",
  INACTIVE: "rgba(148,163,184,0.07)",
};
const CHART_H = 210;

// ---------------------------------------------------------------- 상태
let rec = null;            // 현재 기록 데이터
let charts = [];           // uPlot 인스턴스
let stateSegs = [];        // [{t0, t1, state}]
let syncing = false;       // 줌 전파 재진입 방지
let records = [];
let currentId = null;
let customPanels = loadLS("pl_analyzer.custom", []);
let layout = loadLS("pl_analyzer.layout", "split");
let sceneCenter = loadLS("pl_analyzer.scene_center", "origin");
let scene, sceneRecord, scenePromise;
let sceneReady = false;
let displayTime = 0;
let vmode = loadLS("pl_analyzer.vmode", "play");      // "play"(영상 재생) | "follow"(그래프 커서 따라감)
let sidebarCollapsed = loadLS("pl_analyzer.sidebar_collapsed", false);
// 영상과 3D가 공유하는 재생 상태
const V = { t: 0, playing: false, speed: 1, shown: -1, want: -1, loading: false, lastTs: 0, lastCursor: 0, request: null, raf: 0 };

const $ = (id) => document.getElementById(id);

function setContentLoading(show) {
  const overlay = $("content-loading");
  if (!show) {
    overlay.hidden = true;
    return;
  }
  const content = $("content").getBoundingClientRect();
  const topbar = $("topbar").getBoundingClientRect();
  overlay.style.left = `${Math.max(0, content.left)}px`;
  overlay.style.right = `${Math.max(0, window.innerWidth - content.right)}px`;
  overlay.style.top = `${Math.max(0, topbar.bottom)}px`;
  overlay.style.bottom = "0";
  overlay.hidden = false;
}

function loadLS(k, d) { try { return JSON.parse(localStorage.getItem(k)) ?? d; } catch { return d; } }
function saveLS(k, v) { try { localStorage.setItem(k, JSON.stringify(v)); } catch { /* 무시 */ } }

async function api(path, opts) {
  const r = await fetch(path, opts);
  const j = await r.json().catch(() => ({}));
  if (!r.ok) throw new Error(j.error || `${r.status} ${r.statusText}`);
  return j;
}

function fmtDur(s) {
  if (!isFinite(s)) return "—";
  const m = Math.floor(s / 60);
  return m ? `${m}분 ${(s % 60).toFixed(0)}초` : `${s.toFixed(1)}초`;
}
function fmtSize(b) {
  if (b > 1e9) return (b / 1e9).toFixed(1) + " GB";
  if (b > 1e6) return (b / 1e6).toFixed(1) + " MB";
  return (b / 1e3).toFixed(0) + " KB";
}
function fmtVal(v) {
  if (v == null || !isFinite(v)) return "—";
  const a = Math.abs(v);
  if (a !== 0 && (a < 1e-3 || a >= 1e5)) return v.toExponential(2);
  return a >= 100 ? v.toFixed(1) : a >= 10 ? v.toFixed(2) : v.toFixed(4);
}
function cssVar(n) { return getComputedStyle(document.documentElement).getPropertyValue(n).trim(); }

// ---------------------------------------------------------------- 기록 목록
async function loadRecords() {
  const j = await api("/api/records");
  records = j.records;
  renderRecords();
  const want = decodeURIComponent(location.hash.slice(1));
  const preferred = records.find((r) => r.id === want) || records[0];
  if (preferred) await openRecord(preferred.id);
}
function renderRecords() {
  const ul = $("rec-list");
  ul.innerHTML = "";
  if (!records.length) {
    ul.innerHTML = `<li class="empty">이 폴더에 기록(.db3)이 없음</li>`;
    return;
  }
  let previousDate = null;
  for (const r of records) {
    const date = r.start ? r.start.slice(0, 10) : "날짜 알 수 없음";
    if (date !== previousDate) {
      const heading = document.createElement("li");
      heading.className = "rec-date";
      heading.textContent = date;
      ul.appendChild(heading);
      previousDate = date;
    }
    const li = document.createElement("li");
    li.className = r.id === currentId ? "active" : "";
    li.innerHTML = `<div class="rec-name"></div><div class="rec-sub"></div>`;
    const name = li.children[0];
    name.append(document.createTextNode(r.name));
    const sub = li.children[1];
    const parts = [
      r.start ? r.start.replace("T", " ").slice(5, 16) : null,
      r.duration != null ? fmtDur(r.duration) : null,
      fmtSize(r.size),
    ].filter(Boolean);
    for (const p of parts) { const s = document.createElement("span"); s.textContent = p; sub.appendChild(s); }
    li.onclick = () => openRecord(r.id);
    ul.appendChild(li);
  }
}

// ---------------------------------------------------------------- 기록 열기
async function openRecord(id) {
  setPlaying(false);
  currentId = id;
  history.replaceState(null, "", "#" + encodeURIComponent(id));
  renderRecords();
  $("rec-title").textContent = records.find((r) => r.id === id)?.name ?? id;
  $("rec-meta").textContent = "불러오는 중…";
  $("errors").innerHTML = "";
  setContentLoading(true);
  let selected;
  try {
    selected = await api("/api/record?id=" + encodeURIComponent(id));
  } catch (e) {
    if (currentId !== id) return;
    setContentLoading(false);
    $("rec-meta").textContent = "";
    $("errors").innerHTML = `<div></div>`;
    $("errors").firstChild.textContent = "불러오기 실패: " + e.message;
    return;
  }
  if (currentId !== id) return;
  rec = selected;
  setContentLoading(false);
  const topics = Object.keys(rec.topics);
  $("rec-meta").textContent =
    `${fmtDur(rec.duration)} · 토픽 ${topics.length}개 · control samples ${rec.topics[T]?.t.length ?? 0}개`;
  $("errors").innerHTML = "";
  for (const e of rec.errors || []) {
    const d = document.createElement("div");
    d.textContent = "역직렬화 실패(건너뜀): " + e;
    $("errors").appendChild(d);
  }
  stateSegs = buildStateSegs();
  renderStateLegend();
  fillFieldList();
  renderPanels();
  resetTimeline();
  updateScene();
}

function buildStateSegs() {
  const src = rec.topics[T];
  const st = src?.strings?.state;
  if (!st) return [];
  const segs = [];
  let cur = null;
  for (let i = 0; i < st.length; i++) {
    const state = src.fields.active?.[i] ? st[i] : "INACTIVE";
    if (!cur || cur.state !== state) {
      if (cur) cur.t1 = src.t[i];
      cur = { state, t0: src.t[i], t1: src.t[i] };
      segs.push(cur);
    }
  }
  if (cur) cur.t1 = src.t[src.t.length - 1];
  return segs;
}
function stateAt(t) {
  let lo = 0, hi = stateSegs.length - 1, ans = null;
  while (lo <= hi) {
    const m = (lo + hi) >> 1;
    if (stateSegs[m].t0 <= t) { ans = stateSegs[m]; lo = m + 1; } else hi = m - 1;
  }
  return ans?.state ?? "";
}

function renderStateLegend() {
  const el = $("state-legend");
  el.innerHTML = "";
  if (!stateSegs.length) return;
  const present = new Set(stateSegs.map((s) => s.state));
  for (const [k, c] of Object.entries(STATE_COLORS)) {
    if (!present.has(k)) continue;
    const s = document.createElement("span");
    s.innerHTML = `<i style="background:${c.replace(/[\d.]+\)$/, "0.55)")}"></i>${k}`;
    el.appendChild(s);
  }
}

function fillFieldList() {
  const dl = $("field-list");
  dl.innerHTML = "";
  for (const [tp, d] of Object.entries(rec.topics)) {
    for (const f of Object.keys(d.fields)) {
      const o = document.createElement("option");
      o.value = `${tp} ${f}`;
      dl.appendChild(o);
    }
  }
  $("custom-add").hidden = false;
}

// ---------------------------------------------------------------- 차트
function destroyCharts() {
  charts.forEach((c) => c.destroy());
  charts = [];
}

function renderPanels() {
  destroyCharts();
  $("empty").hidden = true;
  const container = $("control-panels");
  container.replaceChildren();
  for (const p of [...PANELS, ...customPanels.map((p, i) => ({ ...p, custom: i }))]) {
    if (p.series.some((s) => fieldCol(s.t || T, s.f))) container.appendChild(makePanel(p));
  }
}

function makePanel(p) {
  const det = document.createElement("section");
  det.className = "panel";
  const head = document.createElement("div");
  head.className = "panel-head";
  const title = document.createElement("div");
  title.className = "panel-title";
  title.textContent = p.title;
  const note = document.createElement("div");
  note.className = "note";
  note.textContent = p.note || "";
  head.append(title, note);
  if (p.custom != null) {
    const x = document.createElement("button");
    x.className = "btn small panel-delete";
    x.textContent = "삭제";
    x.onclick = () => {
      customPanels.splice(p.custom, 1);
      saveLS("pl_analyzer.custom", customPanels);
      renderPanels();
    };
    head.appendChild(x);
  }
  det.appendChild(head);
  const body = document.createElement("div");
  det.appendChild(body);

  const built = buildData(p);
  if (!built) {
    body.innerHTML = '<div class="missing">이 기록에 해당 필드가 없음</div>';
    return det;
  }
  const help = makeHelp(p, built);
  body.appendChild(help);
  const chartEl = document.createElement("div");
  body.appendChild(chartEl);
  const chart = makeChart(p, built, chartEl);
  bindToggles(p, built.specs, help, chart);
  return det;
}

function seriesName(s, multiTopic) {
  if (s.label) return s.label;
  return multiTopic && s.t !== T ? `${s.t.split("/").pop()}.${s.f}` : s.f;
}
function fieldCol(tp, f) {
  return rec.topics[tp]?.fields[f] ?? null;
}
function seriesColor(s, i) { return s.color || PALETTE[i % PALETTE.length]; }

/* 패널 위 필터 칩: [체크박스][필드명][수식] 을 작은 버튼으로 나열. 칩 클릭 = 선 켜기/끄기. */
function makeHelp(p, { specs, multiTopic }) {
  const box = document.createElement("div");
  box.className = "help";
  specs.forEach((s, i) => {
    const chip = document.createElement("label");
    chip.className = "chip";
    chip.innerHTML = `<input type="checkbox" class="cb"><span class="nm mono"></span>`;
    chip.querySelector(".cb").checked = seriesShown(p, s);
    chip.querySelector(".nm").textContent = seriesName(s, multiTopic) + (s.scale === "y2" ? " (R)" : "");
    chip.title = s.f;
    chip.style.setProperty("--series", seriesColor(s, i));
    box.appendChild(chip);
  });
  return box;
}

/* 선 표시 여부 — 패널·필드별로 브라우저에 기억. 기본은 패널 정의의 show(없으면 켬). */
const showKey = (p, s) => `pl_analyzer.show.${p.title}.${s.t || T}.${s.f}`;
function seriesShown(p, s) { return loadLS(showKey(p, s), s.show ?? true); }

/* 수식표 체크박스 ↔ 차트 선 표시 연동 (범례 클릭으로 바꿔도 체크박스가 따라간다). */
function bindToggles(p, specs, box, u) {
  const cbs = box.querySelectorAll(".cb");
  cbs.forEach((cb, i) => {
    cb.onchange = () => u.setSeries(i + 1, { show: cb.checked });
  });
  u.hooks.setSeries = u.hooks.setSeries || [];
  u.hooks.setSeries.push((u, idx, opts) => {
    if (idx == null || opts.show == null || !cbs[idx - 1]) return;
    cbs[idx - 1].checked = opts.show;
    saveLS(showKey(p, specs[idx - 1]), opts.show);
  });
}

/* 패널의 시리즈들을 하나의 x축으로 정렬. 토픽이 여러 개면 uPlot.join 으로 합친다. */
function buildData(p) {
  const groups = new Map();   // topic → [spec]
  for (const s of p.series) {
    const tp = s.t || T;
    if (!fieldCol(tp, s.f)) continue;
    if (!groups.has(tp)) groups.set(tp, []);
    groups.get(tp).push(s);
  }
  if (!groups.size) return null;
  const tables = [], specs = [];
  for (const [tp, ss] of groups) {
    const d = rec.topics[tp];
    const cols = ss.map((s) => {
      const src = fieldCol(tp, s.f);
      if (s.mul == null && s.add == null) return src;
      const mul = s.mul ?? 1, add = s.add ?? 0;
      return src.map((v) => (v == null ? null : v * mul + add));
    });
    tables.push([d.t, ...cols]);
    specs.push(...ss.map((s) => ({ ...s, t: tp })));
  }
  const data = tables.length === 1
    ? tables[0]
    : uPlot.join(tables, tables.map((t) => t.slice(1).map(() => 1)));   // 1 = 원래 null(끊김)은 유지
  return { data, specs, multiTopic: groups.size > 1 };
}

function chartWidth() {
  return Math.max(300, $("graphs").clientWidth - 22);
}

function makeChart(p, { data, specs, multiTopic }, el) {
  const axisStroke = cssVar("--axis"), grid = cssVar("--grid");
  const hasY2 = specs.some((s) => s.scale === "y2");
  const series = [{ label: "t[s]", value: (u, v) => (v == null ? "—" : v.toFixed(3)) }];
  specs.forEach((s, i) => {
    const mul = s.mul ?? 1, add = s.add ?? 0;
    series.push({
      label: seriesName(s, multiTopic),
      scale: s.scale || "y",
      stroke: seriesColor(s, i),
      width: s.width ?? 1.5,
      dash: s.dash,
      show: seriesShown(p, s),
      spanGaps: multiTopic,
      paths: s.step ? uPlot.paths.stepped({ align: 1 }) : undefined,
      points: { show: false },
      value: (u, v) => (v == null ? "—" : p.flags ? String(Math.round((v - add) / mul)) : fmtVal((v - add) / mul)),
    });
  });
  const dur = rec.duration || 1;
  const axes = [
    { stroke: axisStroke, grid: { stroke: grid }, ticks: { stroke: grid } },
    p.flags
      ? { stroke: axisStroke, grid: { show: false }, ticks: { show: false }, size: 110,
          splits: () => specs.map((s) => (s.add ?? 0) + 0.4),   // 각 플래그 줄 가운데에 이름
          values: () => specs.map((s) => seriesName(s, false)) }
      : { stroke: axisStroke, grid: { stroke: grid }, ticks: { stroke: grid }, size: 60 },
  ];
  if (hasY2) axes.push({ scale: "y2", side: 1, stroke: axisStroke, grid: { show: false }, size: 50 });

  const opts = {
    width: chartWidth(),
    height: CHART_H,
    series,
    axes,
    scales: {
      x: { time: false, auto: false, range: (u, mn, mx) => [mn ?? 0, mx ?? dur] },
      ...(p.flags ? { y: { auto: false, range: [-0.2, p.series.length] } } : {}),
    },
    legend: { live: true },
    cursor: {
      sync: { key: "precision-landing", setSeries: false },
      drag: { x: true, y: false, setScale: true },
      bind: { dblclick: () => () => { resetZoom(); return null; } },
      points: { size: 6 },
    },
    plugins: [statePlugin()],
    hooks: {
      setScale: [(u, key) => {
        if (key !== "x" || syncing) return;
        syncing = true;
        const { min, max } = u.scales.x;
        for (const c of charts) if (c !== u) c.setScale("x", { min, max });
        syncing = false;
      }],
      setCursor: [(u) => {
        const { left, idx } = u.cursor;
        if (left == null || left < 0 || idx == null) return;
        const t = u.posToVal(left, "x");
        if (vmode === "follow") {
          if (timelineOn()) showAt(t);
        }
      }],
    },
  };
  const u = new uPlot(opts, data, el);
  // 범례 이름에 마우스를 올리면 설명이 뜨게 한다
  u.root.querySelectorAll(".u-legend tr.u-series").forEach((tr, i) => {
    const s = specs[i - 1];
    if (s) tr.title = s.f;
  });
  // Play 모드에서 그래프를 클릭하면 영상이 그 시각으로 이동한다(드래그 확대와는 별개).
  let downX = null;
  u.over.addEventListener("mousedown", (e) => { downX = e.clientX; });
  u.over.addEventListener("mouseup", (e) => {
    if (downX == null || Math.abs(e.clientX - downX) > 3) return;
    if (timelineOn() && vmode === "play" && u.cursor.left >= 0) seekTimeline(u.posToVal(u.cursor.left, "x"));
  });
  u.setScale("x", { min: 0, max: dur });
  // 방금 만든 차트를 다른 차트들의 현재 줌에 맞춘다
  if (charts.length) {
    const { min, max } = charts[0].scales.x;
    u.setScale("x", { min, max });
  }
  charts.push(u);
  return u;
}

/* Use the landing phase to shade the chart background. */
function statePlugin() {
  return {
    hooks: {
      drawClear: [(u) => {
        if (!stateSegs.length) return;
        const { ctx } = u;
        const { left, top, width, height } = u.bbox;
        ctx.save();
        for (const s of stateSegs) {
          const c = STATE_COLORS[s.state];
          if (!c) continue;
          const x0 = Math.max(u.valToPos(s.t0, "x", true), left);
          const x1 = Math.min(u.valToPos(s.t1, "x", true), left + width);
          if (x1 <= x0) continue;
          ctx.fillStyle = c;
          ctx.fillRect(x0, top, x1 - x0, height);
        }
        ctx.restore();
      }],
    },
  };
}

function resetZoom() {
  if (!rec) return;
  syncing = true;
  for (const c of charts) c.setScale("x", { min: 0, max: rec.duration || 1 });
  syncing = false;
}

function addCustom() {
  const raw = $("custom-input").value.trim();
  if (!raw || !rec) return;
  const series = [];
  for (const part of raw.split(",")) {
    const bits = part.trim().split(/\s+/);
    if (!bits[0]) continue;
    const [tp, f] = bits.length >= 2 ? bits : [T, bits[0]];
    if (!fieldCol(tp, f)) {
      alert(`필드 없음: ${tp} ${f}`);
      return;
    }
    series.push({ t: tp, f });
  }
  if (!series.length) return;
  customPanels.push({ title: "Custom: " + series.map((s) => s.f).join(", "), series });
  saveLS("pl_analyzer.custom", customPanels);
  $("custom-input").value = "";
  renderPanels();
  $("control-panels").lastElementChild?.scrollIntoView({ behavior: "smooth", block: "center" });
}

// ---------------------------------------------------------------- 화면 영상
const videoData = () => rec?.video;
const hasVideo = () => !!(videoData() && videoData().t.length);
const videoOn = () => layout === "split" && hasVideo();
const timelineOn = () => videoOn() || (layout === "3d" && sceneReady);

async function updateScene() {
  const record = rec;
  const status = $("scene-status");
  if (layout !== "3d") return;
  if (!record) {
    status.textContent = "기록을 선택하세요.";
    return;
  }
  if (!scene) status.textContent = "3D 화면을 여는 중…";
  try {
    scenePromise ??= import("./scene.mjs").then(({ LandingScene }) =>
      new LandingScene($("scene-host")));
    scene = await scenePromise;
  } catch (error) {
    status.textContent = "3D 화면을 열 수 없습니다. 브라우저의 WebGL 지원을 확인하세요.";
    console.warn(error);
    return;
  }
  if (record !== rec || layout !== "3d") return;
  if (sceneRecord !== record) {
    const data = scene.load(record.topics);
    sceneRecord = record;
    const hasDrone = data.drone.some((row) => row.length > 1);
    const hasEstimate = data.estimate.some((row) => row.length > 1);
    $("scene-center").options[1].disabled = !hasDrone;
    $("scene-truth").hidden = !data.truth.some((row) => row.length > 1);
    sceneReady = hasDrone || hasEstimate || !$("scene-truth").hidden;
    status.textContent = !sceneReady ? "이 기록에는 표시할 위치 데이터가 없습니다."
      : data.hasTruth && $("scene-truth").hidden
        ? "실제 마커를 표시할 수 없습니다. PX4와 Gazebo 위치를 맞출 공통 샘플이 필요합니다."
        : "";
  }
  $("v-play").disabled = $("v-seek").disabled = !sceneReady;
  showAt(displayTime);
}

function showAt(t) {
  displayTime = t;
  if (videoOn()) {
    showFrameAt(t);
  } else if (layout === "3d" && scene && sceneRecord === rec) {
    scene.setTime(t, sceneCenter);
    $("v-info").textContent = `t = ${t.toFixed(2)} s`;
  }
}

function frameIndexAt(t) {          // t 이하인 마지막 프레임
  const ts = videoData().t;
  let lo = 0, hi = ts.length - 1, ans = -1;
  while (lo <= hi) {
    const m = (lo + hi) >> 1;
    if (ts[m] <= t) { ans = m; lo = m + 1; } else hi = m - 1;
  }
  return ans;
}

/* 프레임을 한 장씩 받아 바꾼다. 받는 중에 다른 요청이 오면 마지막 것만 남긴다(밀림 방지). */
function showFrameAt(t) {
  if (!hasVideo()) return;
  V.want = frameIndexAt(t);
  updateVideoInfo(t);
  if (V.want < 0) {
    V.request = null;
    V.loading = false;
    V.shown = -1;
    $("v-img").removeAttribute("src");
    $("v-empty").textContent = "Waiting for first HUD frame";
    return;
  }
  $("v-empty").textContent = "";
  pumpFrame();
}
function pumpFrame() {
  if (V.loading || V.want === V.shown || V.want < 0) return;
  const i = V.want, id = currentId;
  const token = V.request = Symbol();
  V.loading = true;
  const img = new Image();
  img.onload = () => {
    if (token !== V.request || id !== currentId) return;
    V.loading = false;
    $("v-img").src = img.src;
    V.shown = i;
    pumpFrame();
  };
  img.onerror = () => {
    if (token !== V.request || id !== currentId) return;
    V.loading = false; V.shown = i; pumpFrame();
  };
  img.src = `/api/frame?id=${encodeURIComponent(id)}&i=${i}`;
}
function updateVideoInfo(t) {
  if (!hasVideo()) return;
  const st = stateAt(t);
  $("v-info").textContent =
    `t = ${t.toFixed(2)} s · frame ${V.want < 0 ? "—" : V.want + 1}/${videoData().t.length}${st ? " · " + st : ""}`;
}

/* 재생 위치를 모든 그래프의 커서로 표시한다.
 * uPlot의 cursor sync는 포인터 이벤트는 전파하지만, setCursor()로 넣은 재생 위치는
 * 다른 차트로 전파하지 않는다. 따라서 각 차트의 동일한 시간 좌표에 직접 둔다. */
function cursorToTime(t) {
  const first = charts[0];
  if (!first) return;
  const { min, max } = first.scales.x;
  if (t < min || t > max) return;
  for (const u of charts) {
    u.setCursor({
      left: u.valToPos(t, "x"),
      top: u.bbox.height / devicePixelRatio / 2,
    });
  }
}

function seekTimeline(t) {
  if (!timelineOn()) return;
  V.t = Math.min(Math.max(t, 0), rec.duration);
  $("v-seek").value = V.t;
  showAt(V.t);
  cursorToTime(V.t);
}
function setPlaying(on) {
  cancelAnimationFrame(V.raf);
  V.playing = on && timelineOn();
  $("v-play").textContent = V.playing ? "❚❚" : "▶";
  if (V.playing) {
    if (V.t >= rec.duration - 1e-3) V.t = 0;
    V.lastTs = performance.now();
    V.raf = requestAnimationFrame(tick);
  }
}
function tick(now) {
  if (!V.playing || !timelineOn() || vmode !== "play") { setPlaying(false); return; }
  V.t += (now - V.lastTs) / 1000 * V.speed;
  V.lastTs = now;
  if (V.t >= rec.duration) { V.t = rec.duration; setPlaying(false); }
  $("v-seek").value = V.t;
  showAt(V.t);
  if (now - V.lastCursor > 50) { cursorToTime(V.t); V.lastCursor = now; }   // 그래프 커서는 20Hz
  if (V.playing) V.raf = requestAnimationFrame(tick);
}

function resetTimeline() {
  setPlaying(false);
  displayTime = 0;
  sceneReady = false;
  $("scene-truth").hidden = true;
  V.t = 0; V.shown = -1; V.want = -1;
  V.loading = false; V.request = null;
  $("v-img").removeAttribute("src");
  $("v-info").textContent = "";
  const ok = hasVideo();
  $("v-seek").max = rec ? (rec.duration || 1) : 1;
  $("v-seek").value = 0;
  $("v-empty").textContent = !rec ? "기록을 선택하세요"
    : ok ? "" : "이 기록에는 compressed 영상이 없습니다";
  $("v-play").disabled = $("v-seek").disabled = !videoOn();
  if (ok && layout === "split") showFrameAt(0);
}

function applyLayout() {
  const is3d = layout === "3d";
  document.body.classList.toggle("split", layout !== "graphs");
  document.body.classList.toggle("follow", vmode === "follow");
  $("video-pane").hidden = layout === "graphs";
  $("scene-pane").hidden = !is3d;
  $("video-frame").hidden = is3d;
  $("v-play").disabled = $("v-seek").disabled = !timelineOn();
  document.querySelectorAll("#layout button").forEach((b) => b.classList.toggle("on", b.dataset.l === layout));
  document.querySelectorAll("#vmode button").forEach((b) => b.classList.toggle("on", b.dataset.m === vmode));
  if (!timelineOn() || vmode !== "play") setPlaying(false);
  if (vmode === "play") displayTime = V.t;
  if (is3d) updateScene();
  if (videoOn()) showAt(displayTime);
}

function applySidebar() {
  document.body.classList.toggle("sidebar-collapsed", sidebarCollapsed);
  $("btn-sidebar").setAttribute("aria-expanded", String(!sidebarCollapsed));
}

// ---------------------------------------------------------------- 시작
function bind() {
  $("btn-sidebar").onclick = () => {
    sidebarCollapsed = !sidebarCollapsed;
    saveLS("pl_analyzer.sidebar_collapsed", sidebarCollapsed);
    applySidebar();
  };
  document.querySelectorAll("#layout button").forEach((b) => b.onclick = () => {
    layout = b.dataset.l; saveLS("pl_analyzer.layout", layout); applyLayout();
  });
  document.querySelectorAll("#vmode button").forEach((b) => b.onclick = () => {
    vmode = b.dataset.m; saveLS("pl_analyzer.vmode", vmode); applyLayout();
  });
  $("scene-center").value = sceneCenter;
  $("scene-center").onchange = () => {
    sceneCenter = $("scene-center").value;
    saveLS("pl_analyzer.scene_center", sceneCenter);
    showAt(displayTime);
  };
  $("v-play").onclick = () => setPlaying(!V.playing);
  $("v-seek").oninput = () => { setPlaying(false); seekTimeline(parseFloat($("v-seek").value)); };
  $("v-speed").onchange = () => { V.speed = parseFloat($("v-speed").value); };
  document.addEventListener("keydown", (e) => {   // Space = 재생/일시정지 (입력창 밖에서만)
    if (e.code !== "Space" || !timelineOn() || vmode !== "play") return;
    if (/^(INPUT|SELECT|TEXTAREA|BUTTON)$/.test(document.activeElement?.tagName)) return;
    e.preventDefault();
    setPlaying(!V.playing);
  });
  applyLayout();
  resetTimeline();
  applySidebar();
  $("btn-custom").onclick = addCustom;
  $("custom-input").onkeydown = (e) => { if (e.key === "Enter") addCustom(); };
  window.addEventListener("hashchange", () => {
    const id = decodeURIComponent(location.hash.slice(1));
    if (id && id !== currentId && records.some((r) => r.id === id)) openRecord(id);
  });
  let raf = 0;
  new ResizeObserver(() => {
    cancelAnimationFrame(raf);
    raf = requestAnimationFrame(() => {
      const w = chartWidth();
      for (const c of charts) if (c.width !== w) c.setSize({ width: w, height: CHART_H });
    });
  }).observe($("graphs"));
}

(async function init() {
  bind();
  try {
    await loadRecords();
  } catch (error) {
    $("rec-meta").textContent = error.message;
  }
})();

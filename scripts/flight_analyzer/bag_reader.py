"""Read ROS 2 SQLite bags and index HUD frames without a ROS installation."""
import json
import math
import sqlite3
from contextlib import closing
from dataclasses import fields, is_dataclass
from pathlib import Path

import numpy as np

MSG_DIR = Path(__file__).resolve().parents[2] / "src/px4_msgs/msg"

# 영상은 그래프 대상이 아니고 무거우므로 역직렬화하지 않는다(프레임 색인만 만든다).
SKIP_TYPES = {"sensor_msgs/msg/CompressedImage", "sensor_msgs/msg/Image"}
VIDEO_TYPE = "sensor_msgs/msg/CompressedImage"
VIDEO_TOPIC = "/pl/monitor/image/compressed"
LEGACY_TOPICS = {
    "/camera/camera_info": "/pl/camera/camera_info",
    "/target_pose": "/pl/target_pose",
    "/target_pose_world": "/pl/target_pose_world",
}
RATE_TOPICS = {
    "/pl/camera/camera_info": "camera",
    "/pl/detection/status": "detection",
}
CONTROL_SETPOINT = "/fmu/in/trajectory_setpoint"


def rate_series(timestamps, start, end):
    """Count arrivals in one-second bins on the bag timeline."""
    duration = (end - start) / 1e9
    widths = [min(1.0, duration - i) for i in range(math.ceil(duration))]
    fields = {}
    if not widths:
        return {"t": [], "fields": fields}
    for stream, times in timestamps.items():
        counts = [0] * len(widths)
        for stamp in times:
            index = min((stamp - start) // 1_000_000_000, len(counts) - 1)
            counts[index] += 1
        fields[stream] = [count / width for count, width in zip(counts, widths)]
    return {"t": [i + width / 2 for i, width in enumerate(widths)], "fields": fields}



# 평탄화에서 버릴 필드 (시각은 bag 수신시각으로 통일, MultiArray layout 은 잡음).
SKIP_FIELDS = {"header", "layout"}
MAX_ARRAY = 16

_typestore = None


def typestore(msg_dir=None):
    """Load standard ROS types and any available PX4 message definitions."""
    global _typestore
    if _typestore is None:
        from rosbags.typesys import Stores, get_types_from_msg, get_typestore
        ts = get_typestore(Stores.ROS2_HUMBLE)
        add = {}
        for f in sorted((msg_dir or MSG_DIR).rglob("*.msg")):
            add.update(get_types_from_msg(f.read_text(), f"px4_msgs/msg/{f.stem}"))
        ts.register(add)
        _typestore = ts
    return _typestore


def _is_rosbag_db(db: Path) -> bool:
    """rosbag2 sqlite DB인지 확인한다. 다른 .db3(예: PX4 parameter DB)는 제외한다."""
    try:
        with closing(_connect(db)) as con:
            tables = {row[0] for row in con.execute(
                "SELECT name FROM sqlite_master WHERE type = 'table'")}
        return {"topics", "messages"}.issubset(tables)
    except sqlite3.Error:
        return False


def find_db3(record: Path) -> list:
    """기록 폴더 안의 rosbag2 SQLite DB들만 찾는다.

    최상단의 PX4/기타 `.db3`가 `bag/` 아래 rosbag을 가리는 일이 없도록 두 위치를
    모두 검사하고, rosbag 필수 테이블(topics/messages)로 파일 종류를 판별한다.
    """
    if record.is_file():
        return [record] if record.suffix == ".db3" and _is_rosbag_db(record) else []
    try:
        out = sorted({*record.glob("*.db3"), *record.glob("*/*.db3")})
    except OSError:
        return []
    return [p for p in out if record.resolve() in p.resolve().parents
            and p.is_file() and _is_rosbag_db(p)]


def _connect(db: Path):
    # 읽기 전용 — 녹화 중인 bag 을 열어도 건드리지 않는다.
    return sqlite3.connect(db.resolve().as_uri() + "?mode=ro", uri=True)


def _scalar(v):
    """JSON 에 담을 스칼라로. NaN/inf → None, bool → 0/1."""
    if isinstance(v, (bool, np.bool_)):
        return int(v)
    if isinstance(v, (int, np.integer)):
        return int(v)
    if isinstance(v, (float, np.floating)):
        f = float(v)
        return f if math.isfinite(f) else None
    return v


def flatten(msg, prefix: str = "") -> dict:
    """rosbags 메시지(dataclass) → {열이름: 스칼라|문자열}.

    중첩 메시지는 prefix_ 로 재귀, 원시 배열은 name_0.. 로 전개(최대 16),
    메시지 배열은 개수(name_count) + 첫 원소(name_0_*) 만 남긴다.
    """
    out = {}
    for f in fields(msg):
        name = f.name
        if name in SKIP_FIELDS or name.startswith("__"):   # rosbags 내부 필드(__msgtype__)
            continue
        val = getattr(msg, name)
        key = prefix + name
        if is_dataclass(val):
            out.update(flatten(val, key + "_"))
        elif isinstance(val, str):
            out[key] = val
        elif isinstance(val, (np.ndarray, list, tuple)):
            seq = list(val)
            if seq and is_dataclass(seq[0]):
                out[key + "_count"] = len(seq)
                out.update(flatten(seq[0], key + "_0_"))
            elif not seq and isinstance(val, list):
                out[key + "_count"] = 0
            else:
                for i, v in enumerate(seq[:MAX_ARRAY]):
                    out[f"{key}_{i}"] = _scalar(v)
        else:
            out[key] = _scalar(val)
    return out


def flatten_json(value, prefix=""):
    """Expose the JSON status fields as chart columns."""
    if isinstance(value, dict):
        return {key: item for name, child in value.items()
                for key, item in flatten_json(child, prefix + name + "_").items()}
    if isinstance(value, list):
        return {key: item for index, child in enumerate(value[:MAX_ARRAY])
                for key, item in flatten_json(child, prefix + str(index) + "_").items()}
    return {prefix.removesuffix("_"): _scalar(value)}


def load_record(record: Path) -> dict:
    """기록 하나를 토픽별 열(column) 데이터로 읽는다.

    반환:
      {"duration", "errors": [...],
       "topics": {토픽: {"type", "t": [초...],
                         "fields": {열: [수치|None...]},
                         "strings": {열: [문자열...]}}}}
    t 는 기록 전체의 첫 메시지(영상 포함) 기준 상대초 — 모든 토픽이 같은 시간축을 쓴다.
    """
    ts = typestore()
    dbs = find_db3(record)
    if not dbs:
        raise FileNotFoundError(f".db3 가 없음: {record}")

    rows = {}      # 토픽 → [(t_ns, flat dict)]
    types = {}
    errors = {}
    frames = []    # (t_ns, db path, message id), indexed without loading the frame
    timestamps = {}
    t0 = t_end = None
    for db in dbs:
        with closing(_connect(db)) as con:
            lo, hi = con.execute("SELECT MIN(timestamp), MAX(timestamp) FROM messages").fetchone()
            if lo is not None:
                t0 = lo if t0 is None else min(t0, lo)
                t_end = hi if t_end is None else max(t_end, hi)
            topics = {tid: (LEGACY_TOPICS.get(name, name), ty) for tid, name, ty in
                      con.execute("SELECT id, name, type FROM topics")}
            for tid, (name, ty) in topics.items():
                if name == VIDEO_TOPIC and ty == VIDEO_TYPE:
                    frames.extend(
                        (t_ns, str(db), mid) for mid, t_ns in con.execute(
                            "SELECT id, timestamp FROM messages WHERE topic_id = ?", (tid,)))
            want = [tid for tid, (_, ty) in topics.items() if ty not in SKIP_TYPES]
            if not want:
                continue
            q = ("SELECT topic_id, timestamp, data FROM messages WHERE topic_id IN (%s) "
                 "ORDER BY timestamp" % ",".join("?" * len(want)))
            for tid, t_ns, data in con.execute(q, want):
                name, ty = topics[tid]
                stream = RATE_TOPICS.get(name)
                if name == CONTROL_SETPOINT or name.startswith(CONTROL_SETPOINT + "_v"):
                    stream = "control"
                if stream:
                    timestamps.setdefault(stream, []).append(t_ns)
                if name in errors:
                    continue
                try:
                    msg = ts.deserialize_cdr(bytes(data), ty)
                    if ty == "std_msgs/msg/String" and name.startswith("/pl/"):
                        flat = flatten_json(json.loads(msg.data))
                    else:
                        flat = flatten(msg)
                except Exception as e:  # 타입 미등록 / .msg 정의 불일치
                    errors[name] = f"{name} ({ty}): {type(e).__name__}: {e}"
                    rows.pop(name, None)
                    continue
                types[name] = ty
                rows.setdefault(name, []).append((t_ns, flat))

    if t0 is None:
        t0 = t_end = 0
    out_topics = {}
    for name, items in rows.items():
        items.sort(key=lambda item: item[0])
        # 행마다 키가 다를 수 있어(검출 0/1개 등) 키 합집합으로 열을 만든다.
        keys = []
        seen = set()
        for _, flat in items:
            for k in flat:
                if k not in seen:
                    seen.add(k)
                    keys.append(k)
        num, strs = {}, {}
        for k in keys:
            col = [flat.get(k) for _, flat in items]
            if any(isinstance(v, str) for v in col):
                strs[k] = ["" if v is None else str(v) for v in col]
            else:
                num[k] = col
        out_topics[name] = {
            "type": types[name],
            "t": [(t - t0) / 1e9 for t, _ in items],
            "fields": num,
            "strings": strs,
        }
    hud_frames = sorted(frames)
    if hud_frames:
        timestamps["viz"] = [t for t, _, _ in hud_frames]
    return {
        "output_rates": rate_series(timestamps, t0, t_end),
        "duration": (t_end - t0) / 1e9,
        "topics": out_topics,
        "video": {"topic": VIDEO_TOPIC, "t": [(t - t0) / 1e9 for t, _, _ in hud_frames]},
        "errors": list(errors.values()),
        "_frames": [(db, mid) for _, db, mid in hud_frames],
    }


def read_frame(db: str, msg_id: int):
    """색인된 프레임 1장 → (이미지 바이트, mimetype)."""
    con = _connect(Path(db))
    try:
        row = con.execute(
            "SELECT m.data, t.type FROM messages m JOIN topics t ON m.topic_id = t.id "
            "WHERE m.id = ?", (msg_id,)).fetchone()
    finally:
        con.close()
    if row is None:
        raise KeyError(msg_id)
    msg = typestore().deserialize_cdr(bytes(row[0]), row[1])
    fmt = (msg.format or "").lower()
    mime = "image/png" if "png" in fmt else "image/jpeg"
    return bytes(msg.data), mime

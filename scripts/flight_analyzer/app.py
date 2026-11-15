"""Browse recorded precision landings in a web browser."""
import argparse
import os
import shutil
import subprocess
import tempfile
import threading
from collections import OrderedDict
from datetime import datetime
from pathlib import Path
from statistics import median

from flask import Flask, Response, abort, jsonify, request, send_file

import bag_reader

HERE = Path(__file__).resolve().parent
DEFAULT_LOG_ROOT = HERE.parents[1] / "flight_logs"

app = Flask(__name__, static_folder="static", static_url_path="/static")
state = {"root": None}
_cache = OrderedDict()                 # (경로, mtime, 크기) → 로드 결과. 최근 4개만 유지
_cache_lock = threading.Lock()


def _root() -> Path:
    if state["root"] is None:
        abort(409, "데이터 폴더가 선택되지 않음")
    return state["root"]


def _record_path(rec_id: str) -> Path:
    """사이드바 id(데이터 폴더 기준 상대경로) → 실제 경로. 폴더 밖 접근 차단."""
    root = _root()
    p = (root / rec_id).resolve() if rec_id not in ("", ".") else root
    if p != root and root not in p.parents:
        abort(400, "데이터 폴더 밖 경로")
    if not p.exists():
        abort(404, f"기록 없음: {rec_id}")
    return p


def _records(root: Path) -> list:
    """기록 목록은 디렉터리·파일 정보만으로 빠르게 만든다.

    이전에는 목록을 열 때마다 모든 bag DB를 열어 topic별 COUNT/MIN/MAX를 계산했다.
    기록 수가 많으면 첫 화면만 보는데도 수 초가 걸린다. 목록에는 폴더명과 DB 크기만
    싣고, 길이·토픽 수·제어 메시지 수는 사용자가 기록을 선택한 뒤 load_record()에서
    계산한다.
    """
    def db_files(record: Path) -> list[Path]:
        # auto_save 형식(<record>/bag/*.db3)과, 복사한 일반 rosbag 디렉터리 모두 지원.
        candidates = [record / "bag", record]
        return [p for directory in candidates if directory.is_dir()
                for p in directory.glob("*.db3") if p.is_file()]

    def looks_like_record(record: Path) -> bool:
        files = db_files(record)
        if not files:
            return False
        # 표준 rosbag은 metadata.yaml을 갖고, auto_save 기록은 bag/ 아래 DB를 둔다.
        # metadata가 없는 강제 종료 bag도 auto_save 폴더 구조이면 목록에 남긴다.
        return (record / "metadata.yaml").is_file() or (record / "bag").is_dir() or \
            (record / "manifest.json").is_file()

    def metadata_summary(record: Path) -> tuple[str | None, float | None]:
        """rosbag2의 작은 metadata.yaml에서 시작 시각·길이만 읽는다.

        YAML 파서를 추가하지 않고 rosbag2가 고정 형식으로 쓰는 두 nanosecond 필드만
        찾는다. metadata가 없는 미정상 종료 bag은 목록에는 남기되 두 값만 비워 둔다.
        """
        metadata_paths = (record / "bag" / "metadata.yaml", record / "metadata.yaml")
        path = next((p for p in metadata_paths if p.is_file()), None)
        if path is None:
            return None, None
        section = None
        start_ns = duration_ns = None
        try:
            for raw in path.read_text(encoding="utf-8", errors="replace").splitlines():
                line = raw.strip()
                if line == "starting_time:":
                    section = "starting_time"
                elif line == "duration:":
                    section = "duration"
                elif section == "starting_time" and line.startswith("nanoseconds_since_epoch:"):
                    start_ns = int(line.split(":", 1)[1].strip())
                elif section == "duration" and line.startswith("nanoseconds:"):
                    duration_ns = int(line.split(":", 1)[1].strip())
        except (OSError, ValueError):
            return None, None
        start = (datetime.fromtimestamp(start_ns / 1e9).isoformat(timespec="seconds")
                 if start_ns is not None else None)
        duration = duration_ns / 1e9 if duration_ns is not None else None
        return start, duration

    candidates = [root] if looks_like_record(root) else []
    candidates.extend(
        d for d in root.iterdir()
        if d.is_dir() and d.resolve().parent == root
        and not d.name.startswith(".") and looks_like_record(d))
    out = []
    for d in candidates:
        files = db_files(d)
        start, duration = metadata_summary(d)
        out.append({
            "id": "." if d == root else d.relative_to(root).as_posix(),
            "name": d.name,
            "size": sum(p.stat().st_size for p in files),
            "start": start,
            "duration": duration,
        })
    out.sort(key=lambda r: r["name"], reverse=True)   # <YYYYmmdd_HHMMSS> → 최신이 위
    return out


# ---------------------------------------------------------------- pages
@app.get("/")
def index():
    return app.send_static_file("index.html")


# ---------------------------------------------------------------- api
@app.get("/api/records")
def records():
    root = _root()
    return jsonify({"root": str(root), "records": _records(root)})


def _load_cached(p: Path) -> dict:
    """기록 로드(최근 4개 캐시). 캐시 키에 db 파일 mtime/크기를 넣어 녹화 중 갱신도 반영."""
    dbs = bag_reader.find_db3(p)
    key = (str(p), tuple((str(d), d.stat().st_mtime_ns, d.stat().st_size) for d in dbs))
    with _cache_lock:
        if key in _cache:
            _cache.move_to_end(key)
            return _cache[key]
    data = bag_reader.load_record(p)
    data["name"] = p.name
    with _cache_lock:
        _cache[key] = data
        while len(_cache) > 4:
            _cache.popitem(last=False)
    return data


@app.get("/api/frame")
def frame():
    """화면 영상 i번째 프레임(JPEG). 뷰어가 재생/커서 위치에 맞춰 한 장씩 요청한다."""
    p = _record_path(request.args.get("id", ""))
    try:
        idx = int(request.args.get("i", "0"))
        data = _load_cached(p)
        if idx < 0:
            abort(404)
        db, mid = data["_frames"][idx]
        img, mime = bag_reader.read_frame(db, mid)
    except (ValueError, IndexError, KeyError, FileNotFoundError):
        abort(404)
    return Response(img, mimetype=mime, headers={"Cache-Control": "no-cache"})


def _encode_hud(ffmpeg: str, data: dict, directory: Path) -> Path:
    """Encode HUD frames with variable timing relative to the first frame."""
    times = data["video"]["t"]
    ticks = [round((t - times[0]) * 1000) for t in times]
    gaps = [b - a for a, b in zip(ticks, ticks[1:]) if b > a]
    tail = max(1, round(median(gaps))) if gaps else 33
    lines = ["ffconcat version 1.0"]
    for i, ((db, mid), tick) in enumerate(zip(data["_frames"], ticks)):
        end = ticks[i + 1] if i + 1 < len(ticks) else tick + tail
        if end == tick:
            continue
        image, mime = bag_reader.read_frame(db, mid)
        name = f"{i}.{'png' if mime == 'image/png' else 'jpg'}"
        (directory / name).write_bytes(image)
        lines.extend((f"file {name}", "option framerate 1000", f"duration {(end - tick) / 1000:.3f}"))
    # concat uses the following frame timestamp to establish the last hold.
    lines.extend((f"file {name}", "option framerate 1000"))
    (directory / "frames.ffconcat").write_text("\n".join(lines) + "\n")
    output = directory / "hud.mp4"
    subprocess.run([
        ffmpeg, "-nostdin", "-hide_banner", "-loglevel", "error",
        "-f", "concat", "-safe", "0", "-i", "frames.ffconcat",
        "-an", "-vsync", "vfr", "-enc_time_base", "1:1000",
        "-vf", "pad=ceil(iw/2)*2:ceil(ih/2)*2",
        "-c:v", "libx264", "-preset", "veryfast", "-pix_fmt", "yuv420p",
        "-movflags", "+faststart", output.name,
    ], cwd=directory, check=True, capture_output=True)
    return output


@app.get("/api/video.mp4")
def download_video():
    p = _record_path(request.args.get("id", ""))
    try:
        data = _load_cached(p)
    except FileNotFoundError as error:
        abort(404, str(error))
    if not data["_frames"]:
        abort(404, "이 기록에는 저장할 HUD 영상이 없습니다.")
    ffmpeg = shutil.which("ffmpeg")
    if not ffmpeg:
        abort(503, "MP4 저장을 사용하려면 뷰어를 실행하는 컴퓨터에 FFmpeg를 설치하세요.")
    with tempfile.TemporaryDirectory(prefix="pl-hud-") as temp:
        try:
            output = _encode_hud(ffmpeg, data, Path(temp))
        except (OSError, subprocess.CalledProcessError, KeyError) as error:
            app.logger.error("HUD export failed for %s: %s", p.name, getattr(error, "stderr", None) or error)
            abort(500, "HUD 영상을 MP4로 변환하지 못했습니다.")
        # The open file remains readable after cleanup on macOS and Linux.
        size = output.stat().st_size
        video = output.open("rb")
    response = send_file(video, mimetype="video/mp4", as_attachment=True,
                         download_name=f"{p.name}-hud.mp4", conditional=False)
    response.content_length = size
    return response


@app.get("/api/record")
def record():
    p = _record_path(request.args.get("id", ""))
    try:
        data = _load_cached(p)
    except FileNotFoundError as e:
        return jsonify({"error": str(e)}), 404
    payload = {k: v for k, v in data.items() if not k.startswith("_")}
    config = p / "real.yaml"
    if p.name == "bag" and p.parent.is_relative_to(_root()):
        config = p.parent / "real.yaml"
    if config.resolve().parent == config.parent and config.is_file():
        try:
            payload["real_yaml"] = config.read_text(encoding="utf-8")
        except (OSError, UnicodeError):
            pass
    return jsonify(payload)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=5050)
    ap.add_argument("--log-dir", type=Path, default=os.environ.get("PL_LOG_DIR", DEFAULT_LOG_ROOT))
    ap.add_argument("--px4-msg-dir", type=Path, help="Matching px4_msgs/msg directory")
    args = ap.parse_args()

    state["root"] = args.log_dir.expanduser().resolve()
    if state["root"] == DEFAULT_LOG_ROOT:
        state["root"].mkdir(exist_ok=True)
    if not state["root"].is_dir():
        ap.error(f"Log directory not found: {state['root']}")
    msg_dir = args.px4_msg_dir.expanduser() if args.px4_msg_dir else None
    if msg_dir is not None and not msg_dir.is_dir():
        ap.error(f"PX4 message directory not found: {msg_dir}")
    bag_reader.typestore(msg_dir)
    print(f"Precision Landing Flight Analyzer: http://{args.host}:{args.port}/")
    app.run(host=args.host, port=args.port, threaded=True)


if __name__ == "__main__":
    main()

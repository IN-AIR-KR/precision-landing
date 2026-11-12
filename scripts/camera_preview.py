#!/usr/bin/env python3
"""CSI/USB 카메라 영상 확인용 미리보기 예제.

cam0(CSI 0번 포트 = /dev/video0)에 연결된 카메라가 실제로 프레임을 내보내는지
눈으로 확인하기 위한 독립 실행 스크립트다. ROS 없이 OpenCV + GStreamer만 쓴다.

사용 예:
    python3 scripts/camera_preview.py                  # cam0, 자동 백엔드, GUI 창
    python3 scripts/camera_preview.py --backend argus  # Jetson ISP 경로만 시도
    python3 scripts/camera_preview.py --no-gui         # SSH 등 화면 없을 때 FPS만 출력
    python3 scripts/camera_preview.py --device /dev/video1 --backend v4l2

키: q 또는 ESC 종료, s 스냅샷 저장(--save-dir, 기본 log/).

백엔드를 열 때는 자식 프로세스로 먼저 한 장을 받아본다. 드라이버가 프레임을
주지 않으면 read()가 무한 대기하기 때문에, --open-timeout 으로 끊고 다음
백엔드로 넘어가기 위해서다.
"""
import argparse
import os
import subprocess
import sys
import time
from pathlib import Path

import cv2

ROOT = Path(__file__).resolve().parent.parent


def argus_pipeline(a):
    """Jetson ISP(libargus) 경로. nvarguscamerasrc 플러그인이 설치돼 있어야 한다."""
    return (
        f"nvarguscamerasrc sensor-id={a.sensor_id} ! "
        f"video/x-raw(memory:NVMM),width={a.width},height={a.height},"
        f"framerate={a.fps}/1 ! "
        f"nvvidconv flip-method={a.flip_method} ! "
        f"video/x-raw,format=BGRx ! videoconvert ! video/x-raw,format=BGR ! "
        f"appsink drop=true max-buffers=2 sync=false"
    )


def v4l2_pipeline(a):
    """USB 웹캠 등 이미 YUV를 내보내는 장치용."""
    return (
        f"v4l2src device={a.device} io-mode=2 ! "
        f"video/x-raw,width={a.width},height={a.height},framerate={a.fps}/1 ! "
        f"videoconvert ! video/x-raw,format=BGR ! "
        f"appsink drop=true max-buffers=2 sync=false"
    )


def bayer_pipeline(a):
    """ISP를 거치지 않고 VI에서 받은 베이어 영상을 소프트웨어로 디베이어."""
    return (
        f"v4l2src device={a.device} io-mode=2 ! "
        f"video/x-bayer,format=bggr,width={a.width},height={a.height},"
        f"framerate={a.fps}/1 ! "
        f"bayer2rgb ! videoconvert ! video/x-raw,format=BGR ! "
        f"appsink drop=true max-buffers=2 sync=false"
    )


def test_pipeline(a):
    """카메라 없이 창/표시 경로만 확인할 때 쓰는 합성 패턴(auto 에서는 시도하지 않음)."""
    return (
        f"videotestsrc is-live=true ! "
        f"video/x-raw,width={a.width},height={a.height},framerate={a.fps}/1 ! "
        f"videoconvert ! video/x-raw,format=BGR ! "
        f"appsink drop=true max-buffers=2 sync=false"
    )


BACKENDS = {'argus': argus_pipeline, 'v4l2': v4l2_pipeline, 'bayer': bayer_pipeline,
            'test': test_pipeline}
AUTO_BACKENDS = ['argus', 'v4l2', 'bayer']
# 플러그인이 없으면 OpenCV 의 GStreamer 백엔드가 그냥 멈춰버리므로 먼저 확인한다.
REQUIRED_ELEMENTS = {'argus': ['nvarguscamerasrc', 'nvvidconv'], 'v4l2': ['v4l2src'],
                     'bayer': ['v4l2src', 'bayer2rgb'], 'test': ['videotestsrc']}


def missing_elements(name):
    missing = []
    for element in REQUIRED_ELEMENTS[name]:
        done = subprocess.run(['gst-inspect-1.0', element], capture_output=True)
        if done.returncode != 0:
            missing.append(element)
    return missing


def probe(name, a):
    """자식 프로세스에서 백엔드 하나를 열어 첫 프레임을 받아본다."""
    missing = missing_elements(name)
    if missing:
        return False, f"GStreamer 플러그인 없음: {', '.join(missing)}"
    cmd = [sys.executable, str(Path(__file__).resolve()), '--probe', name,
           '--device', a.device, '--sensor-id', str(a.sensor_id),
           '--width', str(a.width), '--height', str(a.height),
           '--fps', str(a.fps), '--flip-method', str(a.flip_method)]
    try:
        done = subprocess.run(cmd, timeout=a.open_timeout, capture_output=True, text=True)
    except subprocess.TimeoutExpired:
        return False, f'{a.open_timeout:g}초 안에 프레임이 오지 않음'
    if done.returncode == 0:
        return True, 'ok'
    tail = [l for l in (done.stdout + done.stderr).splitlines() if l.strip()]
    return False, tail[-1] if tail else f'종료 코드 {done.returncode}'


def run_probe(name, a):
    """--probe 전용 경로: 한 장만 받아보고 성공 여부를 종료 코드로 알린다."""
    cap = cv2.VideoCapture(BACKENDS[name](a), cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        print(f'{name}: 파이프라인을 열지 못함', file=sys.stderr)
        return 1
    ok, frame = cap.read()
    cap.release()
    if not ok or frame is None:
        print(f'{name}: 프레임 없음', file=sys.stderr)
        return 1
    print(f'{name}: {frame.shape[1]}x{frame.shape[0]}')
    return 0


def hint(device):
    print(
        '\n[camera_preview] 어떤 백엔드로도 프레임을 받지 못했다.\n'
        f'  - 장치 확인:   v4l2-ctl --list-devices\n'
        f'  - 지원 포맷:   v4l2-ctl -d {device} --list-formats-ext\n'
        '  - CSI 센서가 베이어(BG10)만 내보내면 Jetson ISP 경로가 필요하다.\n'
        '    nvarguscamerasrc 가 없으면: sudo apt install nvidia-l4t-gstreamer\n'
        "    (설치 후 'gst-inspect-1.0 nvarguscamerasrc' 로 확인)\n",
        file=sys.stderr,
    )


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--backend', choices=['auto', *BACKENDS], default='auto',
                   help="auto 는 argus→v4l2→bayer 순서. test 는 카메라 없이 창만 확인")
    p.add_argument('--sensor-id', type=int, default=0, help='CSI 포트 번호 (cam0 = 0)')
    p.add_argument('--device', default='/dev/video0', help='v4l2/bayer 백엔드가 쓸 장치 노드')
    p.add_argument('--width', type=int, default=1280)
    p.add_argument('--height', type=int, default=720)
    p.add_argument('--fps', type=int, default=30)
    p.add_argument('--flip-method', type=int, default=0, help='argus: 0 없음, 2 180도 회전')
    p.add_argument('--open-timeout', type=float, default=8.0, help='백엔드별 첫 프레임 대기 시간(초)')
    p.add_argument('--no-gui', action='store_true', help='창 없이 수신 프레임/FPS만 출력')
    p.add_argument('--duration', type=float, default=0.0, help='초 단위 자동 종료 (0이면 무한)')
    p.add_argument('--save-dir', default=str(ROOT / 'log'), help="스냅샷 저장 위치 ('s' 키)")
    p.add_argument('--probe', choices=list(BACKENDS), help=argparse.SUPPRESS)
    a = p.parse_args()

    if a.probe:
        return run_probe(a.probe, a)

    gui = not a.no_gui and bool(os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'))
    if not a.no_gui and not gui:
        print('[camera_preview] DISPLAY 가 없어 헤드리스 모드로 실행한다.', flush=True)

    chosen = None
    for name in (AUTO_BACKENDS if a.backend == 'auto' else [a.backend]):
        ok, why = probe(name, a)
        print(f"[camera_preview] {name}: {'사용 가능' if ok else why}", flush=True)
        if ok:
            chosen = name
            break
    if chosen is None:
        hint(a.device)
        return 1

    pipeline = BACKENDS[chosen](a)
    print(f'[camera_preview] {chosen} 파이프라인으로 재생: {pipeline}', flush=True)
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        print('[camera_preview] 파이프라인을 다시 열지 못했다.', file=sys.stderr)
        return 1

    window = 'camera_preview'
    if gui:
        cv2.namedWindow(window, cv2.WINDOW_AUTOSIZE)

    save_dir = Path(a.save_dir)
    start = last_log = time.monotonic()
    frames = fps = 0
    rc = 0
    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                print('[camera_preview] 프레임 수신 중단', file=sys.stderr)
                rc = 1
                break
            frames += 1
            now = time.monotonic()
            if now - last_log >= 1.0:
                fps = frames / (now - last_log)
                frames, last_log = 0, now
                if not gui:
                    print(f'[camera_preview] {chosen} {frame.shape[1]}x{frame.shape[0]} {fps:.1f} fps', flush=True)

            if gui:
                view = frame.copy()
                cv2.putText(view, f'{chosen}  {view.shape[1]}x{view.shape[0]}  {fps:.1f} fps',
                            (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.imshow(window, view)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord('q'), 27):
                    break
                if key == ord('s'):
                    save_dir.mkdir(parents=True, exist_ok=True)
                    path = save_dir / f'camera_preview_{time.strftime("%Y%m%d_%H%M%S")}.png'
                    cv2.imwrite(str(path), frame)
                    print(f'[camera_preview] 저장: {path}', flush=True)

            if a.duration and time.monotonic() - start >= a.duration:
                break
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        if gui:
            cv2.destroyAllWindows()
    return rc


if __name__ == '__main__':
    sys.exit(main())

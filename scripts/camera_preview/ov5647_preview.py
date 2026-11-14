#!/usr/bin/env python3
"""OV5647 CSI 카메라 라이브 뷰어 (OpenCV 창).

Jetson Orin Nano + OV5647 조합은 Argus(하드웨어 ISP)가 동작하지 않아
`/dev/video0`에서 10비트 raw Bayer(BG10)만 나옵니다. 그래서 일반적인
cv2.VideoCapture로는 열리지 않습니다. 이 스크립트는 v4l2-ctl이 stdout으로
흘리는 raw 프레임을 받아 소프트웨어 디베이어한 뒤 창에 띄웁니다.

설치·배경은 docs/csi_camera_ov5647.md 참고.

사용법:

    python3 scripts/camera_preview/ov5647_preview.py                  # 1280x720 @60fps
    python3 scripts/camera_preview/ov5647_preview.py --mode 1         # 1920x1080 @30fps
    python3 scripts/camera_preview/ov5647_preview.py --auto-exposure  # 밝기 자동 조절
    python3 scripts/camera_preview/ov5647_preview.py --no-aruco       # 마커 검출 끄기

프로젝트가 `.deps/python`에 쓰는 OpenCV는 `opencv-contrib-python-headless`라
창을 띄울 수 없습니다(`cv2.error: The function is not implemented`). 이미
`source scripts/env.sh` 한 셸에서 실행하면 그 경로를 PYTHONPATH에서 빼고
자동으로 다시 실행해, GTK3가 들어간 시스템 apt OpenCV(4.5.4)를 씁니다.
ArUco는 신(4.7+)·구(4.5) API를 모두 지원하므로 어느 쪽에서든 검출됩니다.

키:
    q / ESC  종료
    s        현재 프레임 PNG 저장
    a        자동 노출 토글
    [ / ]    노출 -/+ 20%
    - / =    게인 -/+ 20%
    g        컬러 / 흑백 전환
"""
import argparse
import os
import subprocess
import sys
import time
from pathlib import Path

import cv2
import numpy as np

# 드라이버의 sensor_mode 테이블 (docs/csi_camera_ov5647.md 4.2 참고)
# mode: (width, height, fps, bytesperline)  bytesperline=None 이면 width*2
MODES = {
    0: (2592, 1944, 15, None),
    1: (1920, 1080, 30, None),
    2: (1296, 972, 30, 2624),   # 2x2 binned, 스트라이드가 width*2 가 아님
    3: (640, 480, 90, None),
    4: (1280, 720, 60, None),   # binned, 16:9 crop
}


def reexec_without_headless_cv2():
    """`source scripts/env.sh` 한 셸에서 실행해도 동작하도록.

    .deps/python 의 OpenCV 는 headless 빌드라 창을 못 띄운다. 그 경로를
    PYTHONPATH 에서 빼고 자기 자신을 다시 실행해 시스템 OpenCV(GTK3)를 쓴다.
    """
    if os.environ.get('CSI_VIEW_REEXEC') or has_gui():
        return
    parts = [p for p in os.environ.get('PYTHONPATH', '').split(os.pathsep) if p]
    kept = [p for p in parts if '.deps/python' not in p]
    if len(kept) == len(parts):
        return                      # headless 인데 .deps/python 때문이 아니다
    env = dict(os.environ, CSI_VIEW_REEXEC='1')
    if kept:
        env['PYTHONPATH'] = os.pathsep.join(kept)
    else:
        env.pop('PYTHONPATH', None)
    print('headless OpenCV 를 우회해 시스템 OpenCV 로 다시 시작합니다.',
          file=sys.stderr)
    os.execve(sys.executable, [sys.executable, os.path.abspath(__file__)] + sys.argv[1:], env)


def make_detector(dict_id):
    """OpenCV 4.7+ 의 ArucoDetector 와 4.5 의 구 API 를 함께 지원한다."""
    if hasattr(cv2.aruco, 'ArucoDetector'):          # 4.7 이상
        det = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(dict_id),
            cv2.aruco.DetectorParameters())
        return lambda img: det.detectMarkers(img)
    # 4.5.x: 모듈 수준 함수 + Dictionary_get
    dictionary = cv2.aruco.Dictionary_get(dict_id)
    params = cv2.aruco.DetectorParameters_create()
    return lambda img: cv2.aruco.detectMarkers(img, dictionary, parameters=params)


def has_gui():
    """cv2 가 창을 띄울 수 있는 빌드인지 확인."""
    try:
        cv2.namedWindow('__probe__', cv2.WINDOW_NORMAL)
        cv2.destroyWindow('__probe__')
        return True
    except cv2.error:
        return False


def set_ctrl(device, **ctrls):
    """스트리밍 중에도 적용되는 V4L2 컨트롤 변경."""
    arg = ','.join(f'{k}={int(v)}' for k, v in ctrls.items())
    subprocess.run(['v4l2-ctl', '-d', device, f'--set-ctrl={arg}'],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)


def start_stream(device, mode, width, height, exposure, gain):
    """v4l2-ctl 을 띄워 raw 프레임을 stdout 으로 받는다."""
    cmd = [
        'v4l2-ctl', '-d', device,
        f'--set-ctrl=sensor_mode={mode},exposure={exposure},gain={gain}',
        f'--set-fmt-video=width={width},height={height},pixelformat=BG10',
        '--stream-mmap', '--stream-count=0', '--stream-to=-',
    ]
    return subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                            bufsize=0)


def read_exact(stream, nbytes, buf):
    """정확히 nbytes 를 채운다. 스트림이 끊기면 False."""
    view = memoryview(buf)
    got = 0
    while got < nbytes:
        n = stream.readinto(view[got:nbytes])
        if not n:
            return False
        got += n
    return True


def main():
    ap = argparse.ArgumentParser(description='OV5647 CSI 라이브 뷰어')
    ap.add_argument('--device', default='/dev/video0')
    ap.add_argument('--mode', type=int, default=4, choices=sorted(MODES),
                    help='sensor_mode (기본 4: 1280x720 @60fps)')
    ap.add_argument('--exposure', type=int, default=20000, help='노출, 마이크로초')
    ap.add_argument('--gain', type=int, default=128, help='게인, 1/16 단위 (16=1배)')
    ap.add_argument('--auto-exposure', action='store_true',
                    help='평균 밝기를 목표값에 맞추는 간단한 소프트웨어 AE')
    ap.add_argument('--target', type=int, default=110, help='AE 목표 평균 밝기 (0-255)')
    ap.add_argument('--gray', action='store_true', help='흑백으로 시작')
    ap.add_argument('--no-aruco', action='store_true', help='ArUco 검출 끄기')
    ap.add_argument('--dict', type=int, default=cv2.aruco.DICT_4X4_250,
                    help='ArUco 딕셔너리 ID (기본 DICT_4X4_250=2)')
    ap.add_argument('--scale', type=float, default=1.0, help='창 표시 배율')
    ap.add_argument('--outdir', default='.run', help='s 키로 저장할 디렉터리')
    args = ap.parse_args()

    if not Path(args.device).exists():
        sys.exit(f'{args.device} 가 없습니다. 카메라가 인식됐는지 확인하세요:\n'
                 '  sudo modprobe -r ov5647 && sudo modprobe ov5647\n'
                 '  (docs/csi_camera_ov5647.md 5.1 참고)')
    if not os.environ.get('DISPLAY') and not os.environ.get('WAYLAND_DISPLAY'):
        sys.exit('DISPLAY 가 없습니다. 데스크톱 세션에서 실행하세요.')
    reexec_without_headless_cv2()
    if not has_gui():
        sys.exit(f'이 OpenCV({cv2.__version__}, {cv2.__file__})는 GUI 없이 빌드됐습니다.\n'
                 'scripts/env.sh 를 source 하지 않은 새 셸에서 실행하세요.\n'
                 '(프로젝트의 .deps/python OpenCV 는 headless 빌드입니다)')

    width, height, fps, stride = MODES[args.mode]
    stride = stride or width * 2
    frame_bytes = stride * height

    detect = None
    if not args.no_aruco:
        if not hasattr(cv2, 'aruco'):
            print('경고: 이 OpenCV 에는 aruco 모듈이 없습니다. 검출을 끕니다.',
                  file=sys.stderr)
        else:
            detect = make_detector(args.dict)

    exposure, gain = args.exposure, args.gain
    auto = args.auto_exposure
    show_gray = args.gray
    max_exposure = int(0.95e6 / fps)   # 노출은 프레임 길이를 넘을 수 없다

    print(f'모드 {args.mode}: {width}x{height} @{fps}fps, 스트라이드 {stride}B')
    print(f'노출 {exposure}us, 게인 {gain}/16, AE {"켜짐" if auto else "꺼짐"}')
    print('종료: q 또는 ESC')

    proc = start_stream(args.device, args.mode, width, height, exposure, gain)
    buf = bytearray(frame_bytes)
    win = 'OV5647 CSI'
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, int(width * args.scale), int(height * args.scale))

    frames, t0, shown_fps = 0, time.monotonic(), 0.0
    last_ae = 0.0
    try:
        while True:
            if proc.poll() is not None:
                print('v4l2-ctl 가 종료됐습니다. 다른 프로세스가 카메라를 '
                      '쓰고 있는지 확인하세요.', file=sys.stderr)
                break
            if not read_exact(proc.stdout, frame_bytes, buf):
                print('스트림이 끊겼습니다.', file=sys.stderr)
                break

            raw = np.frombuffer(buf, dtype='<u2').reshape(height, stride // 2)
            raw = raw[:, :width]                       # 스트라이드 패딩 제거
            bayer8 = (raw >> 8).astype(np.uint8)       # R35+ 는 16비트로 스케일됨

            gray = cv2.cvtColor(bayer8, cv2.COLOR_BayerBG2GRAY)
            view = (cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR) if show_gray
                    else cv2.cvtColor(bayer8, cv2.COLOR_BayerBG2BGR))

            ids = None
            if detect is not None:
                corners, ids, _ = detect(gray)
                if ids is not None:
                    cv2.aruco.drawDetectedMarkers(view, corners, ids)

            mean = float(gray.mean())

            # 간단한 비례 제어 AE. Argus 가 없으므로 직접 맞춘다.
            now = time.monotonic()
            if auto and now - last_ae > 0.25:
                last_ae = now
                if abs(mean - args.target) > 6:
                    ratio = args.target / max(mean, 1.0)
                    ratio = min(max(ratio, 0.5), 2.0)      # 한 번에 과하게 안 움직이게
                    new_exp = int(min(max(exposure * ratio, 131), max_exposure))
                    if new_exp != exposure:
                        exposure = new_exp
                        set_ctrl(args.device, exposure=exposure)
                    elif mean < args.target:               # 노출 상한이면 게인으로
                        gain = int(min(gain * 1.2, 1023))
                        set_ctrl(args.device, gain=gain)
                    elif gain > 16:
                        gain = int(max(gain / 1.2, 16))
                        set_ctrl(args.device, gain=gain)

            frames += 1
            if now - t0 >= 1.0:
                shown_fps = frames / (now - t0)
                frames, t0 = 0, now

            n_ids = 0 if ids is None else len(ids)
            hud = (f'{width}x{height} m{args.mode} | {shown_fps:4.1f}fps | '
                   f'exp {exposure}us gain {gain} | mean {mean:5.1f}'
                   f'{" | AE" if auto else ""} | ArUco {n_ids}')
            cv2.rectangle(view, (0, 0), (view.shape[1], 28), (0, 0, 0), -1)
            cv2.putText(view, hud, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        (0, 255, 0), 1, cv2.LINE_AA)

            cv2.imshow(win, view)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):
                break
            elif key == ord('s'):
                out = Path(args.outdir)
                out.mkdir(parents=True, exist_ok=True)
                path = out / f'csi_{time.strftime("%Y%m%d_%H%M%S")}.png'
                cv2.imwrite(str(path), view)
                print(f'저장: {path}')
            elif key == ord('a'):
                auto = not auto
                print(f'AE {"켜짐" if auto else "꺼짐"}')
            elif key in (ord('['), ord(']')):
                exposure = int(exposure * (0.8 if key == ord('[') else 1.25))
                exposure = min(max(exposure, 131), max_exposure)
                set_ctrl(args.device, exposure=exposure)
            elif key in (ord('-'), ord('=')):
                gain = int(gain * (0.8 if key == ord('-') else 1.25))
                gain = min(max(gain, 16), 1023)
                set_ctrl(args.device, gain=gain)
            elif key == ord('g'):
                show_gray = not show_gray
    except KeyboardInterrupt:
        pass
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            proc.kill()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()

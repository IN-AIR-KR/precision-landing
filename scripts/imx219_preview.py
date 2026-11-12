#!/usr/bin/env python3
"""IMX219(Raspberry Pi Camera v2) 미리보기 — Jetson Orin Nano.

CAM0에 연결된 IMX219가 실제로 프레임을 내보내는지 눈으로 확인하는 예제다.
두 경로를 지원한다.

  argus : nvarguscamerasrc -> Jetson 하드웨어 ISP. 자동 노출·화이트밸런스가
          적용된 정상 컬러 영상이 나온다. nvidia-l4t-gstreamer 필요.
  raw   : v4l2-ctl 이 stdout 으로 흘리는 RG10(10비트 RGGB 베이어)을 받아
          소프트웨어 디베이어. ISP 없이도 동작하지만 노출을 직접 잡아야 한다.

사용 예:
    python3 scripts/imx219_preview.py                 # auto: argus -> raw
    python3 scripts/imx219_preview.py --backend raw   # ISP 없이 raw 경로
    python3 scripts/imx219_preview.py --mode 4        # 1280x720 @60fps
    python3 scripts/imx219_preview.py --no-aruco

키: q/ESC 종료, s 스냅샷 저장(.run/), g 흑백 전환,
    [ ] 노출 -/+, - = 게인 -/+ (raw 경로 전용)

주의 — raw 경로에서 반드시 override_enable=1 을 켜야 한다. 기본값 0 이면
Tegra VI 드라이버가 스트리밍 시작 시 노출·게인을 센서 모드 기본값으로
되돌려 버려서, exposure 를 아무리 바꿔도 밝기가 그대로다. 실측:

    override_enable=0 : exposure 4000/16000/33000us -> 평균 모두 12876 (변화 없음)
    override_enable=1 : exposure 4000/16000/33000us -> 평균 15536/39293/45711

OV5647 용 뷰어는 scripts/csi_view.py 를 쓴다(BG10/BGGR, 모드표가 다름).
배경은 docs/csi_camera_ov5647.md 참고.
"""
import argparse
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

import cv2
import numpy as np

# IMX219 (nv_imx219 드라이버) 의 sensor_mode 테이블
# mode: (width, height, fps)
MODES = {
    0: (3280, 2464, 21),
    1: (3280, 1848, 28),
    2: (1920, 1080, 30),
    3: (1640, 1232, 30),
    4: (1280, 720, 60),
}

# RG10 은 10비트 값을 16비트로 좌측 정렬(<<6)해 담는다. 블랙 레벨 64(10bit) = 4096.
BLACK_LEVEL = 4096
WHITE_LEVEL = 65535
GAIN_MIN, GAIN_MAX = 16, 171          # IMX219 는 OV5647(1023)보다 상한이 훨씬 낮다


def has_gui():
    try:
        cv2.namedWindow('__probe__', cv2.WINDOW_NORMAL)
        cv2.destroyWindow('__probe__')
        return True
    except cv2.error:
        return False


def reexec_without_headless_cv2():
    """.deps/python 의 headless OpenCV 를 피해 시스템 OpenCV(GTK3)로 재실행."""
    if os.environ.get('IMX219_REEXEC') or has_gui():
        return
    parts = [p for p in os.environ.get('PYTHONPATH', '').split(os.pathsep) if p]
    kept = [p for p in parts if '.deps/python' not in p]
    if len(kept) == len(parts):
        return
    env = dict(os.environ, IMX219_REEXEC='1')
    env['PYTHONPATH'] = os.pathsep.join(kept) if kept else ''
    if not kept:
        env.pop('PYTHONPATH', None)
    print('headless OpenCV 를 우회해 시스템 OpenCV 로 다시 시작합니다.', file=sys.stderr)
    os.execve(sys.executable, [sys.executable, os.path.abspath(__file__)] + sys.argv[1:], env)


def make_detector(dict_id):
    """OpenCV 4.7+ 신규 API 와 4.5 구 API 를 모두 지원."""
    if hasattr(cv2.aruco, 'ArucoDetector'):
        det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(dict_id),
                                      cv2.aruco.DetectorParameters())
        return lambda img: det.detectMarkers(img)
    dictionary = cv2.aruco.Dictionary_get(dict_id)
    params = cv2.aruco.DetectorParameters_create()
    return lambda img: cv2.aruco.detectMarkers(img, dictionary, parameters=params)


def have_argus():
    if not shutil.which('gst-inspect-1.0'):
        return False
    return subprocess.run(['gst-inspect-1.0', 'nvarguscamerasrc'],
                          stdout=subprocess.DEVNULL,
                          stderr=subprocess.DEVNULL).returncode == 0


def argus_pipeline(sensor_id, width, height, fps):
    return (f'nvarguscamerasrc sensor-id={sensor_id} ! '
            f'video/x-raw(memory:NVMM),width={width},height={height},'
            f'framerate={fps}/1,format=NV12 ! '
            'nvvidconv ! video/x-raw,format=BGRx ! '
            'videoconvert ! video/x-raw,format=BGR ! '
            'appsink drop=true max-buffers=2 sync=false')


def set_ctrl(device, **ctrls):
    arg = ','.join(f'{k}={int(v)}' for k, v in ctrls.items())
    subprocess.run(['v4l2-ctl', '-d', device, f'--set-ctrl={arg}'],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)


def start_raw_stream(device, mode, width, height, exposure, gain):
    """override_enable=1 이 핵심. 없으면 exposure/gain 이 무시된다."""
    cmd = ['v4l2-ctl', '-d', device,
           f'--set-ctrl=override_enable=1,sensor_mode={mode},'
           f'exposure={exposure},gain={gain}',
           f'--set-fmt-video=width={width},height={height},pixelformat=RG10',
           '--stream-mmap', '--stream-count=0', '--stream-to=-']
    return subprocess.Popen(cmd, stdout=subprocess.PIPE,
                            stderr=subprocess.DEVNULL, bufsize=0)


def read_exact(stream, nbytes, buf):
    view = memoryview(buf)
    got = 0
    while got < nbytes:
        n = stream.readinto(view[got:nbytes])
        if not n:
            return False
        got += n
    return True


def develop(raw16):
    """RG10 raw -> 8비트 BGR + 흑백. 블랙 레벨을 빼고 스케일한다."""
    x = raw16.astype(np.int32) - BLACK_LEVEL
    np.clip(x, 0, None, out=x)
    x = (x * 255) // (WHITE_LEVEL - BLACK_LEVEL)
    bayer8 = x.astype(np.uint8)
    return (cv2.cvtColor(bayer8, cv2.COLOR_BayerRG2BGR),
            cv2.cvtColor(bayer8, cv2.COLOR_BayerRG2GRAY))


def main():
    ap = argparse.ArgumentParser(description='IMX219 미리보기 (Jetson Orin Nano)')
    ap.add_argument('--backend', choices=['auto', 'argus', 'raw'], default='auto',
                    help='auto 는 argus 가 있으면 argus, 없으면 raw')
    ap.add_argument('--device', default='/dev/video0')
    ap.add_argument('--sensor-id', type=int, default=0, help='argus 백엔드의 sensor-id')
    ap.add_argument('--mode', type=int, default=2, choices=sorted(MODES),
                    help='sensor_mode (기본 2: 1920x1080 @30fps)')
    ap.add_argument('--exposure', type=int, default=20000,
                    help='raw 경로 노출, 마이크로초 (argus 는 자동)')
    ap.add_argument('--gain', type=int, default=100,
                    help=f'raw 경로 게인, 1/16 단위 ({GAIN_MIN}~{GAIN_MAX})')
    ap.add_argument('--gray', action='store_true')
    ap.add_argument('--no-aruco', action='store_true')
    ap.add_argument('--dict', type=int, default=cv2.aruco.DICT_4X4_250)
    ap.add_argument('--scale', type=float, default=0.75, help='창 표시 배율')
    ap.add_argument('--outdir', default='.run')
    args = ap.parse_args()

    if not Path(args.device).exists():
        sys.exit(f'{args.device} 가 없습니다. IMX219 오버레이로 부팅했는지 확인하세요.\n'
                 '  부팅 메뉴의 "Custom Header Config: <CSI Camera IMX219-A>"\n'
                 '  (docs/csi_camera_ov5647.md 참고)')
    if not os.environ.get('DISPLAY') and not os.environ.get('WAYLAND_DISPLAY'):
        sys.exit('DISPLAY 가 없습니다. 데스크톱 세션에서 실행하세요.')
    reexec_without_headless_cv2()
    if not has_gui():
        sys.exit(f'이 OpenCV({cv2.__version__})는 GUI 없이 빌드됐습니다.')

    width, height, fps = MODES[args.mode]
    backend = args.backend
    if backend == 'auto':
        backend = 'argus' if have_argus() else 'raw'
        if backend == 'raw':
            print('nvarguscamerasrc 가 없어 raw 경로로 갑니다. '
                  'ISP 를 쓰려면: sudo apt install nvidia-l4t-gstreamer', file=sys.stderr)
    if backend == 'argus' and not have_argus():
        sys.exit('nvarguscamerasrc 가 없습니다. nvidia-l4t-gstreamer 를 설치하세요.')

    detect = None if args.no_aruco or not hasattr(cv2, 'aruco') else make_detector(args.dict)
    show_gray = args.gray
    exposure, gain = args.exposure, min(max(args.gain, GAIN_MIN), GAIN_MAX)
    max_exposure = int(0.95e6 / fps)

    print(f'백엔드 {backend} | 모드 {args.mode}: {width}x{height} @{fps}fps')
    if backend == 'raw':
        print(f'노출 {exposure}us, 게인 {gain}/16 (override_enable=1)')
    print('종료: q 또는 ESC')

    win = 'IMX219'
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, int(width * args.scale), int(height * args.scale))

    cap = proc = None
    frame_bytes = width * height * 2
    buf = bytearray(frame_bytes)
    if backend == 'argus':
        cap = cv2.VideoCapture(argus_pipeline(args.sensor_id, width, height, fps),
                               cv2.CAP_GSTREAMER)
        if not cap.isOpened():
            sys.exit('argus 파이프라인을 열지 못했습니다. --backend raw 로 시도해 보세요.')
    else:
        proc = start_raw_stream(args.device, args.mode, width, height, exposure, gain)

    frames, t0, shown_fps = 0, time.monotonic(), 0.0
    try:
        while True:
            if backend == 'argus':
                ok, view = cap.read()
                if not ok:
                    print('프레임을 받지 못했습니다.', file=sys.stderr)
                    break
                gray = cv2.cvtColor(view, cv2.COLOR_BGR2GRAY)
                if show_gray:
                    view = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            else:
                if proc.poll() is not None:
                    print('v4l2-ctl 가 종료됐습니다. 다른 프로세스가 카메라를 '
                          '쓰고 있는지 확인하세요.', file=sys.stderr)
                    break
                if not read_exact(proc.stdout, frame_bytes, buf):
                    print('스트림이 끊겼습니다.', file=sys.stderr)
                    break
                raw = np.frombuffer(buf, dtype='<u2').reshape(height, width)
                color, gray = develop(raw)
                view = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR) if show_gray else color

            ids = None
            if detect is not None:
                corners, ids, _ = detect(gray)
                if ids is not None:
                    cv2.aruco.drawDetectedMarkers(view, corners, ids)

            frames += 1
            now = time.monotonic()
            if now - t0 >= 1.0:
                shown_fps = frames / (now - t0)
                frames, t0 = 0, now

            hud = f'{width}x{height} m{args.mode} {backend} | {shown_fps:4.1f}fps | '
            hud += ('ISP auto' if backend == 'argus'
                    else f'exp {exposure}us gain {gain}')
            hud += f' | mean {gray.mean():5.1f} | ArUco {0 if ids is None else len(ids)}'
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
                path = out / f'imx219_{time.strftime("%Y%m%d_%H%M%S")}.png'
                cv2.imwrite(str(path), view)
                print(f'저장: {path}')
            elif key == ord('g'):
                show_gray = not show_gray
            elif backend == 'raw' and key in (ord('['), ord(']')):
                exposure = int(exposure * (0.8 if key == ord('[') else 1.25))
                exposure = min(max(exposure, 13), max_exposure)
                set_ctrl(args.device, exposure=exposure)
            elif backend == 'raw' and key in (ord('-'), ord('=')):
                gain = int(gain * (0.8 if key == ord('-') else 1.25))
                gain = min(max(gain, GAIN_MIN), GAIN_MAX)
                set_ctrl(args.device, gain=gain)
    except KeyboardInterrupt:
        pass
    finally:
        if cap is not None:
            cap.release()
        if proc is not None:
            proc.terminate()
            try:
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                proc.kill()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()

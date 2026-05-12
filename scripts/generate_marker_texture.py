#!/usr/bin/env python3
"""
V-마커 텍스처 생성기

1024×1024 px PNG 파일을 생성한다.
실제 마커 크기: 3×3m → 1px = ~2.93mm

레이아웃:
  - 배경: 짙은 회색 (50, 50, 50)
  - 흰색 원형 테두리: outer r=490px, inner r=440px (테두리 두께 약 50px)
  - 흰색 V 형상: 두꺼운 두 직선이 아래 중심에서 만남
  - ArUco ID 0 (DICT_4X4_50): 중앙 배치, 171px ≈ 0.5m

사용법:
    python3 generate_marker_texture.py
    python3 generate_marker_texture.py --output /path/to/v_marker.png
    python3 generate_marker_texture.py --preview  # 화면에 바로 표시
"""

import argparse
import math
import os
import sys

import cv2
import cv2.aruco as aruco
import numpy as np


SIZE = 1024          # 텍스처 해상도 (px)
REAL_SIZE = 3.0      # 실제 마커 크기 (m)
PX_PER_M = SIZE / REAL_SIZE  # 341.3 px/m


def draw_circular_border(img: np.ndarray, cx: int, cy: int,
                          outer_r: int, inner_r: int, color: tuple):
    """흰색 원형 링 그리기"""
    cv2.circle(img, (cx, cy), outer_r, color, -1)
    bg_color = (int(img[0, 0, 0]),) * 3  # 배경색으로 내부 지우기
    cv2.circle(img, (cx, cy), inner_r, bg_color, -1)


def draw_v_shape(img: np.ndarray, cx: int, cy: int, color: tuple,
                  arm_length: int = 340, arm_width: int = 55):
    """
    V 형상 그리기.
    꼭짓점은 중앙 아래쪽, 두 팔은 위 좌우로 뻗어 나감.
    """
    vertex = (cx, cy + 160)
    left_tip  = (cx - arm_length, cy - arm_length // 3)
    right_tip = (cx + arm_length, cy - arm_length // 3)
    cv2.line(img, vertex, left_tip,  color, arm_width)
    cv2.line(img, vertex, right_tip, color, arm_width)


def embed_aruco(img: np.ndarray, cx: int, cy: int, marker_real_m: float = 0.5):
    """
    ArUco ID 0 (DICT_4X4_50) 마커를 이미지 중앙에 임베드.
    marker_real_m → px 로 환산하여 크기 결정.
    """
    marker_px = int(marker_real_m * PX_PER_M)
    # marker_px가 짝수가 되도록 조정 (중앙 정렬 편의)
    if marker_px % 2 != 0:
        marker_px += 1

    dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    marker_gray = np.zeros((marker_px, marker_px), dtype=np.uint8)
    aruco.generateImageMarker(dictionary, 0, marker_px, marker_gray, 1)
    marker_bgr = cv2.cvtColor(marker_gray, cv2.COLOR_GRAY2BGR)

    x0 = cx - marker_px // 2
    y0 = cy - marker_px // 2
    img[y0:y0 + marker_px, x0:x0 + marker_px] = marker_bgr


def generate(output_path: str, preview: bool = False):
    img = np.full((SIZE, SIZE, 3), 50, dtype=np.uint8)  # 짙은 회색 배경
    WHITE = (255, 255, 255)
    cx = cy = SIZE // 2

    draw_circular_border(img, cx, cy, outer_r=490, inner_r=440, color=WHITE)
    draw_v_shape(img, cx, cy, WHITE)
    embed_aruco(img, cx, cy, marker_real_m=0.5)

    if preview:
        cv2.imshow('V-Marker Texture Preview', cv2.resize(img, (600, 600)))
        print('미리보기 창을 닫으려면 아무 키나 누르세요...')
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    cv2.imwrite(output_path, img)
    print(f'생성 완료: {output_path}  ({SIZE}x{SIZE} px)')


def main():
    # 기본 출력 경로: 이 스크립트 기준 상대 경로
    script_dir = os.path.dirname(os.path.abspath(__file__))
    default_output = os.path.join(
        script_dir, '..', 'simulation', 'models', 'v_marker', 'v_marker.png')
    default_output = os.path.normpath(default_output)

    parser = argparse.ArgumentParser(description='V-마커 텍스처 생성')
    parser.add_argument('--output', default=default_output,
                        help=f'출력 PNG 경로 (기본: {default_output})')
    parser.add_argument('--preview', action='store_true',
                        help='생성 후 화면에 미리보기')
    args = parser.parse_args()

    generate(args.output, args.preview)


if __name__ == '__main__':
    main()

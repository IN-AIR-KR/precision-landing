# Live HUD streaming

`pl_streaming` subscribes to the Jetson's local BGR HUD (`/pl/monitor/image`),
encodes H.264 with GStreamer and publishes to MediaMTX over RTSP. On the real
profile the HUD is capped at 640 pixels wide, with a 30 Hz render target and
JPEG quality 45 for newly recorded bags. Camera input for detection stays at
its configured resolution. The JPEG HUD in the bag plays back independently
of the live stream.

## Jetson

Install the GStreamer plugins and introspection bindings, then build the real profile.

```bash
sudo apt install python3-gi gir1.2-gstreamer-1.0 gstreamer1.0-tools \
  gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-rtsp
./scripts/build.sh real
```

On the ground-station PC, install MediaMTX and run it on your test LAN with
the supplied configuration. From the repository root:

```bash
mediamtx src/pl_streaming/mediamtx.yml
```

On the Jetson, start the real profile with the ground station's LAN IP.

```bash
./scripts/run_real.sh stream_url:=rtsp://GROUND_STATION_IP:8554/hud
```

Open `http://localhost:8889/hud` in a browser on the ground station.
The publisher uses RTSP over UDP by default. MediaMTX must accept RTSP on TCP
8554 and RTP/RTCP on its configured UDP ports (defaults 8000/8001).
Browser playback uses WebRTC and its configured ICE listener.

The defaults are 640×360, up to 30 fps, 700 kbps, x264 ultrafast/zerolatency,
no B-frames, an IDR at most every 0.5 s at full frame rate, and one queued
raw frame in GStreamer. Keep the raw HUD subscriber on Jetson to avoid
sending the raw feed over DDS. The stream can also be started separately to
adjust bitrate, frame rate, size or try TCP when UDP is unavailable. In this
case start the real launch without `stream_url`.

```bash
source scripts/env.sh
export ROS_DOMAIN_ID=0  # Match real.yaml if customized.
ros2 run pl_streaming hud_streamer --ros-args \
  -p url:=rtsp://GROUND_STATION_IP:8554/hud -p rtsp_transport:=tcp
```

Start by comparing the video delay and detection throughput with live
streaming off and on. Verify the recorded JPEG frame intervals in a fresh
flight bag. Existing bags retain their original frame rate.

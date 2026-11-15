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

Install MediaMTX on the Jetson. The real launch looks for `mediamtx` on
`PATH` or at `.deps/hud/mediamtx` in this checkout. From the repository root:

```bash
./scripts/run_real.sh
```

The launch starts MediaMTX with the packaged configuration and publishes
HUD video to it at `rtsp://127.0.0.1:8554/hud`. Both processes stop with
the launch. To disable streaming for a run, pass `stream_url:=`.
If a MediaMTX server is already running, supply its RTSP publish URL:

```bash
./scripts/run_real.sh stream_url:=rtsp://127.0.0.1:8554/hud
```

On the ground station, open `http://JETSON_LAN_IP:8889/hud` in a browser
for WebRTC playback, or open `rtsp://JETSON_LAN_IP:8554/hud` in an RTSP player.
With the default MediaMTX ports, browser playback needs TCP 8889 and UDP 8189
reachable on the Jetson. For RTSP playback over TCP, allow TCP 8554.
The Jetson publisher uses RTSP over UDP on loopback by default.

The defaults are 640×360, up to 30 fps, 700 kbps, x264 ultrafast/zerolatency,
no B-frames, an IDR at most every 0.5 s at full frame rate, and one queued
raw frame in GStreamer. Keep the raw HUD subscriber on Jetson to avoid
sending the raw feed over DDS. The stream can also be started separately to
adjust bitrate, frame rate, size or try TCP when UDP is unavailable. For
this setup, start MediaMTX separately and pass `stream_url:=` to the real launch.

```bash
source scripts/env.sh
export ROS_DOMAIN_ID=0  # Match real.yaml if customized.
ros2 run pl_streaming hud_streamer --ros-args \
  -p url:=rtsp://127.0.0.1:8554/hud -p rtsp_transport:=tcp
```

Start by comparing the video delay and detection throughput with live
streaming off and on. Verify the recorded JPEG frame intervals in a fresh
flight bag. Existing bags retain their original frame rate.

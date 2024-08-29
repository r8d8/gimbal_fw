Jetson Nano pipeline with [nvstabilize](https://github.com/ahtabrizi/gstnvstabilize)

```
GST_DEBUG=3 gst-launch-1.0 rtspsrc location=rtsp://admin:Pwd12345@192.168.1.68/Streaming/Channels/0 latency=10 ! queue \
  ! rtph264depay ! h264parse ! nvv4l2decoder enable-max-performance=1 \
  ! nvvidconv ! 'video/x-raw,width=1280,height=720,format=RGBA'! nvstabilize \
  ! 'video/x-raw,width=1280,height=720' ! nvvidconv ! 'video/x-raw(memory:NVMM), format=I420, width=1920, height=1080' \
  ! nvv4l2h264enc maxperf-enable=1 bitrate=8000000 ! rtph264pay ! udpsink host=192.168.191.18 port=5600 sync=false          
```

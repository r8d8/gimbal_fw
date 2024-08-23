Jetson Nano pipeline with [nvstabilize](https://github.com/ahtabrizi/gstnvstabilize)

```

GST_DEBUG=3 gst-launch-1.0 nvarguscamerasrc sensor_id=0\
! "video/x-raw(memory:NVMM),width=1920,height=1080,framerate=60/1"\
! nvvidconv ! 'video/x-raw,width=1280,height=720,format=RGBA'\
! nvstabilize ! 'video/x-raw,width=1280,height=720' \
! nvvidconv ! 'video/x-raw(memory:NVMM), format=I420, width=1920, height=1080'\
! nvv4l2h264enc ! rtph264pay ! udpsink host=192.168.191.18 port=5600 sync=false

```

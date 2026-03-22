import time
import struct
import zmq
import rvl
from pyorbbecsdk import Context, Pipeline, Config, OBSensorType, OBFormat, OBAlignMode

W, H = 640, 480
FPS = 30

context = Context()
pipeline = Pipeline(context.query_devices().get_device_by_index(0))
config = Config()

config.enable_stream(pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR).get_video_stream_profile(W, H, OBFormat.MJPG, FPS))
config.enable_stream(pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR).get_video_stream_profile(W, H, OBFormat.Y16, FPS))
config.set_align_mode(OBAlignMode.HW_MODE)

pipeline.enable_frame_sync()
pipeline.start(config)

sock = zmq.Context().socket(zmq.PUB)
sock.setsockopt(zmq.SNDHWM, 1)
sock.bind("tcp://0.0.0.0:11000")

while True:
    frames = pipeline.wait_for_frames(1000)
    if frames is None: continue

    cframe = frames.get_color_frame()
    dframe = frames.get_depth_frame()

    if cframe is None or dframe is None: continue

    sock.send_multipart([
        struct.pack("d", time.time()),
        cframe.get_data(),
        rvl.compress(dframe.get_data())
    ])
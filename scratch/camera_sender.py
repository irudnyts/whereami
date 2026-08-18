import cv2

from picamera2 import Picamera2
from picamera2.encoders import H264Encoder
from picamera2.outputs import FfmpegOutput

import os
from dotenv import load_dotenv

load_dotenv()

MAC_IP = os.getenv("MAC_IP")
PORT = int(os.getenv("PORT"))

print(MAC_IP)

picam2 = Picamera2()

config = picam2.create_video_configuration(
    main={
        "size": (640, 480),
        "format": "RGB888",
    },
    controls={
        "FrameRate": 30
    }
)

picam2.configure(config)

encoder = H264Encoder(
    bitrate=2_000_000,
    repeat=True,
    iperiod=30
)

output = FfmpegOutput(
    f"-f mpegts udp://{MAC_IP}:{PORT}?pkt_size=1316"
)

picam2.start_recording(encoder, output)


try:
    while True:
        frame = picam2.capture_array("main")
        cv2.imshow("Raspberry Pi Camera", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break


finally:
    picam2.stop_recording()
    cv2.destroyAllWindows()
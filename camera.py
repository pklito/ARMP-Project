
from src.CameraUtils.CameraRun import draw_arucos, drawBothFrames, run_object_detection
from src.CameraUtils.CameraStreamer import CameraStreamer
if __name__ == "__main__":
    camera = CameraStreamer()
    # drawBothFrames(camera)
    while True:
        draw_arucos(camera)

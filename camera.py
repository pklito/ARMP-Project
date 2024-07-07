
from src.CameraUtils.CameraRun import draw_arucos, drawBothFrames, run_object_detection
from src.CameraUtils.CameraRun import drawBothFrames, run_object_detection, localization_detection
from src.CameraUtils.CameraStreamer import CameraStreamer
if __name__ == "__main__":
    camera = CameraStreamer()
    # drawBothFrames(camera)
    # ret = True
    # while ret is not None:
    #     ret = draw_arucos(camera)
    # localization_detection(camera)
    camera.stream()

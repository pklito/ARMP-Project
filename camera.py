
from src.CameraUtils.CameraRun import drawBothFrames, run_object_detection, localization_detection
from src.CameraUtils.CameraStreamer import CameraStreamer
if __name__ == "__main__":
    camera = CameraStreamer()
    localization_detection(camera)

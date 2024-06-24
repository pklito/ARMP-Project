
from src.CameraUtils.CameraRun import run_object_detection
from src.CameraUtils.CameraStreamer import CameraStreamer
if __name__ == "__main__":
    camera = CameraStreamer()
    run_object_detection(camera)

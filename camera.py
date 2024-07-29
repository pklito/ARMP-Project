
from src.CameraUtils.CameraRun import *
from src.CameraUtils.FakeCameraStreamer import FakeCameraStreamer
if __name__ == "__main__":

    camera = FakeCameraStreamer("Square path color only.mp4",loop=True,framerate=30)
    runCamera(camera, generateAll)


from src.CameraUtils.CameraRun import *
from src.CameraUtils.CameraStreamer import CameraStreamer
from src.CameraUtils.FakeCameraStreamer import FakeCameraStreamer
if __name__ == "__main__":

    camera = FakeCameraStreamer("C:/Users/paulo/Videos/Screen Recordings/Square path color only.mp4",loop=True)
    runCamera(camera, generateAll)


from src.CameraUtils.CameraRun import draw_arucos, drawBothFrames, run_object_detection
from src.CameraUtils.CameraRun import *
from src.CameraUtils.CameraStreamer import CameraStreamer
from src.CameraUtils.FakeCameraStreamer import FakeCameraStreamer
if __name__ == "__main__":

    camera = FakeCameraStreamer("C:/Users/paulo/Videos/Screen Recordings/Square path color only.mp4")
    #ball_hsv_mask_grayscale(camera)
    #run_object_detection(camera)
    # ret = True
    # while ret is not None:
    #     ret = draw_arucos(camera)
    localization_detection(camera)

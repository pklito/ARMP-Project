import pickle
from time import sleep, time
from CameraController import CameraWrapper
from Robots import TaskRobot, AssistanceRobot
import signal
import sys

assistance_robot = AssistanceRobot()
task_robot = TaskRobot()

def generate_samples(robot):
    samples = []
    print('Sampling starts (Press Ctrl+C to stop)')

    def signal_handler(sig, frame):
        print('\nSampling stopped.')
        print(f'Total samples collected: {len(samples)}')
        print(samples)
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    while True:
        input("Press Enter to take a sample... ")
        samples.append(robot.get_config())


if __name__ == "__main__":
    generate_samples(task_robot)
    [
        [0.5634124279022217, -3.0224253139891566, 0.3156502882586878, -0.41231973588977056, 0.9947256445884705, -1.5919516722308558],
        [0.15939290821552277, -2.7659503422179164, 0.3096535841571253, -0.6408603352359314, 0.7281607985496521, -1.6211860815631312],
        [0.6545559763908386, -0.8553479474833985, 0.9092324415790003, -3.162144800225729, -0.6023572126971644, -1.5884917418109339]
    ]

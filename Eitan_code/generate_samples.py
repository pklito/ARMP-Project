import pickle
from time import sleep, time
from Eitan_code.CameraController import CameraWrapper
from Eitan_code.Robots import TaskRobot, AssistanceRobot
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
    generate_samples(assistance_robot)
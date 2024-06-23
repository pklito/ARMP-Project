
import rtde.rtde as rtde
import rtde.rtde_config as rtde_config
import sys

def setp_to_list(sp):
    sp_list = []
    for i in range(0, 6):
        sp_list.append(sp.__dict__["input_double_register_%i" % i])
    return sp_list


def list_to_setp(sp, list):
    for i in range(0, 6):
        sp.__dict__["input_double_register_%i" % i] = list[i]
    return sp

class RTDERobot:
    # Variables
    setp = None
    watchdog = None
    con = None

    def __init__(self, ROBOT_HOST = "192.168.0.12", ROBOT_PORT = 30004, config_filename = "control_loop_configuration.xml"):
        # Load config files
        conf = rtde_config.ConfigFile(config_filename)
        state_names, state_types = conf.get_recipe("state")
        setp_names, setp_types = conf.get_recipe("setp")
        watchdog_names, watchdog_types = conf.get_recipe("watchdog")

        # Connect
        self.con = rtde.RTDE(ROBOT_HOST, ROBOT_PORT)
        self.con.connect()

        # get controller version
        self.con.get_controller_version()

        # setup recipes
        self.con.send_output_setup(state_names, state_types)
        self.setp = self.con.send_input_setup(setp_names, setp_types)
        self.watchdog = self.con.send_input_setup(watchdog_names, watchdog_types)

        if not self.con.send_start():
            print("Error: connection to robot failed")
            sys.exit()

    def disconnect(self):
        self.con.send_pause()
        self.con.disconnect()

    def __del__(self):
        self.disconnect()


    def sendConfig(self, config):
        self.setp = list_to_setp(self.setp, config)
        self.con.send(self.setp)

    def sendWatchdog(self, value):
        self.watchdog.input_int_register_0 = value
        self.con.send(self.watchdog)

    def getState(self):
        return self.con.receive()

    def getTargetConfig(self):
        return setp_to_list(self.setp)

if __name__ == '__main__':
    robot = RTDERobot()

    setp1 = [0.44, -2.22, -0.0, -0.93, 1.08, -1.57]
    setp2 = [0.44, -2.22, -0.0, -0.93, 1.08, -1.0]
    move_completed = True
    keep_running = True
    # do something...

    wd = 0
    while keep_running:

        state = robot.getState()
        if state is None:
            break

        if move_completed and state.output_int_register_0 == 1:
            move_completed = False
            new_setp = setp1 if robot.getTargetConfig() == setp2 else setp2
            robot.sendConfig(new_setp)
            wd = 1
        elif not move_completed and state.output_int_register_0 == 0:
            print("Move to confirmed pose = " + str(state.target_q))
            move_completed = True
            wd = 0

        robot.sendWatchdog(wd)


from utils.config_loader import ConfigLoader
import utils.utils as utils

import socket
import select
import struct
import numpy as np
import time

class URRobot():
    """Universal Robots."""
    NUM_BYTES = 2048
    ROBOT_STATE_MESSAGE = 16

    def __init__(self, configuration_filepath, start_on_creation=True):
        """
        Initializes a new UR robot.

        Args:
            configuration_filepath (string): path to the configuration JSON file.
        """
        self.configuration_filepath = configuration_filepath
        self.config = ConfigLoader.load(configuration_filepath)
        #creates a class which instance attributes are based on the config dictionary
        for k, v in self.config.items():
            setattr(self, k, v)
        self.joint_acc = self.JOINT_ACC_DEFAULT
        self.joint_vel = self.JOINT_VEL_DEFAULT
        self.tool_acc = self.TOOL_ACC_DEFAULT
        self.tool_vel = self.TOOL_VEL_DEFAULT

        if start_on_creation:
            self.start()

    # Robot methods
    # -------------------------------------
    def go_home(self):
        """Moves the robot to home pose."""
        self.move_joints(self.home_joints_rad)

    def get_state(self):
        return self.parse_tcp_state_data(self.get_tcp_state())

    def get_current_joints(self):
        return self.parse_tcp_state_data(self.get_tcp_state(), subpackages=['joint_data'])['joint_data']

    def move_joints(self, joint_configuration_rad):
        command = "movej([%f" % joint_configuration_rad[0]
        for joint_idx in range(1,6):
            command = command + (",%f" % joint_configuration_rad[joint_idx])
        command = command + "],a=%f,v=%f)\n" % (self.joint_acc, self.joint_vel)

        self.send_tcp_command(command)

        # Block until robot reaches desired joint positions.
        current_joints = self.get_current_joints()
        while not all([np.abs(current_joints[i] - joint_configuration_rad[i]) < self.joint_tolerance for i in range(6)]):
            current_joints = self.get_current_joints()
            time.sleep(0.01)


    # Extension methods
    # -------------------------------------

    def activate_safe_mode(self):
        self.joint_acc = self.JOINT_ACC_SAFE
        self.joint_vel = self.JOINT_VEL_SAFE
        self.tool_acc = self.TOOL_ACC_SAFE
        self.tool_vel = self.TOOL_VEL_SAFE

    def deactivate_safe_mode(self):
        self.joint_acc = self.JOINT_ACC_DEFAULT
        self.joint_vel = self.JOINT_VEL_DEFAULT
        self.tool_acc = self.TOOL_ACC_DEFAULT
        self.tool_vel = self.TOOL_VEL_DEFAULT

    def start(self):
        self.load_program()
        self.power_on()
        self.brake_release()

    def load_program(self):
        command = 'load ' + self.program + '\n'
        response = self.send_tcp_command(command, port=self.dashboard_port).decode("utf-8")

    def power_on(self):
        command = 'power on\n'
        response = self.send_tcp_command(command, port=self.dashboard_port).decode("utf-8")

    def brake_release(self):
        command = 'brake release\n'
        response = self.send_tcp_command(command, port=self.dashboard_port).decode("utf-8")

    def get_robot_mode(self):
        command = 'robotmode\n'
        response = self.send_tcp_command(command, port=self.dashboard_port).decode("utf-8")
        return response

    def play_program(self, n_trials=1):
        is_running = self.check_program_running()
        if not is_running:
            command = 'play\n'
            response = self.send_tcp_command(command, port=self.dashboard_port).decode("utf-8")
            print(response)
            if 'Starting program' in response:
                is_running = self.check_program_running()
                while not is_running:
                    is_running = self.check_program_running()
                    time.sleep(0.01)
            elif n_trials > 0:
                n_trials -=1
                self.play_program(n_trials=n_trials)

    def stop_program(self, n_trials=1):
        command = 'stop\n'
        response = self.send_tcp_command(command, port=self.dashboard_port).decode("utf-8")
        if 'Stopped' in response:
            is_running = self.check_program_running()
            while is_running:
                is_running = self.check_program_running()
                time.sleep(0.01)
        elif n_trials > 0:
            n_trials -=1
            self.stop_program(n_trials=n_trials)

    def check_program_running(self):
        command = 'running\n'
        response = self.send_tcp_command(command, port=self.dashboard_port).decode("utf-8")
        return 'true' in response

    def get_tcp_state(self):
        def assure_is_robot_state_data(state_data):
            # Read package header
            data_bytes = bytearray()
            data_bytes.extend(state_data)
            data_length = struct.unpack("!i", data_bytes[0:4])[0];
            robot_message_type = data_bytes[4]
            if robot_message_type == self.ROBOT_STATE_MESSAGE: #Version Message = 20 (only sent once), Robot State Message = 16
                return state_data
            else:
                state_data = self.tcp_socket.recv(self.NUM_BYTES)
                return assure_is_robot_state_data(state_data)

        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_socket.connect((self.robot_ip, self.tcp_port))
        tcp_state_data = self.tcp_socket.recv(self.NUM_BYTES)
        tcp_state_data = assure_is_robot_state_data(tcp_state_data) #First package contains version info (MSG_TYPE 20)
        self.tcp_socket.close()
        return tcp_state_data

    def parse_tcp_state_data(self, state_data, subpackages=['joint_data', 'tool_data', 'masterboard_data', 'cartesian_data', 'force_data']):
        #Read package header
        data_bytes = bytearray()
        data_bytes.extend(state_data)
        data_length = struct.unpack("!i", data_bytes[0:4])[0]
        robot_message_type = data_bytes[4]
        assert(robot_message_type == self.ROBOT_STATE_MESSAGE) #Version Message = 20 (only sent once), Robot State Message = 16
        byte_idx = 5

        #Methods to parse each subpackage
        def parse_joint_data(data_bytes, byte_idx):
            actual_joint_positions = [0,0,0,0,0,0]
            target_joint_positions = [0,0,0,0,0,0]
            for joint_idx in range(6):
                actual_joint_positions[joint_idx] = struct.unpack('!d', data_bytes[(byte_idx+0):(byte_idx+8)])[0]
                target_joint_positions[joint_idx] = struct.unpack('!d', data_bytes[(byte_idx+8):(byte_idx+16)])[0]
                byte_idx += 41
            return actual_joint_positions

        def parse_cartesian_data(data_bytes, byte_idx):
            actual_tool_pose = [0,0,0,0,0,0]
            for pose_value_idx in range(6):
                actual_tool_pose[pose_value_idx] = struct.unpack('!d', data_bytes[(byte_idx+0):(byte_idx+8)])[0]
                byte_idx += 8
            return actual_tool_pose

        def parse_tool_data(data_bytes, byte_idx):
            byte_idx += 2
            tool_analog_input2 = struct.unpack('!d', data_bytes[(byte_idx+0):(byte_idx+8)])[0]
            return tool_analog_input2

        def parse_masterboard_data(data_bytes, byte_idx):
            def bitfield(n):
                return [1 if digit=='1' else 0 for digit in bin(n)[2:]]

            def parse_digital_inputs(data_bytes, byte_idx):
                digital_inputs = struct.unpack('!i', data_bytes[(byte_idx+0):(byte_idx+4)])[0]
                digital_inputs_bits = bitfield(digital_inputs)
                return digital_inputs_bits

            def parse_digital_outputs(data_bytes, byte_idx):
                digital_outputs = struct.unpack('!i', data_bytes[(byte_idx+4):(byte_idx+8)])[0]
                digital_outputs_bits = bitfield(digital_outputs)
                return digital_outputs_bits

            din_bits = parse_digital_inputs(data_bytes, byte_idx)
            dout_bits = parse_digital_outputs(data_bytes, byte_idx)
            return {'digital_inputs': din_bits, 'digital_outputs': dout_bits}

        def parse_force_data(data_bytes, byte_idx):
            force_vector = [0,0,0,0,0,0]
            for value_idx in range(6):
                force_vector[value_idx] = struct.unpack('!d', data_bytes[(byte_idx+0):(byte_idx+8)])[0]
                byte_idx += 8
            robot_dexterity = struct.unpack('!d', data_bytes[(byte_idx+0):(byte_idx+8)])[0]
            return {'force': force_vector, 'dexterity': robot_dexterity}

        #Sub-package types
        subpackage_type = {'1':'joint_data', '2':'tool_data', '3':'masterboard_data', '4':'cartesian_data', '7':'force_data'}
        #Sub-package parse functions
        parse_functions = {'joint_data' : parse_joint_data, 'tool_data' : parse_tool_data,
                            'masterboard_data' : parse_masterboard_data, 'cartesian_data' : parse_cartesian_data, 'force_data' : parse_force_data}
        tcp_state_data = {}

        while byte_idx < data_length or len(subpackages) > 0:
            package_length = struct.unpack("!i", data_bytes[byte_idx:(byte_idx+4)])[0]
            byte_idx += 4
            package_idx = str(data_bytes[byte_idx])
            if package_idx in subpackage_type:
                subpackage = subpackage_type.pop(package_idx)
                if subpackage in subpackages:
                    subpackages.remove(subpackage)
                    tcp_state_data[subpackage] = parse_functions[subpackage](data_bytes, byte_idx + 1)
            byte_idx += package_length - 4

        return tcp_state_data

    def send_tcp_command(self, command, host_ip=None, port=None):
        response_data = None
        if command:
            if not host_ip: host_ip = self.robot_ip
            if not port: port = self.tcp_port
            self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.tcp_socket.connect((host_ip, port))
            self.tcp_socket.recv(self.NUM_BYTES)
            self.tcp_socket.send(str.encode(command))
            response_data = self.tcp_socket.recv(self.NUM_BYTES)
            self.tcp_socket.close()
        return response_data

    def set_digital_out_signal(self, number, value):
        command = f"set_digital_out({number}, {value})\n"
        self.send_tcp_command(command)

    def get_cartesian_pose(self):
        return self.parse_tcp_state_data(self.get_tcp_state(), subpackages=['cartesian_data'])['cartesian_data']

    def move_to_pose(self, position, orientation):
        command = "movel(p[%f,%f,%f,%f,%f,%f],a=%f,v=%f,t=0,r=0)\n" % (position[0],position[1],position[2],orientation[0],orientation[1],orientation[2],self.tool_acc,self.tool_vel)
        self.send_tcp_command(command)
        #Block until robot reaches desired pose
        current_pose = self.get_cartesian_pose()
        while not all([np.abs(current_pose[i] - position[i]) < self.pose_tolerance[i] for i in range(3)]):
            current_pose = self.get_cartesian_pose()
            time.sleep(0.01)

    def get_tool_data(self):
        return self.parse_tcp_state_data(self.get_tcp_state(), subpackages=['tool_data'])['tool_data']

    def move_wrt_tool(self, position):
        current_pose = self.get_cartesian_pose()
        current_pose = np.array(current_pose)
        current_position = current_pose[0:3]
        orientation = current_pose[3:6]
        # Transform from position to base_world_position
        position.shape = (3,1)
        current_pose.shape = (1,6)
        T_be = utils.V2T(current_pose)
        base_world_position = np.dot(T_be[0:3,0:3], position[0:3,0]) + current_position
        self.move_to_pose(base_world_position[0:3], orientation)

    def orientate_wrt_tool(self, orientation):
        command =  "def process():\n"
        command += "    pose_wrt_tool = p[%f,%f,%f,%f,%f,%f]\n" % (0.0, 0.0, 0.0, orientation[0], orientation[1], orientation[2])
        command += "    pose_wrt_base = pose_trans(get_forward_kin(), pose_wrt_tool)\n"
        command += "    movel(pose_wrt_base ,a=%f,v=%f)\n" % (self.tool_acc, self.tool_vel)
        command += "end\n"
        self.send_tcp_command(command)

    def get_digital_inputs(self):
        return self.parse_tcp_state_data(self.get_tcp_state(), subpackages=['masterboard_data'])['masterboard_data']['digital_inputs']

    def move_with_forces(self, joint_selector, joint_forces, n_seconds=0.7, max_dist_from_orig_m=0.1, max_force_N=90, max_z_velocity=0.05):
        #Push in Z axis towards the target
        command =  "def process():\n"
        command += "    start_pose = get_actual_tcp_pose()\n"
        command += "    current_pose = get_actual_tcp_pose()\n"
        command += "    force_mode(tool_pose(), [%d,%d,%d,%d,%d,%d], [%f,%f,%f,%f,%f,%f], 2, [0.1, 0.1, %f, 0.17, 0.17, 0.17])\n" % (joint_selector[0], joint_selector[1], joint_selector[2], joint_selector[3], joint_selector[4], joint_selector[5], joint_forces[0], joint_forces[1], joint_forces[2], joint_forces[3], joint_forces[4], joint_forces[5], max_z_velocity)
        command += "    while (point_dist(start_pose, current_pose) <= %f and force() < %d):\n" % (max_dist_from_orig_m, max_force_N)
        command += "        sync()\n"
        command += "        current_pose = get_actual_tcp_pose()\n"
        command += "    end\n"
        command += "    end_force_mode()\n"
        command += "end\n"
        self.send_tcp_command(command)
        time.sleep(n_seconds)
        command =  "def process():\n"
        command += "    end_force_mode()\n"
        command += "end\n"
        self.send_tcp_command(command)

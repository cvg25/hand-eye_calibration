from utils.config_loader import ConfigLoader
import numpy as np
import socket

class RealsenseD415TCP():

    def __init__(self, configuration_filepath, start_on_creation=True):
        """
        Initializes a new Realsense D415 Camera over TCP connection.

        Args:
            configuration_filepath (string): path to the configuration JSON file.
        """
        self.configuration_filepath = configuration_filepath
        self.config = ConfigLoader.load(configuration_filepath)
        self.intrinsics = None
        self.tcp_socket = None
        #creates a class which instance attributes are based on the config dictionary
        for k, v in self.config.items():
            setattr(self, k, v)
        if start_on_creation:
            self.start()

    def get_state(self):
        # Ping the server with anything
        self.tcp_socket.send(b'asdf')

        # Fetch TCP data:
        #     color camera intrinsics, 9 floats, number of bytes: 9 x 4
        #     depth scale for converting depth from uint16 to float, 1 float, number of bytes: 4
        #     depth image, self.im_width x self.im_height uint16, number of bytes: self.im_width x self.im_height x 2
        #     color image, self.im_width x self.im_height x 3 uint8, number of bytes: self.im_width x self.im_height x 3
        data = b''
        while len(data) < (10*4 + self.im_height*self.im_width*5):
            data += self.tcp_socket.recv(self.buffer_size)
        # Reorganize TCP data into color and depth frame
        self.intrinsics = np.fromstring(data[0:(9*4)], np.float32).reshape(3, 3)
        depth_scale = np.fromstring(data[(9*4):(10*4)], np.float32)[0]
        depth_img = np.fromstring(data[(10*4):((10*4)+self.im_width*self.im_height*2)], np.uint16).reshape(self.im_height, self.im_width)
        color_img = np.fromstring(data[((10*4)+self.im_width*self.im_height*2):], np.uint8).reshape(self.im_height, self.im_width, 3)
        depth_img = depth_img.astype(float) * depth_scale
        return color_img, depth_img

    def start(self):
        # Connect to server
        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_socket.connect((self.tcp_host_ip, self.tcp_port))

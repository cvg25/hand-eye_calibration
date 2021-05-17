from robot.ur_robot import URRobot
from vision.realsense_d415_tcp import RealsenseD415TCP
import utils.utils as utils
import vision.utils as visionutils
from utils.config_loader import ConfigLoader
import numpy as np
import time
import cv2
import argparse

def touch_tester(args):
    config = ConfigLoader.load(args.config_file)
    print(f'Touch tester in {config["calibration_type"]} mode as indicated on configuration file.')
    robot = URRobot(config["robot_config_file"])
    robot.activate_safe_mode()
    robot.move_joints(robot.home_joints_rad)

    camera = RealsenseD415TCP(config["camera_config_file"])
    # Load camera pose (from running camera_calibrator.py), and depth scale
    cam_pose = np.loadtxt(config["calibration_camera_pose"], delimiter=' ')
    cam_depth_scale = np.loadtxt(config["calibration_depth_scale"], delimiter=' ')

    click_point_pix = ()
    def mouseclick_callback(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            nonlocal click_point_pix, camera, robot, depth, color, cam_pose, cam_depth_scale
            click_point_pix = (x, y)
            pix_width = x
            pix_height = y
            world_position = visionutils.transform_pix_to_world_pos(depth, pix_width, pix_height, cam_pose, camera.intrinsics, cam_depth_scale)
            if config["calibration_type"] == "EYE_IN_HAND":
                tool_world_position = world_position
                robot.move_wrt_tool(tool_world_position)
                # Equivalent option: Transform tool_world_position to robot_base_world_pose and then use robot.move_to_pose()
                #
                # Get current robot_base_world_pose
                # current_pose = robot.get_cartesian_pose()
                # current_pose = np.array(current_pose)
                # current_position = current_pose[0:3]
                # current_orientation = current_pose[3:6]

                # Transform from tool_world_position to base_world_position
                # tool_world_position.shape = (3,1)
                # current_pose.shape = (1,6)
                # T_eb = utils.V2T(current_pose)
                # base_world_position = np.dot(T_eb[0:3,0:3], tool_world_position[0:3,0]) + current_position
                # robot.move_to_pose(base_world_position[0:3], current_orientation)

            else: # "EYE_TO_HAND"
                current_pose = robot.get_cartesian_pose()
                current_pose = np.array(current_pose)
                current_orientation = current_pose[3:6]
                base_world_position = world_position
                robot.move_to_pose(base_world_position, current_orientation)

    # Show color frame
    cv2.namedWindow('color')
    cv2.setMouseCallback('color', mouseclick_callback)

    while True:
        color, depth = camera.get_state()
        if len(click_point_pix) != 0:
            color = cv2.circle(color, click_point_pix, 7, (0,0,255), 2)
        color = cv2.cvtColor(color, cv2.COLOR_BGR2RGB )
        cv2.imshow('color', color)
        cv2.waitKey(1)

    cv2.destroyAllWindows()

if __name__ == '__main__':
    # Parse arguments
    parser = argparse.ArgumentParser(description='Calibrate a camera with respect to a robot, and save the transformation matrix.')

    # Setup options
    parser.add_argument('--config_file', dest='config_file', action='store', default='./configurations/touch_tester_config.json', help='Configuration file for the calibration.')
    args = parser.parse_args()
    touch_tester(args)

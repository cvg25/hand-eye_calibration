import cv2
import numpy as np

def find_checkerboard(color_img, checkerboard_size):
    """
    Returns X,Y pixel coordinates of checkerboard center in the image.
    """
    global refine_criteria
    bgr_color_data = cv2.cvtColor(color_img, cv2.COLOR_RGB2BGR)
    gray_data = cv2.cvtColor(bgr_color_data, cv2.COLOR_BGR2GRAY)
    checkerboard_found, corners = cv2.findChessboardCorners(gray_data, checkerboard_size, None, cv2.CALIB_CB_ADAPTIVE_THRESH)
    if checkerboard_found:
        refine_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners_refined = cv2.cornerSubPix(gray_data, corners, checkerboard_size, (-1,-1), refine_criteria)
        # Get observed checkerboard center 3D point in camera space
        checkerboard_pix = np.round(corners_refined[4,0,:]).astype(int)
        return checkerboard_pix
    else:
        return None

def get_3d_from_2d_point(point_wh, depth, camera_intrinsics, depth_scale=1.00):
    width = point_wh[0]
    height = point_wh[1]
    z3d = depth[height][width] * depth_scale
    x3d = np.multiply(width - camera_intrinsics[0][2], z3d / camera_intrinsics[0][0])
    y3d = np.multiply(height - camera_intrinsics[1][2], z3d / camera_intrinsics[1][1])
    return np.array([[x3d, y3d, z3d]])

def transform_pix_to_world_pos(depth, pix_w, pix_h, world_transform, cam_intrinsics, depth_scale):
    camera_3d_point = get_3d_from_2d_point((pix_w, pix_h), depth, cam_intrinsics, depth_scale)
    z_3d = camera_3d_point[0,2]
    if z_3d > 0.0:
        camera_3d_point.shape = (3,1)
        world_position = np.dot(world_transform[0:3,0:3], camera_3d_point) + world_transform[0:3,3:]
        return world_position.flatten()
    else:
        return None

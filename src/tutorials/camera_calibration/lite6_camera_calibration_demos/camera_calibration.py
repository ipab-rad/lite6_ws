#!/usr/bin/env python3
"""
A ROS 2 calibration node.

Implementation is heavily inspired by the great work of Alexander Khazatsky.
Source: https://github.com/AlexanderKhazatsky/R2D2/blob/main/r2d2/calibration/calibration_utils.py
"""

import time
import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from launch_param_builder import ParameterBuilder

import cv2
from cv2 import aruco
from cv_bridge import CvBridge

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from std_srvs.srv import SetBool

from moveit.planning import MoveItPy
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory

class CameraCalibration(Node):
    """A ROS 2 node to calibrate camera"""

    def __init__(self):
        """Initialize"""
        super().__init__("camera_calibration")
        self.logger = self.get_logger()

        # read in camera config
        self.calib_config = ParameterBuilder("lite6_camera_calibration_demos").yaml("config/camera_calibration.yaml").to_dict()
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_100)
        self.charuco_board = cv2.aruco.CharucoBoard_create(
            self.calib_config["charuco"]["squares_x"],
            self.calib_config["charuco"]["squares_y"],
            self.calib_config["charuco"]["square_length"],
            self.calib_config["charuco"]["marker_length"],
            self.aruco_dict
        )
        self.detector_params = cv2.aruco.DetectorParameters_create()
        self.detector_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.calib_flags = cv2.CALIB_USE_INTRINSIC_GUESS + cv2.CALIB_FIX_PRINCIPAL_POINT + cv2.CALIB_FIX_FOCAL_LENGTH
        self.cv_bridge = CvBridge()

        # subscribe to camera info
        self.create_subscription(
            CameraInfo,
            self.calib_config["camera_info_topic"],
            self._camera_info_callback,
            10
        )

        # subscribe to camera image
        self.create_subscription(
            Image,
            self.calib_config["camera_topic"],
            self._image_callback,
            10
        )
    
        # create a service client for running hand eye calibration
        self.calib_client = self.create_service(
            SetBool,
            self.calib_config["calibration_service_name"],
            self.run_hand_eye_calibration,
        )

        # track latest values of camera info and image 
        self._latest_image = None
        self._camera_info = None

        # instantiate moveit python interface
        moveit_config = (
        MoveItConfigsBuilder(robot_name="lite6", package_name="moveit_resources_lite6_moveit_config")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_semantic("config/lite6.srdf")
        .robot_description(file_path=get_package_share_directory("moveit_resources_lite6_description")
            + "/urdf/lite6.urdf")
        .moveit_cpp(
            file_path=get_package_share_directory("lite6_motion_planning_demos")
            + "/config/moveit_cpp.yaml"
        )
        .to_moveit_configs()
        ).to_dict()

        self.lite6 = MoveItPy(config_dict=moveit_config)
        self.lite6_arm = self.lite6.get_planning_component("lite6")

    def _image_callback(self, msg):
        """Callback function for image topic"""
        self._latest_image = self.cv_bridge.imgmsg_to_cv2(msg, "mono8")

    def _camera_info_callback(self, msg):
        """Callback function for camera info topic"""
        self._camera_info = msg
    
    def base2robot(self):
        self.lite6_arm.set_start_state_to_current_state()
        robot_state = self.lite6_arm.get_start_state()
        t_mat = robot_state.get_frame_transform("link6")
        return t_mat[:3, :-1], t_mat[:3, -1]
    

    def move_to_random_pose(self):
        # set plan start state to current state
        self.lite6_arm.set_start_state_to_current_state()
        
        # random pose within workspace
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "link_base"
        pose_goal.pose.position.x = np.random.uniform(
                self.calib_config["workspace"]["x_min"], 
                self.calib_config["workspace"]["x_max"]
                )
        pose_goal.pose.position.y = np.random.uniform(
                self.calib_config["workspace"]["y_min"],
                self.calib_config["workspace"]["y_max"]
                )
        pose_goal.pose.position.z = np.random.uniform(
                self.calib_config["workspace"]["z_min"],
                self.calib_config["workspace"]["z_max"]
                )

        # Create a rotation object from Euler angles specifying axes of rotation
        rot_x = np.random.uniform(
                self.calib_config["workspace"]["rot_x_min"],
                self.calib_config["workspace"]["rot_x_max"]
                )
        rot_y = np.random.uniform(
                self.calib_config["workspace"]["rot_y_min"],
                self.calib_config["workspace"]["rot_y_max"]
                )
        rot_z = np.random.uniform(
                self.calib_config["workspace"]["rot_z_min"],
                self.calib_config["workspace"]["rot_z_max"]
                )
        rot = Rotation.from_euler('xyz', [rot_x, rot_y, rot_z], degrees=True)
        rot_quat = rot.as_quat()
        pose_goal.pose.orientation.x = rot_quat[0]
        pose_goal.pose.orientation.y = rot_quat[1]
        pose_goal.pose.orientation.z = rot_quat[2]
        pose_goal.pose.orientation.w = rot_quat[3]
       
        # get current pose
        self.lite6_arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="link6")
        
        # plan and execute
        plan_result = self.lite6_arm.plan()
        if plan_result:
            robot_trajectory = plan_result.trajectory
            self.lite6.execute(robot_trajectory, controllers=[])
    
    
    def detect_charuco_board(self, image):
        """Detect charuco board in image"""
        # detect aruco markers
        corners, ids, rejectedImgPoints = aruco.detectMarkers(
            image, self.aruco_dict, parameters=self.detector_params
        )
        
        corners, ids, _, _ = cv2.aruco.refineDetectedMarkers(
            image,
            self.charuco_board,
            corners,
            ids,
            rejectedImgPoints,
            parameters=self.detector_params,
            cameraMatrix=np.array(self._camera_info.k).reshape(3,3),
            )

        # if no markers found, return
        if ids is None:
            return None, None

        # detect charuco board
        num_corners_found, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
            corners, 
            ids, 
            image, 
            self.charuco_board, 
            cameraMatrix=np.array(self._camera_info.k).reshape(3,3),
        )

        # if no charuco board found, return
        if num_corners_found < 5:
            return None, None

        # draw detected markers
        #image = aruco.drawDetectedMarkers(image, corners, ids)

        # draw detected charuco board
        #image = aruco.drawDetectedCornersCharuco(
        #    image, charuco_corners, charuco_ids
        #)

        return image, charuco_corners, charuco_ids, image.shape[:2]
    
    def calc_target_to_camera(self, readings):
        init_corners_all = []  # Corners discovered in all images processed
        init_ids_all = []  # Aruco ids corresponding to corners discovered
        fixed_image_size = readings[0][3]

        # Proccess Readings #
        init_successes = []
        for i in range(len(readings)):
            corners, charuco_corners, charuco_ids, img_size = readings[i]
            assert img_size == fixed_image_size
            init_corners_all.append(charuco_corners)
            init_ids_all.append(charuco_ids)
            init_successes.append(i)

        # First Pass: Find Outliers #
        threshold = 1
        if len(init_successes) < threshold:
            return None

        calibration_error, cameraMatrix, distCoeffs, rvecs, tvecs, stdIntrinsics, stdExtrinsics, perViewErrors = (
            aruco.calibrateCameraCharucoExtended(
                charucoCorners=init_corners_all,
                charucoIds=init_ids_all,
                board=self.charuco_board,
                imageSize=fixed_image_size,
                flags=self.calib_flags,
                cameraMatrix=np.array(self._camera_info.k).reshape(3,3),
                distCoeffs=np.array(self._camera_info.d),
            )
        )

        # Remove Outliers #
        threshold = 1
        final_corners_all = [
                init_corners_all[i] for i in range(len(perViewErrors)) if perViewErrors[i] <= 3.0 # TODO: read from params
        ]
        final_ids_all = [
            init_ids_all[i] for i in range(len(perViewErrors)) if perViewErrors[i] <= 3.0
        ]
        final_successes = [
            init_successes[i] for i in range(len(perViewErrors)) if perViewErrors[i] <= 3.0
        ]
        if len(final_successes) < threshold:
            return None

        # Second Pass: Calculate Finalized Extrinsics #
        calibration_error, cameraMatrix, distCoeffs, rvecs, tvecs = aruco.calibrateCameraCharuco(
            charucoCorners=final_corners_all,
            charucoIds=final_ids_all,
            board=self.charuco_board,
            imageSize=fixed_image_size,
            flags=self.calib_flags,
            cameraMatrix=np.array(self._camera_info.k).reshape(3,3),
            distCoeffs=np.array(self._camera_info.d),
        )

        # Return Transformation #
        if calibration_error > 3.0:
            return None

        rmats = [Rotation.from_rotvec(rvec.flatten()).as_matrix() for rvec in rvecs]
        tvecs = [tvec.flatten() for tvec in tvecs]

        return rmats, tvecs, final_successes

    def run_hand_eye_calibration(self, request, response):
        """Run calibration"""
        # check if we have both camera info and image
        if self._latest_image is None or self._camera_info is None:
            self.logger.warn("No image or camera info received yet")
            return

        # run calibration
        self.logger.info("Running calibration")
        
        # collect samples
        images = []
        base_to_ee = []
        for i in range(self.calib_config["num_samples"]):
            self.logger.info("Collecting sample {}".format(i))
            self.move_to_random_pose()
            # capture image
            images.append(self._latest_image)

            # capture base to ee transform
            base_to_ee.append(self.base2robot())

            time.sleep(self.calib_config["sample_delay"])
        
        # process captured images
        readings = []
        for image in images:
            readings.append(self.detect_charuco_board(image))
        
        # calculate target to camera transform
        target_to_camera = self.calc_target_to_camera(readings)
        
        # run calibration
        rmat, pos = cv2.calibrateHandEye(
            R_gripper2base=[x[0] for x in base_to_ee],
            t_gripper2base=[x[1] for x in base_to_ee],
            R_target2cam=target_to_camera[0],
            t_target2cam=target_to_camera[1],
            method=4,
        )

        # return success response
        response.success = True
        response.message = "Calibration successful"
        return response

if __name__=="__main__":
    rclpy.init()
    node = CameraCalibration()
    rclpy.spin(node)
    rclpy.shutdown()

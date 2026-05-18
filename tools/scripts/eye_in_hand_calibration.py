#!/usr/bin/env python3

"""
眼在手上的手眼标定脚本，使用OpenCV计算相机相对于机器人末端执行器的变换关系。

用法:
    python3 eye_in_hand_calibration.py --data_path /path/to/calibration/data --pattern_size 8 6 --square_size 0.025
    python3 eye_in_hand_calibration.py --data_path /path/to/calibration/data --camera_matrix_file /path/to/camera_matrix.json
    python3 eye_in_hand_calibration.py --data_path /path/to/calibration/data --aruco_dict DICT_6X6_250 --aruco_size 0.025
    python3 eye_in_hand_calibration.py --data_path /path/to/calibration/data --aruco_dict DICT_6X6_250 --aruco_size 0.025 --aruco_ids 0 1 2 3
    python3 eye_in_hand_calibration.py --data_path /path/to/calibration/data --aruco_board 5 7 0.025 0.01
"""

import os
import json
import argparse
import numpy as np
import cv2
import glob
from scipy.spatial.transform import Rotation
import time


def load_poses(pose_dir):
    """
    加载位姿数据
    """
    pose_files = sorted(glob.glob(os.path.join(pose_dir, "pose_*.json")))
    robot_poses = []
    
    for pose_file in pose_files:
        with open(pose_file, 'r') as f:
            data = json.load(f)
            
        position = data['position']
        orientation = data['orientation']
        
        # 创建旋转矩阵 (从四元数)
        quat = [orientation['x'], orientation['y'], orientation['z'], orientation['w']]
        rot = Rotation.from_quat(quat)
        rot_matrix = rot.as_matrix()
        
        # 创建位姿矩阵 (4x4)
        pose_matrix = np.eye(4)
        pose_matrix[:3, :3] = rot_matrix
        pose_matrix[0, 3] = position['x']
        pose_matrix[1, 3] = position['y']
        pose_matrix[2, 3] = position['z']
        
        robot_poses.append(pose_matrix)
    
    return robot_poses


def get_aruco_dict(dict_name):
    """
    获取指定名称的ArUco字典
    
    参数:
        dict_name: ArUco字典名称，例如 DICT_6X6_250
    
    返回:
        ArUco字典对象
    """
    aruco_dicts = {
        'DICT_4X4_50': cv2.aruco.DICT_4X4_50,
        'DICT_4X4_100': cv2.aruco.DICT_4X4_100,
        'DICT_4X4_250': cv2.aruco.DICT_4X4_250,
        'DICT_4X4_1000': cv2.aruco.DICT_4X4_1000,
        'DICT_5X5_50': cv2.aruco.DICT_5X5_50,
        'DICT_5X5_100': cv2.aruco.DICT_5X5_100,
        'DICT_5X5_250': cv2.aruco.DICT_5X5_250,
        'DICT_5X5_1000': cv2.aruco.DICT_5X5_1000,
        'DICT_6X6_50': cv2.aruco.DICT_6X6_50,
        'DICT_6X6_100': cv2.aruco.DICT_6X6_100,
        'DICT_6X6_250': cv2.aruco.DICT_6X6_250,
        'DICT_6X6_1000': cv2.aruco.DICT_6X6_1000,
        'DICT_7X7_50': cv2.aruco.DICT_7X7_50,
        'DICT_7X7_100': cv2.aruco.DICT_7X7_100,
        'DICT_7X7_250': cv2.aruco.DICT_7X7_250,
        'DICT_7X7_1000': cv2.aruco.DICT_7X7_1000,
        'DICT_ARUCO_ORIGINAL': cv2.aruco.DICT_ARUCO_ORIGINAL
    }
    
    if dict_name not in aruco_dicts:
        raise ValueError(f"未知的ArUco字典名称: {dict_name}")
    
    return cv2.aruco.getPredefinedDictionary(aruco_dicts[dict_name])


def detect_aruco_markers(image_path, aruco_dict, marker_size, camera_matrix=None, dist_coeffs=None, specific_ids=None):
    """
    检测图像中的ArUco标记
    
    参数:
        image_path: 图像路径
        aruco_dict: ArUco字典对象
        marker_size: 标记的实际尺寸（米）
        camera_matrix: 相机内参矩阵（可选）
        dist_coeffs: 畸变系数（可选）
        specific_ids: 需要检测的特定标记ID列表（可选）
    
    返回:
        成功标志, 世界坐标点, 图像坐标点, 可视化图像
    """
    # 读取图像
    img = cv2.imread(image_path)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # 创建ArUco参数 - 兼容新旧版本API
    try:
        # 尝试使用新版本API
        parameters = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
        corners, ids, rejected = detector.detectMarkers(gray)
    except AttributeError:
        # 如果新版本API不可用，使用旧版本API
        parameters = cv2.aruco.DetectorParameters_create()
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    
    # 绘制检测结果并保存可视化图像
    img_vis = img.copy()
    if ids is not None:
        # 如果指定了特定ID，筛选符合的标记
        if specific_ids is not None:
            specific_ids = np.array(specific_ids)
            valid_indices = np.where(np.isin(ids.flatten(), specific_ids))[0]
            if len(valid_indices) == 0:
                print(f"  - 未检测到指定的标记ID {specific_ids}")
                return False, None, None, img_vis
                
            # 筛选符合条件的角点和ID
            corners = [corners[i] for i in valid_indices]
            ids = ids[valid_indices]
            print(f"  - 检测到指定的标记ID: {ids.flatten()}")
        
        cv2.aruco.drawDetectedMarkers(img_vis, corners, ids)
        
        calibration_vis_dir = os.path.join(os.path.dirname(image_path), '..', 'calibration_vis')
        if not os.path.exists(calibration_vis_dir):
            os.makedirs(calibration_vis_dir)
        
        vis_path = os.path.join(calibration_vis_dir, os.path.basename(image_path))
        cv2.imwrite(vis_path, img_vis)
        
        # 如果提供了相机参数，则估计姿态
        if camera_matrix is not None and dist_coeffs is not None:
            # 使用对应版本的API估计姿态
            try:
                # 尝试新版本API
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners, marker_size, camera_matrix, dist_coeffs)
            except:
                # 如果新版本API不可用，使用旧版本API
                rvecs, tvecs = cv2.aruco.estimatePoseSingleMarkers(
                    corners, marker_size, camera_matrix, dist_coeffs)
            
            # 在图像上绘制坐标轴
            for i in range(len(ids)):
                try:
                    cv2.drawFrameAxes(img_vis, camera_matrix, dist_coeffs, 
                                      rvecs[i], tvecs[i], marker_size * 0.5)
                except:
                    # 旧版本可能没有drawFrameAxes
                    cv2.aruco.drawAxis(img_vis, camera_matrix, dist_coeffs, 
                                      rvecs[i], tvecs[i], marker_size * 0.5)
            
            # 创建世界坐标点和图像坐标点
            objPoints = []
            imgPoints = []
            
            for i in range(len(ids)):
                # 为每个标记创建3D坐标 (方形标记的4个角点)
                marker_objp = np.array([
                    [-marker_size/2, marker_size/2, 0],
                    [marker_size/2, marker_size/2, 0],
                    [marker_size/2, -marker_size/2, 0],
                    [-marker_size/2, -marker_size/2, 0]
                ], dtype=np.float32)
                
                # 添加到世界坐标点列表
                objPoints.append(marker_objp)
                
                # 添加检测到的角点到图像坐标点列表
                imgPoints.append(corners[i][0])
            
            # 将列表转换为NumPy数组
            objPoints = np.concatenate(objPoints)
            imgPoints = np.concatenate(imgPoints)
            
            return True, objPoints, imgPoints, img_vis
    
    return False, None, None, img_vis


def detect_aruco_board(image_path, board, camera_matrix=None, dist_coeffs=None, specific_ids=None):
    """
    检测图像中的ArUco标记板
    
    参数:
        image_path: 图像路径
        board: ArUco标记板对象
        camera_matrix: 相机内参矩阵（可选）
        dist_coeffs: 畸变系数（可选）
        specific_ids: 需要检测的特定标记ID列表（可选）
    
    返回:
        成功标志, 世界坐标点, 图像坐标点
    """
    # 读取图像
    img = cv2.imread(image_path)
    if img is None:
        print(f"无法读取图像: {image_path}")
        return False, None, None, None
    
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # 创建ArUco检测器 - 兼容新旧版本API
    try:
        # 尝试使用新版本API
        parameters = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(board.getDictionary(), parameters)
        corners, ids, rejected = detector.detectMarkers(gray)
    except (AttributeError, TypeError):
        # 如果新版本API不可用，使用旧版本API
        parameters = cv2.aruco.DetectorParameters_create()
        dictionary = board.dictionary if hasattr(board, 'dictionary') else cv2.aruco.getPredefinedDictionary(board.get_dict_type())
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)
    
    # 绘制检测结果
    img_vis = img.copy()
    if ids is not None and len(ids) > 0:
        # 如果指定了特定ID，筛选符合的标记
        if specific_ids is not None:
            specific_ids = np.array(specific_ids)
            valid_indices = np.where(np.isin(ids.flatten(), specific_ids))[0]
            if len(valid_indices) == 0:
                print(f"  - 未检测到指定的标记ID {specific_ids}")
                return False, None, None, img_vis
                
            # 筛选符合条件的角点和ID
            corners = [corners[i] for i in valid_indices]
            ids = ids[valid_indices]
            print(f"  - 检测到指定的标记ID: {ids.flatten()}")
        
        cv2.aruco.drawDetectedMarkers(img_vis, corners, ids)
        
        # 保存可视化图像
        calibration_vis_dir = os.path.join(os.path.dirname(image_path), '..', 'calibration_vis')
        if not os.path.exists(calibration_vis_dir):
            os.makedirs(calibration_vis_dir)
        
        vis_path = os.path.join(calibration_vis_dir, os.path.basename(image_path))
        cv2.imwrite(vis_path, img_vis)
        
        # 如果提供了相机参数，则估计标记板姿态
        if camera_matrix is not None and dist_coeffs is not None and len(corners) >= 4:
            try:
                # 尝试使用新版本API
                ret, rvec, tvec = cv2.aruco.estimatePoseBoard(
                    corners, ids, board, camera_matrix, dist_coeffs, None, None)
            except:
                # 如果新版本API不可用，使用旧版本API
                ret, rvec, tvec = cv2.aruco.estimatePoseBoard(
                    corners, ids, board, camera_matrix, dist_coeffs, None, None)
            
            if ret:
                # 在图像上绘制坐标轴
                try:
                    cv2.drawFrameAxes(img_vis, camera_matrix, dist_coeffs, 
                                      rvec, tvec, board.getMarkerLength() * 2)
                except:
                    # 旧版本可能没有drawFrameAxes，或者board.getMarkerLength()
                    marker_length = board.getMarkerLength() if hasattr(board, 'getMarkerLength') else board.getGridSize()[0]
                    cv2.aruco.drawAxis(img_vis, camera_matrix, dist_coeffs, 
                                     rvec, tvec, marker_length * 2)
                
                # 获取标记板的角点
                objPoints, imgPoints = [], []
                
                # 使用检测到的标记的角点
                for i, corner in enumerate(corners):
                    marker_id = ids[i][0]
                    try:
                        # 获取当前标记在板中的3D位置 - 尝试新版本API
                        marker_objp = board.getObjPoints()[marker_id]
                    except (AttributeError, IndexError, TypeError):
                        # 如果不可用，使用旧版本API或者创建一个合理的对象点
                        # 假设标记板是规则排列的
                        marker_size = board.getMarkerLength() if hasattr(board, 'getMarkerLength') else 0.05
                        marker_separation = board.getMarkerSeparation() if hasattr(board, 'getMarkerSeparation') else 0.01
                        
                        # 创建对象点 (基于标记ID的位置)
                        cols = board.getGridSize()[0] if hasattr(board, 'getGridSize') else int(np.sqrt(len(ids)))
                        row = marker_id // cols
                        col = marker_id % cols
                        
                        offset_x = col * (marker_size + marker_separation)
                        offset_y = row * (marker_size + marker_separation)
                        
                        marker_objp = np.array([
                            [offset_x, offset_y, 0],
                            [offset_x + marker_size, offset_y, 0],
                            [offset_x + marker_size, offset_y + marker_size, 0],
                            [offset_x, offset_y + marker_size, 0]
                        ], dtype=np.float32)
                    
                    # 添加到世界坐标点列表
                    objPoints.append(marker_objp)
                    
                    # 添加检测到的角点到图像坐标点列表
                    imgPoints.append(corner[0])
                
                if objPoints and imgPoints:
                    # 将列表转换为NumPy数组
                    objPoints = np.concatenate(objPoints)
                    imgPoints = np.concatenate(imgPoints)
                    
                    return True, objPoints, imgPoints, img_vis
    
    return False, None, None, img_vis


def create_aruco_board(dict_name, board_size, marker_length, marker_separation):
    """
    创建ArUco标记板
    
    参数:
        dict_name: ArUco字典名称
        board_size: 标记板尺寸 (宽, 高) 表示板中标记的行列数
        marker_length: 标记的实际尺寸（米）
        marker_separation: 标记之间的距离（米）
    
    返回:
        ArUco标记板对象
    """
    aruco_dict = get_aruco_dict(dict_name)
    
    try:
        # 尝试使用新版本API
        board = cv2.aruco.GridBoard(
            size=board_size,
            markerLength=marker_length,
            markerSeparation=marker_separation,
            dictionary=aruco_dict
        )
    except (AttributeError, TypeError):
        # 如果新版本API不可用，使用旧版本API
        board = cv2.aruco.GridBoard_create(
            markersX=board_size[0],
            markersY=board_size[1],
            markerLength=marker_length,
            markerSeparation=marker_separation,
            dictionary=aruco_dict
        )
    
    return board


def detect_charuco_board(image_path, pattern_size, square_size):
    """
    检测图像中的棋盘格角点
    """
    # 读取图像
    img = cv2.imread(image_path)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # 寻找棋盘格角点
    ret, corners = cv2.findChessboardCorners(gray, pattern_size, 
                                           cv2.CALIB_CB_ADAPTIVE_THRESH + 
                                           cv2.CALIB_CB_NORMALIZE_IMAGE + 
                                           cv2.CALIB_CB_FAST_CHECK)
    
    if ret:
        # 亚像素优化角点位置
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        
        # 绘制角点 (可视化)
        cv2.drawChessboardCorners(img, pattern_size, corners, ret)
        calibration_vis_dir = os.path.join(os.path.dirname(image_path), '..', 'calibration_vis')
        if not os.path.exists(calibration_vis_dir):
            os.makedirs(calibration_vis_dir)
        
        vis_path = os.path.join(calibration_vis_dir, os.path.basename(image_path))
        cv2.imwrite(vis_path, img)
        
        # 创建世界坐标系中的棋盘格点
        objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
        objp = objp * square_size  # 实际尺寸（米）
        
        return True, objp, corners
    
    return False, None, None


def calibrate_camera(object_points, image_points, image_size):
    """
    相机内参标定
    """
    flags = (cv2.CALIB_RATIONAL_MODEL + cv2.CALIB_FIX_PRINCIPAL_POINT)
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        object_points, image_points, image_size, None, None, flags=flags)
    
    return ret, mtx, dist, rvecs, tvecs


def load_camera_matrix(camera_matrix_file):
    """
    从JSON文件中加载相机内参矩阵和畸变系数
    
    参数:
        camera_matrix_file: JSON文件路径，包含相机内参和畸变系数
        
    返回:
        mtx: 相机内参矩阵
        dist: 畸变系数
    """
    try:
        with open(camera_matrix_file, 'r') as f:
            data = json.load(f)
            
        # 处理 'k' 格式的相机矩阵 (3x3)
        if 'k' in data:
            k = np.array(data['k'])
            if len(k) == 9:  # 扁平化的3x3矩阵
                mtx = np.array([
                    [k[0], k[1], k[2]],
                    [k[3], k[4], k[5]],
                    [k[6], k[7], k[8]]
                ])
            else:
                raise ValueError("相机矩阵格式不正确")
                
        # 处理 'camera_matrix' 格式
        elif 'camera_matrix' in data:
            mtx = np.array(data['camera_matrix'])
            
        else:
            raise ValueError("找不到相机矩阵数据")
            
        # 处理畸变系数
        if 'd' in data:
            dist = np.array(data['d'])
        elif 'distortion_coefficients' in data:
            dist = np.array(data['distortion_coefficients'])
        else:
            print("警告: 未找到畸变系数，使用零畸变")
            dist = np.zeros(5)
            
        # print(f"成功从 {camera_matrix_file} 加载相机内参")
        # print("相机矩阵:")
        # print(mtx)
        # print("畸变系数:")
        # print(dist)
        
        return mtx, dist
    
    except Exception as e:
        print(f"加载相机内参时出错: {e}")
        return None, None


def eye_in_hand_calibration(camera_poses, robot_poses):
    """
    眼在手上标定，计算相机相对于机器人末端执行器的变换
    
    参数:
        camera_poses: 相机姿态列表，每个姿态是一个包含旋转向量和平移向量的元组 (R, T)
        robot_poses: 机器人末端执行器姿态列表，每个姿态是一个4x4变换矩阵
    
    返回:
        camera_to_robot: 相机到机器人末端执行器的变换矩阵
    """
    # 准备旋转矩阵和平移向量列表
    R_gripper2base = []
    t_gripper2base = []
    R_target2cam = []
    t_target2cam = []
    
    for i in range(len(camera_poses)):
        # 相机到标定板的变换
        R_cam, t_cam = camera_poses[i]
        R_target2cam.append(R_cam)
        t_target2cam.append(t_cam)
        
        # 机器人末端到基座的变换
        R_gripper2base.append(robot_poses[i][:3, :3])
        t_gripper2base.append(robot_poses[i][:3, 3])
    
    # 执行手眼标定 (相机在机器人上，即"眼在手上"情况)
    R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
        R_gripper2base, t_gripper2base, R_target2cam, t_target2cam,
        method=cv2.CALIB_HAND_EYE_TSAI
    )
    
    # 构建相机到机器人末端的变换矩阵
    cam2gripper = np.eye(4)
    cam2gripper[:3, :3] = R_cam2gripper
    cam2gripper[:3, 3] = t_cam2gripper.reshape(-1)
    
    return cam2gripper


def main():
    parser = argparse.ArgumentParser(description='眼在手上的手眼标定')
    parser.add_argument('--data_path', type=str, required=True, 
                        help='手眼标定数据路径')
    parser.add_argument('--pattern_size', type=int, nargs=2, default=None,
                        help='棋盘格内角点数量，例如：8 6 表示8x6的棋盘格')
    parser.add_argument('--square_size', type=float, default=0.025,
                        help='棋盘格方格的实际尺寸（米）')
    parser.add_argument('--camera_matrix_file', type=str, default=None,
                        help='相机内参矩阵和畸变系数的JSON文件路径，如果提供则跳过相机标定')
    parser.add_argument('--aruco_dict', type=str, default=None,
                        help='ArUco字典名称，例如DICT_6X6_250')
    parser.add_argument('--aruco_size', type=float, default=0.05,
                        help='ArUco标记的实际尺寸（米）')
    parser.add_argument('--aruco_ids', type=int, nargs='+', default=None,
                        help='指定要检测的ArUco标记ID，例如：0 1 2 3')
    parser.add_argument('--aruco_board', type=float, nargs=4, default=None,
                        help='ArUco标记板参数: 宽度(标记数) 高度(标记数) 标记尺寸(米) 标记间距(米)')
    parser.add_argument('--calibration_method', type=str, default='tsai',
                        choices=['tsai', 'park', 'horaud', 'andreff', 'daniilidis'],
                        help='手眼标定方法')
    args = parser.parse_args()
    
    # 图像和位姿数据路径
    image_dir = os.path.join(args.data_path, 'images')
    pose_dir = os.path.join(args.data_path, 'poses')
    
    # 获取图像文件列表并排序
    image_files = sorted(glob.glob(os.path.join(image_dir, 'image_*.png')))
    
    if not image_files:
        print(f"错误：在 {image_dir} 中未找到图像文件")
        return
    
    # 加载机器人位姿
    robot_poses = load_poses(pose_dir)
    
    # 检查参数，确定使用的标定模式
    calibration_mode = None
    if args.pattern_size is not None:
        calibration_mode = 'chessboard'
    elif args.aruco_dict is not None:
        calibration_mode = 'aruco'
    elif args.aruco_board is not None:
        calibration_mode = 'aruco_board'
    else:
        # 默认使用棋盘格模式，尺寸为8x6
        calibration_mode = 'chessboard'
        args.pattern_size = [8, 6]
    
    # 检测角点并准备标定数据
    object_points = []  # 3D点
    image_points = []   # 2D点
    camera_poses = []   # 相机姿态
    valid_robot_poses = []  # 有效的机器人姿态
    
    print(f"正在处理 {len(image_files)} 个图像...")
    valid_count = 0

    # 如果提供了相机内参文件，加载现有内参
    mtx, dist = None, None
    if args.camera_matrix_file:
        mtx, dist = load_camera_matrix(args.camera_matrix_file)
        if mtx is None or dist is None:
            print("无法加载相机内参文件，退出")
            return
    
    # 如果使用ArUco标记板，创建标记板对象
    board = None
    if calibration_mode == 'aruco_board':
        board_width, board_height, marker_length, marker_separation = args.aruco_board
        board_size = (int(board_width), int(board_height))
        dict_name = args.aruco_dict if args.aruco_dict else 'DICT_6X6_250'
        board = create_aruco_board(dict_name, board_size, marker_length, marker_separation)
    
    # 处理每张图像
    for i, image_file in enumerate(image_files):
        print(f"处理图像 {i+1}/{len(image_files)}: {os.path.basename(image_file)}")
        
        if calibration_mode == 'chessboard':
            # 检测棋盘格角点
            ret, objp, corners = detect_charuco_board(
                image_file, tuple(args.pattern_size), args.square_size)
            
            if ret:
                print(f"  - 成功检测到棋盘格角点")
                image = cv2.imread(image_file)
                img_size = (image.shape[1], image.shape[0])
                
                object_points.append(objp)
                image_points.append(corners)
                
                # 解算相机位姿
                if mtx is not None and dist is not None:
                    # 使用已有的相机参数进行PnP解算
                    ret, rvec, tvec = cv2.solvePnP(objp, corners, mtx, dist)
                    camera_poses.append((cv2.Rodrigues(rvec)[0], tvec))
                    valid_robot_poses.append(robot_poses[i])
                    valid_count += 1
                else:
                    # 暂存数据，待相机标定后再解算位姿
                    valid_count += 1
                
        elif calibration_mode == 'aruco':
            # 检测ArUco标记
            aruco_dict = get_aruco_dict(args.aruco_dict)
            ret, objp, corners, img_vis = detect_aruco_markers(
                image_file, aruco_dict, args.aruco_size, mtx, dist, args.aruco_ids)
            
            if ret:
                print(f"  - 成功检测到ArUco标记")
                image = cv2.imread(image_file)
                img_size = (image.shape[1], image.shape[0])
                
                object_points.append(objp)
                image_points.append(corners)
                
                # 如果已有相机参数，使用PnP解算相机位姿
                if mtx is not None and dist is not None:
                    # 使用所有检测到的角点一次性解算位姿
                    ret, rvec, tvec = cv2.solvePnP(objp, corners, mtx, dist)
                    camera_poses.append((cv2.Rodrigues(rvec)[0], tvec))
                    valid_robot_poses.append(robot_poses[i])
                    valid_count += 1
                else:
                    # 暂存数据，待相机标定后再解算位姿
                    valid_count += 1
        
        elif calibration_mode == 'aruco_board':
            # 检测ArUco标记板
            ret, objp, corners, img_vis = detect_aruco_board(
                image_file, board, mtx, dist, args.aruco_ids)
            
            if ret:
                print(f"  - 成功检测到ArUco标记板")
                image = cv2.imread(image_file)
                img_size = (image.shape[1], image.shape[0])
                
                object_points.append(objp)
                image_points.append(corners)
                
                # 如果已有相机参数，使用PnP解算相机位姿
                if mtx is not None and dist is not None:
                    ret, rvec, tvec = cv2.solvePnP(objp, corners, mtx, dist)
                    camera_poses.append((cv2.Rodrigues(rvec)[0], tvec))
                    valid_robot_poses.append(robot_poses[i])
                    valid_count += 1
                else:
                    # 暂存数据，待相机标定后再解算位姿
                    valid_count += 1
            
    print(f"共有 {valid_count}/{len(image_files)} 个有效的标定图像")
    
    if valid_count < 3:
        print("错误：有效的标定图像数量太少，至少需要3个")
        return
    
    # 相机内参标定或加载
    if args.camera_matrix_file:
        # 使用提供的相机内参
        print("使用提供的相机内参文件，跳过相机标定步骤")
        mtx, dist = load_camera_matrix(args.camera_matrix_file)
        
        # 如果camera_poses为空，需要计算相机位姿
        if not camera_poses:
            camera_poses = []
            valid_robot_poses = []
            for i, (objp, imgp) in enumerate(zip(object_points, image_points)):
                ret, rvec, tvec = cv2.solvePnP(objp, imgp, mtx, dist)
                if ret:
                    camera_poses.append((cv2.Rodrigues(rvec)[0], tvec))
                    valid_robot_poses.append(robot_poses[i])
    else:
        # 进行相机内参标定
        print("正在进行相机内参标定...")
        img = cv2.imread(image_files[0])
        img_size = (img.shape[1], img.shape[0])
        ret, mtx, dist, rvecs, tvecs = calibrate_camera(
            object_points, image_points, img_size)
        
        print("相机内参矩阵:")
        print(mtx)
        print("\n畸变系数:")
        print(dist)
        
        # 计算相机位姿
        camera_poses = []
        valid_robot_poses = []
        for i, (objp, imgp) in enumerate(zip(object_points, image_points)):
            ret, rvec, tvec = cv2.solvePnP(objp, imgp, mtx, dist)
            if ret:
                camera_poses.append((cv2.Rodrigues(rvec)[0], tvec))
                valid_robot_poses.append(robot_poses[i])
    
    # 计算重投影误差
    # mean_error = 0
    # num_points = 0
    # for i in range(len(object_points)):
    #     # 使用camera_poses中的姿态进行投影
    #     if i < len(camera_poses):
    #         R, t = camera_poses[i]
    #         rvec, _ = cv2.Rodrigues(R)
    #         imgpoints2, _ = cv2.projectPoints(object_points[i], rvec, t, mtx, dist)
    #         error = cv2.norm(image_points[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
    #         mean_error += error
    #         num_points += 1
    
    # if num_points > 0:
    #     print(f"\n平均重投影误差: {mean_error/num_points} 像素")
    
    # 选择手眼标定方法
    calibration_methods = {
        'tsai': cv2.CALIB_HAND_EYE_TSAI,
        'park': cv2.CALIB_HAND_EYE_PARK,
        'horaud': cv2.CALIB_HAND_EYE_HORAUD,
        'andreff': cv2.CALIB_HAND_EYE_ANDREFF,
        'daniilidis': cv2.CALIB_HAND_EYE_DANIILIDIS
    }
    
    method = calibration_methods.get(args.calibration_method, cv2.CALIB_HAND_EYE_ANDREFF)
    method_name = args.calibration_method.upper()
    
    # 眼在手上标定
    print(f"\n正在使用 {method_name} 方法进行眼在手上的手眼标定...")
    
    # 准备旋转矩阵和平移向量列表
    R_gripper2base = []
    t_gripper2base = []
    R_target2cam = []
    t_target2cam = []
    
    for i in range(len(camera_poses)):
        # 相机到标定板的变换
        R_cam, t_cam = camera_poses[i]
        R_target2cam.append(R_cam)
        t_target2cam.append(t_cam)
        
        # 机器人末端到基座的变换
        R_gripper2base.append(valid_robot_poses[i][:3, :3])
        t_gripper2base.append(valid_robot_poses[i][:3, 3])
    
    # 执行手眼标定 (相机在机器人上，即"眼在手上"情况)
    R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
        R_gripper2base, t_gripper2base, R_target2cam, t_target2cam,
        method=method
    )
    
    # 构建相机到机器人末端的变换矩阵
    cam2gripper = np.eye(4)
    cam2gripper[:3, :3] = R_cam2gripper
    cam2gripper[:3, 3] = t_cam2gripper.reshape(-1)
    
    print("\n相机到机器人末端执行器的变换矩阵:")
    print(cam2gripper)

    print("\n相机平移向量:")
    print(cam2gripper[:3, 3])
    print("\n相机旋转矩阵:")
    print(cam2gripper[:3, :3])
    
    # 将结果保存到JSON文件
    calibration_type = 'chessboard' if calibration_mode == 'chessboard' else 'aruco'
    result_file = os.path.join(args.data_path, f'calibration_result_{calibration_type}_{time.strftime("%Y%m%d_%H%M%S")}.json')
    
    # 提取旋转矩阵和平移向量
    R_cam2gripper = cam2gripper[:3, :3]
    t_cam2gripper = cam2gripper[:3, 3]
    
    # 转换为欧拉角表示（更易于理解）
    r = Rotation.from_matrix(R_cam2gripper)
    euler_angles = r.as_euler('xyz', degrees=True)
    
    # 将数据转换为可序列化的格式
    calibration_result = {
        "calibration_type": calibration_type,
        "calibration_method": args.calibration_method,
        "camera_matrix": mtx.tolist(),
        "distortion_coefficients": dist.tolist(),
        "camera_to_gripper": {
            "translation": {
                "x": float(t_cam2gripper[0]),
                "y": float(t_cam2gripper[1]),
                "z": float(t_cam2gripper[2])
            },
            "rotation_matrix": R_cam2gripper.tolist(),
            "euler_angles_xyz_degrees": euler_angles.tolist(),
            "transformation_matrix": cam2gripper.tolist()
        },
        # "reprojection_error": float(mean_error/num_points) if num_points > 0 else 0.0,
        "valid_images": valid_count,
        "total_images": len(image_files)
    }
    
    with open(result_file, 'w') as f:
        json.dump(calibration_result, f, indent=2)
    
    print(f"\n标定结果已保存到: {result_file}")
    print("\n标定完成!")


if __name__ == "__main__":
    main()
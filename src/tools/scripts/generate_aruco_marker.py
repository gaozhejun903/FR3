#!/usr/bin/env python3

"""
生成指定编号的ArUco标记，支持不同字典类型

用法:
    python3 generate_aruco_marker.py --id 24 --dict DICT_6X6_250 --size 200 --output marker_24.png
"""

import argparse
import cv2
import numpy as np

def generate_aruco_marker(marker_id, dict_type, size):
    """
    生成指定编号的ArUco标记
    
    参数:
        marker_id: 标记的ID号
        dict_type: 字典类型名称
        size: 图像尺寸（像素）
        
    返回:
        生成的标记图像
    """
    # 支持的ArUco字典类型
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
    
    if dict_type not in aruco_dicts:
        raise ValueError(f"未知的ArUco字典类型: {dict_type}")
    
    # 获取指定的字典
    try:
        # 尝试新版本API
        dictionary = cv2.aruco.getPredefinedDictionary(aruco_dicts[dict_type])
        # 生成标记
        marker_image = cv2.aruco.generateImageMarker(dictionary, marker_id, size)
    except AttributeError:
        # 兼容旧版本API
        dictionary = cv2.aruco.Dictionary_get(aruco_dicts[dict_type])
        # 生成标记
        marker_image = np.zeros((size, size), dtype=np.uint8)
        marker_image = cv2.aruco.drawMarker(dictionary, marker_id, size, marker_image, 1)
        
    # 添加白边，便于打印和识别
    border = int(size * 0.1)  # 边框宽度为图像尺寸的10%
    img_with_border = np.ones((size + 2*border, size + 2*border), dtype=np.uint8) * 255
    img_with_border[border:border+size, border:border+size] = marker_image
    
    return img_with_border

def main():
    """
    主函数
    """
    parser = argparse.ArgumentParser(description='生成ArUco标记')
    parser.add_argument('--id', type=int, required=True, help='标记的ID号')
    parser.add_argument('--dict', type=str, default='DICT_6X6_250', 
                        help='ArUco字典类型，例如: DICT_6X6_250')
    parser.add_argument('--size', type=int, default=200, 
                        help='图像大小（像素）')
    parser.add_argument('--output', type=str, default=None, 
                        help='输出文件路径，如果不提供则使用默认命名')
    args = parser.parse_args()
    
    # 生成标记
    marker_image = generate_aruco_marker(args.id, args.dict, args.size)
    
    # 保存标记
    if args.output:
        output_path = args.output
    else:
        output_path = f"aruco_marker_{args.dict}_{args.id}.png"
    
    cv2.imwrite(output_path, marker_image)
    print(f"标记已保存到: {output_path}")

if __name__ == "__main__":
    main()
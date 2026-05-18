#!/usr/bin/env python3

import rospy
import tf2_ros
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler, euler_from_quaternion

def main():
    rospy.init_node('fake_gripper_tf_publisher')
    
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    
    br = tf2_ros.TransformBroadcaster()
    
    rate = rospy.Rate(10.0)  # 10 Hz
    
    # 源坐标系和目标坐标系的名称
    gripper_frame = rospy.get_param('~gripper_frame', 'gripper_link')  # 夹爪的坐标系
    base_frame = rospy.get_param('~base_frame', 'base_link')  # 基座坐标系
    fake_frame = rospy.get_param('~fake_frame', 'fake_gripper_frame')  # 要发布的虚假坐标系名称
    
    rospy.loginfo(f"Publishing fake TF from {gripper_frame} with orientation from {base_frame}")
    
    while not rospy.is_shutdown():
        try:
            # 获取夹爪相对于世界的变换
            gripper_trans = tfBuffer.lookup_transform('world', gripper_frame, rospy.Time())
            
            # 获取基座相对于世界的变换
            base_trans = tfBuffer.lookup_transform('world', base_frame, rospy.Time())
            
            # 创建新的变换
            trans = geometry_msgs.msg.TransformStamped()
            trans.header.stamp = rospy.Time.now()
            trans.header.frame_id = 'world'
            trans.child_frame_id = fake_frame
            
            # 使用夹爪的位置
            trans.transform.translation.x = gripper_trans.transform.translation.x
            trans.transform.translation.y = gripper_trans.transform.translation.y
            trans.transform.translation.z = gripper_trans.transform.translation.z
            
            # 使用基座的姿态
            trans.transform.rotation = base_trans.transform.rotation
            
            # 发布变换
            br.sendTransform(trans)
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"TF Error: {e}")
            
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
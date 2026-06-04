# ╔══════════════════════════════════════════════════════════════════════╗
# ║  AI-Deep: 此文件已废弃!                                           ║
# ║  - 原因: 使用 ROS1 API (rospy)，ROS2 Humble 不可用               ║
# ║  - 替代: fake_gripper_tf_publisher_node (C++ ROS2 版本)           ║
# ║  - 已集成到: ros2 launch depth_handler depth_full.launch.py       ║
# ╚══════════════════════════════════════════════════════════════════════╝
# #!/usr/bin/env python3
# import rospy
# import tf2_ros
# import geometry_msgs.msg
# from tf.transformations import quaternion_from_euler, euler_from_quaternion
#
# def main():
#     rospy.init_node('fake_gripper_tf_publisher')
#     tfBuffer = tf2_ros.Buffer()
#     listener = tf2_ros.TransformListener(tfBuffer)
#     br = tf2_ros.TransformBroadcaster()
#     rate = rospy.Rate(10.0)
#     gripper_frame = rospy.get_param('~gripper_frame', 'gripper_link')
#     base_frame = rospy.get_param('~base_frame', 'base_link')
#     fake_frame = rospy.get_param('~fake_frame', 'fake_gripper_frame')
#     rospy.loginfo(f"Publishing fake TF from {gripper_frame} with orientation from {base_frame}")
#     while not rospy.is_shutdown():
#         try:
#             gripper_trans = tfBuffer.lookup_transform('world', gripper_frame, rospy.Time())
#             base_trans = tfBuffer.lookup_transform('world', base_frame, rospy.Time())
#             trans = geometry_msgs.msg.TransformStamped()
#             trans.header.stamp = rospy.Time.now()
#             trans.header.frame_id = 'world'
#             trans.child_frame_id = fake_frame
#             trans.transform.translation.x = gripper_trans.transform.translation.x
#             trans.transform.translation.y = gripper_trans.transform.translation.y
#             trans.transform.translation.z = gripper_trans.transform.translation.z
#             trans.transform.rotation = base_trans.transform.rotation
#             br.sendTransform(trans)
#         except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
#             rospy.logwarn(f"TF Error: {e}")
#         rate.sleep()
# if __name__ == '__main__':
#     try: main()
#     except rospy.ROSInterruptException: pass

#!/usr/bin/env python3

import rospy
import geometry_msgs.msg
from tf2_msgs.msg import TFMessage
import tf.transformations as tr
import numpy as np

# Posa della camera rispetto alla base del robot

# T1 = np.array([-0.440, 0.160, 0.444])
# q1 = [-0.553, 0.314, -0.380, 0.672]
# T1 = np.array([-0.452, 0.159, 0.456])
# q1 = [-0.493, 0.246, -0.372, 0.747]
T1 = np.array([-0.465, 0.150, 0.464])
q1 = [-0.415, 0.207, -0.395, 0.793]

RT1 = tr.quaternion_matrix(q1)
RT1[:3,3] = T1

# Posa stimata dell'oggetto rispetto alla camera
n_object = rospy.get_param("n_object")
child_frame = []
s = np.zeros((n_object, 7))
for i in range(n_object):
    object_name = rospy.get_param("object_name_" + str(i+1))
    print('Oggetto numero '+ str(i+1) +': '+ object_name)
    data = np.loadtxt('/home/nuvo/catkin_ws/src/visp_megapose/output/box'+ str(i) + '_pose.txt').astype(np.float64) #Load object pose
    (r,p,y) = tr.euler_from_quaternion([data[3], data[4], data[5], data[6]], 'sxyz')
    RT2 = tr.euler_matrix(r, p, y, 'sxyz')
    RT2[0,3] = data[0]
    RT2[1,3] = data[1]
    RT2[2,3] = data[2]

    T12 = np.matmul(RT1,RT2) 

    # Trasformazione per portare frame sulla superficie dell'oggetto
    if object_name == 'box1':
      a =  0.063
    elif object_name == 'box2':
      a = 0.024
    elif object_name == 'box3':
      a = 0.054
    elif object_name == 'box4':
      a = 0.04
    elif object_name == 'box5':
      a = 0.06
    else:
      rospy.logerr("%s is not a valid model", object_name)


    T = np.array([[    1,   0.0,    0.0,   0.0],
                  [   0.0,    1,    0.0,     a],
                  [   0.0,  0.0,      1,   0.0],
                  [   0.0,  0.0,    0.0,     1]])

    T12 = np.matmul(T12,T) 

    q12 = tr.quaternion_from_matrix(T12)
    T12 = T12[0:3,3]
    child_frame = child_frame + ['object' + str(i)]

    s[i, 0:3] = T12
    s[i, 3:] = q12
    
print('Pose degli oggetti da prelevare pubblicate sul topic tf')

if __name__ == '__main__':
    rospy.init_node('object_pose')
    pub_tf = rospy.Publisher("/tf", TFMessage, queue_size=10)
    header_frame = 'robot_arm_link0'

    while not rospy.is_shutdown():
            rospy.sleep(0.1)

            for i in range(n_object):
               t1 = geometry_msgs.msg.TransformStamped()
               t1.header.frame_id = header_frame
               t1.header.stamp = rospy.Time.now()
               t1.child_frame_id = child_frame[i]
               t1.transform.translation.x = s[i][0]
               t1.transform.translation.y = s[i][1]
               t1.transform.translation.z = s[i][2]
               t1.transform.rotation.x = s[i][3]
               t1.transform.rotation.y = s[i][4]
               t1.transform.rotation.z = s[i][5]
               t1.transform.rotation.w = s[i][6]
               tfm1 = TFMessage([t1])
               pub_tf.publish(tfm1)

    rospy.spin()

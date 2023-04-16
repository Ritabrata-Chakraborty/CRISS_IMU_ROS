#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3

def imu_publisher():
    # Initialize the node
    rospy.init_node('imu_publisher', anonymous=True)

    # Create a publisher for the IMU data
    imu_pub = rospy.Publisher('imu_data', Imu, queue_size=10)

    # Set the loop rate (10 Hz)
    rate = rospy.Rate(10)

    # Initialize the IMU message
    imu_msg = Imu()

    # Set the linear acceleration in the x-axis
    linear_acc_x = 2.5  # m/s^2

    # Set the gravitational acceleration in the z-axis
    gravity_z = 9.8  # m/s^2

    # Set the initial angular acceleration in the z-axis
    angular_acc_z = 1.0  # rad/s^2

    # Set the initial orientation
    imu_msg.orientation.x = 0.0
    imu_msg.orientation.y = 0.0
    imu_msg.orientation.z = 0.0
    imu_msg.orientation.w = 1.0

    # Set the initial angular velocity
    imu_msg.angular_velocity.x = 0.0
    imu_msg.angular_velocity.y = 0.0
    imu_msg.angular_velocity.z = 0.0

    # Set the initial linear acceleration
    imu_msg.linear_acceleration.x = linear_acc_x
    imu_msg.linear_acceleration.y = 0.0
    imu_msg.linear_acceleration.z = gravity_z

    # Set the timestamp
    imu_msg.header.stamp = rospy.Time.now()

    # Publish the initial IMU message
    imu_pub.publish(imu_msg)

    # Loop for 10 seconds
    for i in range(100):
        # Update the angular acceleration in the z-axis
        angular_acc_z += 0.09

        # Update the angular velocity in the z-axis
        imu_msg.angular_velocity.z += angular_acc_z * 0.1

        # Update the orientation
        imu_msg.orientation.z += imu_msg.angular_velocity.z * 0.1

        # Update the timestamp
        imu_msg.header.stamp = rospy.Time.now()

        # Publish the IMU message
        imu_pub.publish(imu_msg)

        # Sleep to maintain the loop rate
        rate.sleep()

if __name__ == '__main__':
    try:
        imu_publisher()
    except rospy.ROSInterruptException:
        pass

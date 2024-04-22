#!/usr/bin/env python3

import rospy
from mavros_msgs.srv import CommandTOL, SetMode
from mavros_msgs.srv import CommandBool
from mavros_msgs.msg import State
from mavros_msgs.msg import PositionTarget
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3
import math 
import tf.transformations

class execute_circle:

    def __init__(self):

        rospy.init_node('execute_circle_node', anonymous=True)
        self.rate = rospy.Rate(10)

        # Create a publisher for the position target
        self.pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)

        # Create a subscriber for position data
        self.sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback)

        # Global variable
        self.pose_obj = PoseStamped()
        self.euler_yaw = Vector3()

        # ROS service clients
        self.takeoff_service = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        self.land_service = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
        self.arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)    

        # Wait for the connection to FCU (Flight Controller Unit)
        while not rospy.is_shutdown() and not self.is_connected():
            rospy.loginfo("Waiting for FCU connection...")
            rospy.sleep(2)

        # Change flight mode to GUIDED
        if self.set_mode("GUIDED"):
            rospy.loginfo("Mode changed to GUIDED")
            rospy.sleep(2)  # Wait for 2 seconds after changing mode
            self.arm_motors()
        else:
            rospy.logerr("Failed to change mode to GUIDED")

    def pose_callback(self, msg):
        self.pose_obj = PoseStamped()
        self.pose_obj = msg
        self.euler_yaw = self.quat_to_eul(msg.pose.orientation)
    
    def is_connected(self):
        state_msg = rospy.wait_for_message('/mavros/state', State, timeout=60)
        rospy.loginfo("Connected with FCU")
        return state_msg.connected
    
    def set_mode(self, mode):
        try:
            response = self.set_mode_service(0, mode)
            return response.mode_sent
        except rospy.ServiceException as e:
            rospy.logerr("Mode change service call failed: " + str(e))
            return False
    def arm_motors(self):
        try:
            response = self.arm_service(True)
            if response.success:
                rospy.loginfo("Motors armed")
                rospy.sleep(4)  # Wait for 4 seconds after arming motors
                self.takeoff()
            else:
                rospy.logerr("Failed to arm motors")
        except rospy.ServiceException as e:
            rospy.logerr("Arming service call failed: " + str(e))
        
    def takeoff(self):

        try:
            response = self.takeoff_service(altitude=1.0, latitude=0.0, longitude=0.0, min_pitch=0.0, yaw=0.0)
            
            if response.success:
                rospy.loginfo("Takeoff successful")
                rospy.sleep(10)  # Wait for 7 seconds after takeoff

                # Move forward by 1.5 m
                self.publish_position_target(3.75, 2.50)

                rospy.sleep(2)

                # Turn left by 90 degrees
                self.turn_left_90_degrees()

                rospy.sleep(1)

                # Execute circle of radius = 1.5 m
                self.execute_circle()

                rospy.sleep(2)

                # Move to the origin
                self.publish_position_target(2.3, 2.5)

                rospy.sleep(2)

                # Turn around by 180 degrees
                self.turn_left_180_degrees()

                rospy.sleep(1)

                # Land
                self.land()

            rospy.signal_shutdown("Task completed")

        except rospy.ServiceException as e:
            rospy.logerr("Takeoff service call failed: " + str(e))

    def land(self):
        try:
            response = self.land_service(altitude=0.0, latitude=0.0, longitude=0.0, min_pitch=0.0, yaw=0.0)
            if response.success:
                rospy.loginfo("Landing command sent.")
                rospy.sleep(10)
            else:
                rospy.logerr("Failed to send landing command.")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: " + str(e))

    def quat_to_eul(self, quaternion):
        
        # Convert quaternion to Euler angles (roll, pitch, yaw)
        euler = tf.transformations.euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        
        # Extract and return the yaw angle (in radians)
        return euler[2]
    

    def turn_left_90_degrees(self):
        # Create a PositionTarget message
        position_target_msg = PositionTarget()

        # Set the coordinate frame value to 8 (LOCAL_NED)
        position_target_msg.coordinate_frame = 8

        # Set the bitmask to control only the yaw_rate
        position_target_msg.type_mask = 1987

        # Set the desired yaw rate to 0.1 rad/s
        position_target_msg.yaw_rate = 0.10

        # Initialize the initial yaw angle
        initial_yaw = self.euler_yaw

        # Loop until the drone turns left by 90 degrees
        while not rospy.is_shutdown():
            # Publish the yaw rate command
            self.pub.publish(position_target_msg)

            # Calculate the current yaw angle
            current_yaw = self.euler_yaw

            # Check if the drone has turned by 90 degrees (approximately)
            if abs(current_yaw - initial_yaw) >= 1.5708:  # 1.5708 radians is approximately 90 degrees
                break

            # Sleep to achieve the desired rate
            self.rate.sleep()

        # Stop the drone
        position_target_msg.yaw_rate = 0.0
        self.pub.publish(position_target_msg)


    def turn_left_180_degrees(self):
        # Create a PositionTarget message
        position_target_msg = PositionTarget()

        # Set the coordinate frame value to 8 (LOCAL_NED)
        position_target_msg.coordinate_frame = 8

        # Set the bitmask to control only the yaw_rate
        position_target_msg.type_mask = 1987

        # Set the desired yaw rate to 0.1 rad/s
        position_target_msg.yaw_rate = 0.10

        # Initialize the initial yaw angle
        initial_yaw = self.euler_yaw

        # Loop until the drone turns left by 180 degrees
        while not rospy.is_shutdown():
            # Publish the yaw rate command
            self.pub.publish(position_target_msg)

            # Calculate the current yaw angle
            current_yaw = self.euler_yaw

            # Check if the drone has turned by 90 degrees (approximately)
            if abs(current_yaw - initial_yaw) >= 3.1415:  # 3.1415 radians is approximately 180 degrees
                break

            # Sleep to achieve the desired rate
            self.rate.sleep()

        # Stop the drone
        position_target_msg.yaw_rate = 0.0
        self.pub.publish(position_target_msg)


    def execute_circle(self):

        while not rospy.is_shutdown() and not(self.pose_obj.pose.position.x > 3.25 and self.pose_obj.pose.position.x < 4.25 and self.pose_obj.pose.position.y < 2.40 and self.pose_obj.pose.position.y > 2.20):
            
            rospy.loginfo("Traversing circle")
            
            # Create a PositionTarget message
            position_target_msg = PositionTarget()

            # Set the coordinate frame value to 8 (LOCAL_NED) ~ body frame velocities
            position_target_msg.coordinate_frame = 8

            # Set the bitmask to 1987 to control position, velocity, acceleration, yaw, and yaw_rate
            position_target_msg.type_mask = 1987

            # Set the desired velocity in the x-axis to 0.15 m/s
            position_target_msg.velocity.x = 0.15

            # Set the desired yaw rate to 0.1 rad/s
            position_target_msg.yaw_rate = 0.1

            # Publish the message
            self.pub.publish(position_target_msg)

            rospy.loginfo("X pos: %.3f",  self.pose_obj.pose.position.x)
            rospy.loginfo("Y pos: %.3f",  self.pose_obj.pose.position.y)

            # Sleep to achieve the desired rate
            self.rate.sleep()

        print("Done at:", self.pose_obj.pose.position.x, self.pose_obj.pose.position.y)

        
        # Create a PositionTarget message
        position_target_msg = PositionTarget()

        # Stopping the drone
        position_target_msg.velocity.x = 0.0
        position_target_msg.yaw_rate = 0.0
        self.pub.publish(position_target_msg)

        rospy.loginfo("Circle formed")


    def publish_position_target(self, x, y):

        goal_x = x
        goal_y = y
        c = 0

        # Create a PositionTarget message
        position_target_msg = PositionTarget()

        # Set the coordinate frame value to 8 (LOCAL_NED)
        position_target_msg.coordinate_frame = 8

        # Set the bitmask to 1987 to control position, velocity, acceleration, yaw, and yaw_rate
        position_target_msg.type_mask = 1987

        p_controller_linear = 0.6
        # p_controller_angular = 0.6
        
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():

            current_x = self.pose_obj.pose.position.x
            current_y = self.pose_obj.pose.position.y
            current_yaw = self.euler_yaw

            if current_x and current_y and current_yaw is not None:
                
                while not rospy.is_shutdown():

                    current_x = self.pose_obj.pose.position.x
                    current_y = self.pose_obj.pose.position.y
                    current_yaw = self.euler_yaw

                    dist = abs(math.sqrt(((goal_x - current_x) ** 2) +
                               ((goal_y - current_y) ** 2)))

                    if (dist < 0.15):
                        c = c + 1
                        break

                    linear_speed = dist * p_controller_linear

                    # set upper limit of linear velocity
                    linear_speed = min(linear_speed, 0.10)
                    # set lower limit of linear velocity
                    linear_speed = max(linear_speed, 0.02)

                    angle_to_goal = math.atan2(
                        goal_y - current_y, goal_x - current_x)

                    if (current_yaw < 0):
                        yaw = 6.28 - abs(current_yaw)

                    else:
                        yaw = current_yaw

                    if (angle_to_goal < 0):
                        angle_to_goal = 6.28 - abs(angle_to_goal)

                    delta_heading = math.atan2(
                        math.sin(angle_to_goal - yaw), math.cos(angle_to_goal - yaw))

                    if abs(angle_to_goal - yaw) < 0.02:
                        angular_speed = 0

                    if delta_heading <= 0.15 and delta_heading >=-0.15:
                        angular_speed = delta_heading

                    elif delta_heading > 0.15:
                        angular_speed = 0.15

                    elif delta_heading < -0.15:
                        angular_speed = -0.15

                    if abs(angular_speed) > 0.08:
                       linear_speed = 0.0

                    position_target_msg.velocity.x = linear_speed
                    position_target_msg.yaw_rate = angular_speed

                    self.pub.publish(position_target_msg)
                    rate.sleep()

                break

        position_target_msg.velocity.x = 0.0
        position_target_msg.yaw_rate = 0.0

        if c > 0:
            self.pub.publish(position_target_msg)
            print("Done")
            c = 0

        else:
            rospy.signal_shutdown("kill")
            print("Keyboard interrupt!")


if __name__ == '__main__':
    try:
        node = execute_circle()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
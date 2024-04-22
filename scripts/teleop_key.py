#!/usr/bin/env python3

import rospy
from mavros_msgs.srv import CommandTOL, SetMode
from mavros_msgs.srv import CommandBool
from mavros_msgs.msg import State
from mavros_msgs.msg import PositionTarget
from geometry_msgs.msg import PoseStamped
import sys, select, os, tty, termios

class teleoperation:

    def __init__(self):

        rospy.init_node('teleop_node', anonymous=True)
        self.rate = rospy.Rate(10)

        # Create a publisher for the position target
        self.pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)

        # Create a subscriber for position data
        self.sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback)

        # Global variable
        self.pose = PoseStamped()
        if os.name != 'nt':
            self.settings = termios.tcgetattr(sys.stdin)
    
        # ROS service clients
        self.takeoff_service = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        self.land_service = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
        self.arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)    

        # Wait for the connection to FCU (Flight Controller Unit)
        while not rospy.is_shutdown() and not self.is_connected():
            rospy.loginfo("Waiting for FCU connection...")
            rospy.sleep(2)

        self.teleop()

    def pose_callback(self, msg):
        self.pose = PoseStamped()
        self.pose = msg

    def getKey(self):
        if os.name == 'nt':
            timeout = 0.1
            startTime = time.time()
            while(1):
                if msvcrt.kbhit():
                    if sys.version_info[0] >= 3:
                        return msvcrt.getch().decode()
                    else:
                        return msvcrt.getch()
                elif time.time() - startTime > timeout:
                    return ''

        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def is_connected(self):
        state_msg = rospy.wait_for_message('/mavros/state', State, timeout=60)
        rospy.loginfo("Connected with FCU")
        rospy.sleep(2)
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
                rospy.sleep(3)  # Wait for 4 seconds after arming motors
            else:
                rospy.logerr("Failed to arm motors")
        except rospy.ServiceException as e:
            rospy.logerr("Arming service call failed: " + str(e))
        
    def takeoff(self, alt):

        try:
            response = self.takeoff_service(altitude=alt, latitude=0.0, longitude=0.0, min_pitch=0.0, yaw=0.0)
            
            if response.success:
                rospy.loginfo("Takeoff successful")
                while self.pose.pose.position.z < (alt - 0.1):
                    pass
                rospy.loginfo("Target altitude reached")

        except rospy.ServiceException as e:
            rospy.logerr("Takeoff service call failed: " + str(e))

    def land(self):
        try:
            response = self.land_service(altitude=0.0, latitude=0.0, longitude=0.0, min_pitch=0.0, yaw=0.0)
            if response.success:
                while self.pose.pose.position.z > 0.1:
                    pass
                rospy.loginfo("Landed")
                rospy.sleep(1)
                rospy.loginfo("Disarming")
            else:
                rospy.logerr("Failed to send landing command")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: " + str(e))

    
    def teleop(self):
        status = 0

        msg = """
        Control Your Drone!
        ---------------------------
        Enter altitude for Takeoff in meters

          Roll / Pitch:    Up / Down:    Yaw:
                w
           a    s    d       u / m       i / o
                x

        w/x : pitch down / pitch up
        a/d : roll left / roll right
        i/o : yaw left / yaw right
        u/m : up / down
        l : land

        space key, s : engage RTL

        CTRL-C to quit
        """

        e = """
        Communications Failed
        """

        if self.set_mode("GUIDED"):
            rospy.loginfo("Mode changed to GUIDED")
            rospy.sleep(1)  # Wait for 2 seconds after changing mode
        
        takeoff_alt = float(input("Enter takeoff altitude: "))
        rospy.loginfo("Taking off to %.1f meters", takeoff_alt)
        self.arm_motors()
        self.takeoff(takeoff_alt)
    
        try:
            print(msg)

            vel = PositionTarget()
            # Set the coordinate frame value to 8 (LOCAL_NED)
            vel.coordinate_frame = 8

            # Set the bitmask to 1987 to control position, velocity, acceleration, yaw, and yaw_rate
            vel.type_mask = 1987
            while not rospy.is_shutdown():
                key = self.getKey()
                if key == 'w' :
                    vel.velocity.x = 1
                    status = status + 1
                
                elif key == 'x' :
                    vel.velocity.x = -1
                    status = status + 1
                
                elif key == 'a' :
                    vel.velocity.y = 0.4
                    status = status + 1
                
                elif key == 'd' :
                    vel.velocity.y = -0.4
                    status = status + 1

                elif key == 'u':
                    vel.velocity.z = 0.15
                    status = status + 1

                elif key == 'm':
                    vel.velocity.z = -0.15
                    status = status + 1

                elif key == 'i':
                    vel.yaw_rate = 0.2
                    status = status + 1

                elif key == 'o':
                    vel.yaw_rate = -0.2
                    status = status + 1
                
                elif key == 'l':
                    vel.velocity.x = 0.0
                    vel.velocity.y = 0.0
                    vel.velocity.z = 0.0
                    vel.yaw_rate = 0.0
                    self.pub.publish(vel)
                
                    rospy.loginfo("Landing...")
                    self.land()

                    # Break if landed
                    break
                
                elif key == ' ' or key == 's' :
                    # Change flight mode to RTL
                    if self.set_mode("RTL"):
                        rospy.loginfo("RTL engaged")
                    while self.pose.pose.position.z > 0.1:
                        pass
                    rospy.loginfo("Returned to launch")
                    rospy.loginfo("Disarming")
                    rospy.sleep(1)

                    # Break if RTL engaged
                    break
            
                # Break if CTRL+C pressed                
                elif (key == '\x03'):                    
                    rospy.loginfo("CTRL + C pressed >> Killing node")
                    rospy.sleep(2)
                    break

                else:
                    vel.velocity.x = 0.0
                    vel.velocity.y = 0.0
                    vel.velocity.z = 0.0
                    vel.yaw_rate = 0.0

                if status == 20 :
                    print(msg)
                    status = 0

                self.pub.publish(vel)
            
            rospy.signal_shutdown("done")

        except:
            print(e)

        finally:
            vel = PositionTarget()
            vel.velocity.x = vel.yaw_rate = 0.0
            self.pub.publish(vel)

        # rospy.signal_shutdown("over")

        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


if __name__ == '__main__':
    try:
        node = teleoperation()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

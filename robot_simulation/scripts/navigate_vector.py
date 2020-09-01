#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import sys
from tf.transformations import euler_from_quaternion


class Navigate_to_Goal(object):
    def __init__(self):
        # Declare publisher and subscriber nodes
        rospy.init_node('navigate_to_goal',anonymous=False)
        self.pub = rospy.Publisher('cmd_vel',Twist,queue_size=10)
        rospy.Subscriber('ground_truth/state',Odometry,self.odom_callback)
        rospy.Subscriber('scan',LaserScan,self.laser_callback)

        # Rate of publishing data
        self.rate = rospy.Rate(10)

        # Robot parameters
        self.x_robot = 0.0
        self.y_robot = 0.0
        self.x_goal = 10.0
        self.y_goal = 10.0
        self.orientation_q = None
        self.robot_yaw = 0.0

        # Distance from goal of the robot
        self.distance_to_goal = self.prev_distance_to_goal = 0.0
        self.desired_angle_goal = 0.0
            
        # Maximum range of laser sensors    
        self.max_sensor_range = 1
        
        # List of all the sensors
        self.sensor_fleft=self.sensor_left=self.sensor_bleft= 0.0
        self.sensor_fright=self.sensor_right=self.sensor_bright=0.0
        self.front_beam_left=0.0
        self.front_beam_right=0.0

    def laser_callback(self,msg):
        
        # Sensors on the left
        self.sensor_fleft = min( min(msg.ranges[1:60])  , self.max_sensor_range)
        self.sensor_left = min( min(msg.ranges[61:90])   , self.max_sensor_range)
        self.sensor_bleft = min( min(msg.ranges[91:110]) , self.max_sensor_range)

        # Sensors on the right
        self.sensor_fright = min( min(msg.ranges[-60:-1])  , self.max_sensor_range)
        self.sensor_right = min( min(msg.ranges[-90:-61])   , self.max_sensor_range)
        self.sensor_bright = min( min(msg.ranges[-110:-91]) , self.max_sensor_range)

        # Sensors on the front of the robot - left and right (to detect head-on collision)
        self.front_beam_left =  min(min(msg.ranges[0:5]) , self.max_sensor_range)
        self.front_beam_right =  min(min(msg.ranges[0:-5]) , self.max_sensor_range)

    def odom_callback(self,msg):
        # Odometry data 
        self.x_robot = msg.pose.pose.position.x
        self.y_robot = msg.pose.pose.position.y
        self.orientation_q = msg.pose.pose.orientation 
        
        # Converting quaternion (xyzw) to euler (rpy)
        orientation_list = [self.orientation_q.x, self.orientation_q.y, self.orientation_q.z, self.orientation_q.w]
        (self.robot_roll, self.robot_pitch, self.robot_yaw) = euler_from_quaternion (orientation_list)
    
    def follow_wall_left(self):
        linear_speed = 0.2
        angular_speed = 0.0
        # ------------------------LEFT--------------------------------------
        if self.front_beam_left < 0.6 and (self.sensor_fleft < 0.5 or self.sensor_left < 0.5) :  # Obstacle detected ahead of the robot while following wall on left
                print("Danger ahead")
                linear_speed = 0.0
                angular_speed = -1.2 * min(self.front_beam_left,self.front_beam_right) 
                if self.front_beam_left == self.max_sensor_range:
                    angular_speed = 0.0
    
        elif self.sensor_left != self.max_sensor_range:                                      
            if self.sensor_left > 0.65 and self.sensor_left < self.max_sensor_range:        # Robot too far from wall while following wall, then approach wall
                print("Approaching wall ...")
                angular_speed =  0.5 * min(self.sensor_left,self.sensor_fleft) 
                
            elif min(self.sensor_left,self.sensor_fleft) < 0.6:                               # Robot too close from wall while following wall, then go away from wall
                print("Avoiding wall ...")
                angular_speed = -0.8 * min(self.sensor_left,self.sensor_fleft) 
                
            elif min(self.sensor_left,self.sensor_fleft) > 0.6 and min(self.sensor_left,self.sensor_fleft) < 0.65: # Robot between a distance from a wall, then go straight
                print("Straight")
                angular_speed = 0.0
        
        elif self.sensor_left == self.max_sensor_range and self.sensor_bleft != self.max_sensor_range:     # Corner detected, stop going and turn to stick to wall
                print("Left Corner Detected")      
                linear_speed = 0.0
                angular_speed = 0.25    
        
        return linear_speed, angular_speed     

    def follow_wall_right(self):
        linear_speed = 0.2
        angular_speed = 0.0
        # ----------------------------RIGHT------------------------------------------
        if self.front_beam_right < 0.6 and (self.sensor_fright < 0.5 or self.sensor_right < 0.5) :  # Obstacle detected ahead of the robot while following wall on right
                print("Danger ahead")
                linear_speed = 0.0
                angular_speed = 1.2 * min(self.front_beam_left,self.front_beam_right)
                if self.front_beam_right == self.max_sensor_range:
                    angular_speed = 0.0
        
        if self.sensor_right != self.max_sensor_range:                                            
            if self.sensor_right > 0.65 and self.sensor_right < self.max_sensor_range:         # Robot too far from wall while following wall, then approach wall
                angular_speed = -0.3 * min(self.sensor_right,self.sensor_fright) 
            
            elif min(self.sensor_right,self.sensor_fright) < 0.6:                          # Robot too close from wall while following wall, then go away from wall
                angular_speed = 0.3 * min(self.sensor_right,self.sensor_fright) 

            elif min(self.sensor_right,self.sensor_fright) > 0.6 and min(self.sensor_right,self.sensor_fright) < 0.65:      # Robot between a distance, then go straight
                angular_speed = 0.0 
        
        elif self.sensor_right == self.max_sensor_range and self.sensor_bright != self.max_sensor_range:        # Corner detected, stop going and turn to stick to wall
                print("Right Corner Detected")
                linear_speed = 0.0
                angular_speed = -0.25    
        
        return linear_speed,angular_speed

    def avoid_obstacles(self):
        threshold_dist = 0.4
        angular_speed = 0.0
        if self.sensor_left < threshold_dist or self.sensor_fleft < threshold_dist: # if obstacle on left, then go to right
            angular_speed = - 0.5 * (self.sensor_left + self.sensor_fleft)
            
        elif self.sensor_right < threshold_dist or self.sensor_fright < threshold_dist: # if obstacle on right, then go to left
            angular_speed = 0.5 * (self.sensor_right + self.sensor_fright)
            
        elif self.sensor_left < threshold_dist and self.sensor_right < threshold_dist: # if obstacles on both side, then add outputs of both - left and right sides
            angular_speed =  0.5 * (-self.sensor_left - self.sensor_fleft) + 0.5 * (self.sensor_right + self.sensor_fright)
        
        return angular_speed

    def run(self):

        # Initialize distance of robot from goal and its desired angle
        self.distance_to_goal = ( (self.x_goal - self.x_robot)**2 + (self.y_goal - self.y_robot)**2 )**0.5
        self.desired_angle_goal = math.atan2(self.y_goal-self.y_robot, self.x_goal-self.x_robot)
        self.prev_distance_to_goal = self.distance_to_goal
        
        linear_speed = 0.15
        angular_speed = 0.0
        vel = Twist()
        

        while not self.distance_to_goal < 0.1:   # As long as robot doesn't come in 0.2m of goal, keep executing the loop
            
            # Calculate the sum of sensors on left and right sides
            total_sensor_left =  (self.max_sensor_range - self.sensor_left) + (self.max_sensor_range - self.sensor_fleft) + (self.max_sensor_range - self.front_beam_left)
            total_sensor_right =  (self.max_sensor_range - self.sensor_right) + (self.max_sensor_range - self.sensor_fright) + (self.max_sensor_range - self.front_beam_right)
            
            # Calculate distance from goal and desired heading
            self.distance_to_goal = ((self.x_goal - self.x_robot)**2 + (self.y_goal - self.y_robot)**2)**0.5
            self.desired_angle_goal = math.atan2(self.y_goal-self.y_robot, self.x_goal-self.x_robot)
            

            # When no obstacles are present in periphery of robot, then go to goal with required heading 
            if ((self.sensor_left==self.max_sensor_range and self.sensor_fleft == self.max_sensor_range ) and (self.sensor_right == self.max_sensor_range and self.sensor_fright == self.max_sensor_range ) ) and (self.front_beam_left == self.max_sensor_range or self.front_beam_right == self.max_sensor_range):
                linear_speed = 0.15
                angular_speed = min(1.5 * (self.desired_angle_goal - self.robot_yaw),0.25)
                print("Going to Goal")


            # When Obstacle is detected --->
            else:
                # Check if obstacle is present on left and right sides both
                if (self.sensor_left!=self.max_sensor_range or self.sensor_fleft!=self.max_sensor_range) and (self.sensor_right!=self.max_sensor_range or self.sensor_fright!=self.max_sensor_range):
                    print("Obstacle detected on Left and Right sides")
                    sum_sensor_left = self.sensor_left + self.sensor_fleft + self.front_beam_left
                    sum_sensor_right = self.sensor_right + self.sensor_fright + self.front_beam_right
                    
                    # If robot closer to left side, then continue following the left wall
                    if sum_sensor_left < sum_sensor_right:            
                        linear_speed,angular_speed =  self.follow_wall_left()
                        print("Sticking to Left wall")
                    
                    # If robot closer to left side, then continue following the right wall
                    elif sum_sensor_right < sum_sensor_left:
                        linear_speed,angular_speed =  self.follow_wall_right()
                        print("Sticking to Right wall")

                # Check if obstacle is only on left side
                elif self.sensor_left!=self.max_sensor_range or self.sensor_fleft!=self.max_sensor_range or self.front_beam_left!=self.max_sensor_range:
                    # Follow left wall by default
                    linear_speed,angular_speed =  self.follow_wall_left()
                    print("Obstacle detected on Left Side")
                    
                    # And keep following it until distance to goal decreases and the robot is facing the correct direction, then go to goal
                    if self.distance_to_goal < self.prev_distance_to_goal and abs(self.robot_yaw - self.desired_angle_goal) < math.pi/4:
                        print("Avoiding Obstacle:Left &  Going To Goal")
                        angular_speed = (-total_sensor_left + total_sensor_right) + ( self.desired_angle_goal - self.robot_yaw )  

                # Check if obstacle is only on right side
                elif self.sensor_right!=self.max_sensor_range or self.sensor_fright!=self.max_sensor_range or self.front_beam_right!=self.max_sensor_range:
                    # Follow right wall by default
                    linear_speed,angular_speed =  self.follow_wall_right()
                    print("Obstacle detected on Right Side")
                    
                    # And keep following it until distance to goal decreases and the robot is facing the correct direction, then go to goal
                    if self.distance_to_goal < self.prev_distance_to_goal and abs(self.robot_yaw - self.desired_angle_goal) < math.pi/4:
                        print("Avoiding Obstacle:Right &  Going To Goal")
                        angular_speed = (-total_sensor_left + total_sensor_right) + ( self.desired_angle_goal - self.robot_yaw )  

            # Measure the closest distance the robot has ever been to the goal position
            if self.distance_to_goal < self.prev_distance_to_goal:
                self.prev_distance_to_goal = self.distance_to_goal

            # Publish the linear and angular velocities
            vel.linear.x = linear_speed
            vel.angular.z = angular_speed
            self.pub.publish(vel)

            self.rate.sleep()
        
        # Once, goal is reached, stop the robot
        print("Goal Reached: ",self.x_robot,self.y_robot)
        vel.linear.x = 0.0
        vel.angular.z = 0.0
        self.pub.publish(vel)


if __name__ == "__main__":
    try:
        a = Navigate_to_Goal()
        a.run()

    except rospy.ROSInterruptException:
        rospy.loginfo("ALERT: Program Shutdown")

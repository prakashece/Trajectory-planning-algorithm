#!/usr/bin/env python

import rospy
import tf
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import pi, sqrt, atan2
import math
import csv
class PID:
    """
    Discrete PID control
    """
    def __init__(self, P=0.0, I=0.0, D=0.0, Derivator=0, Integrator=0, Integrator_max=10, Integrator_min=-10):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.Derivator = Derivator
        self.Integrator = Integrator
        self.Integrator_max = Integrator_max
        self.Integrator_min = Integrator_min
        self.set_point = 0.0
        self.error = 0.0
        self.Derivator = 0
        self.Integrator = 0

        self.p_val,self.i_val,self.d_val=[],[],[]
    def update(self, current_value):
        self.error = self.set_point - current_value
        if self.error > pi:  # specific design for circular situation
            self.error = self.error - 2*pi
        elif self.error < -pi:
            self.error = self.error + 2*pi
        self.P_value = self.Kp * self.error
        self.D_value = self.Kd * ( self.error - self.Derivator)
        self.Derivator = self.error
        self.Integrator = self.Integrator + self.error
        if self.Integrator > self.Integrator_max:
            self.Integrator = self.Integrator_max
        elif self.Integrator < self.Integrator_min:
            self.Integrator = self.Integrator_min
        self.I_value = self.Integrator * self.Ki
        PID = self.P_value + self.I_value + self.D_value
        #appending the PID values
        self.p_val.append(self.P_value)
        self.i_val.append(self.I_value)
        self.d_val.append(self.D_value)
        
        return PID

    def setPoint(self, set_point):
        self.set_point = set_point
        
        
    def setPID(self, set_P=0.0, set_I=0.0, set_D=0.0):
        self.Kp = set_P
        self.Ki = set_I
        self.Kd = set_D

class turtlebot_move():
    def __init__(self,WAYPOINTS):
        rospy.init_node('move_bot', anonymous=False)
        rospy.loginfo("Press CTRL + C to terminate")
        rospy.on_shutdown(self.stop)
        self.angle_vel=[]
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.pid_theta = PID(0,0,0)  # initialization
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)
        self.vel_pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
        self.vel = Twist()
        self.rate = rospy.Rate(1000)
        self.counter = 0
        self.trajectory = [[]]
        #setup to plot the figure
        fig = plt.figure()
        ax1=fig.add_subplot(1,2,1)
        ax2=fig.add_subplot(2,2,2)
        ax3=fig.add_subplot(2,2,4)
        ax1.set_title("Traced Trajectory")
        ax2.set_title("Linear Velocity")
        ax3.set_title("Angular Velocity")
        ax1.set_xlim([0,4.2])
        ax1.set_ylim([0,2.2])
        ax2.set_ylim([0,0.2])
        #ax3.set_ylim([-0.5,0.5])
        ax3.set_ylim([-2,2])
        fig.show()
        x_val,y_val=[],[]
	des = [4,2]
	way = [2,0.66]
	obs = [3,1.22]
        self.x_vel,self.z_vel,self.yaw_out,self.theta_out=[],[],[],[]
        self.p_count=0
        # track a sequence of waypoints
        for point in WAYPOINTS:
        #loop starts here to move the robot
            self.p_count+=1
            rospy.logwarn("in loop")
            self.move_to_point(point[0], point[1]) 
            #updating the figure
            x_val.append(self.x)
            y_val.append(self.y)
            ax1.plot(x_val,y_val,color='b')
            ax1.plot(des[0],des[1],"D",color='b')
	    #ax1.plot(obs[0],obs[1],"*",color='g')
	    #ax1.plot(way[0],way[1],"s",color='r')
	    ax1.set_aspect('equal')
            #ax1.legend(["trajectory","destination"])
            ax2.plot(self.x_vel,color='g')
            ax3.plot(self.z_vel,color='r')
            #ax2.plot(self.p_val,color='g')
            #ax2.plot(self.pid_theta.i_val,color='r')
            #ax3.plot(self.pid_theta.d_val,color='b')
            fig.canvas.draw()
            fig.canvas.flush_events()
        rospy.logwarn("end of loop.")
        self.stop()
	#plt.savefig("case_2_1_ang_PID")
        plt.pause(25)
        rospy.sleep(1)
        rospy.logwarn("Action done.")
	details=["X","Y","YAW","Theta","Linear_v","Angular_v"]
        # plot trajectory
	data_w= np.array([x_val,y_val,self.yaw_out,self.theta_out,self.x_vel,self.z_vel])
        data = np.array(self.trajectory)
	with open("case2_1_pid.csv",'w') as f:
		write=csv.writer(f)
		write.writerow(details)
		write.writerows(data_w)
        #print("plotting")
        #plt.plot(data[:,0],data[:,1])
        #print("plotted")


    def move_to_point(self, x, y):
        # Compute orientation for angular vel and direction vector for linear vel
        diff_x = x - self.x
        diff_y = y - self.y
        direction_vector = np.array([diff_x, diff_y])
        direction_vector = direction_vector/sqrt(diff_x*diff_x + diff_y*diff_y)  # normalization
        theta = atan2(diff_y, diff_x)
        self.pid_theta.setPID(2, 0.02, 0.2) 
        self.pid_theta.setPoint(theta)
        self.theta_out.append(theta)

        # Adjust orientation first

        while not rospy.is_shutdown():
            rospy.loginfo("move to the point")
            diff_x = x - self.x
            diff_y = y - self.y
            vector = np.array([diff_x, diff_y])
            linear = np.dot(vector, direction_vector)
            #linear= np.dot(direction_vector[0],direction_vector[1])
            angular = self.pid_theta.update(self.theta)
	    self.yaw_out.append(self.theta)
            # projection
            if abs(linear) > 0.1:
                linear = linear/abs(linear)*0.1
         
            #if abs(angular) > 0.3:
                #angular = angular/abs(angular)*0.3

            if abs(linear) < 0.1 and abs(angular) < 0.2:
                break
            rospy.loginfo(linear)
            rospy.loginfo(angular)
            self.x_vel.append(linear)
            self.z_vel.append(angular)
            self.vel.linear.x = linear
            self.vel.angular.z = angular
            self.vel_pub.publish(self.vel)
            self.rate.sleep()

    def stop(self):
        self.vel.linear.x = 0
        self.vel.angular.z = 0
        self.vel_pub.publish(self.vel)
        rospy.sleep(1)


    def odom_callback(self, msg):
        # Get (x, y, theta) specification from odometry topic
        quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,\
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
        self.theta = yaw
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Make messages saved and prompted in 5Hz rather than 100Hz
        self.counter += 1
        if self.counter == 20:
            self.counter = 0
            self.trajectory.append([self.x,self.y])
            
            rospy.loginfo("odom: x=" + str(self.x) + ";  y=" + str(self.y) + ";  theta=" + str(self.theta))

def move_function(waypoints):
    try:
    #function to move the turtlebot
        turtlebot_move(waypoints)
    except rospy.ROSInterruptException:
        rospy.loginfo("Action terminated.")

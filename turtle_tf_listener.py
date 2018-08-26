#!/usr/bin/env python  
import roslib
roslib.load_manifest('learning_tf')
import rospy
import math
import tf
import geometry_msgs.msg
import turtlesim.srv


if __name__ == '__main__':
    rospy.init_node('turtle_tf_listener')

    listener = tf.TransformListener()

    rospy.wait_for_service('spawn')
    spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    spawner(4, 2, 0, 'turtle2')

    turtle_vel = rospy.Publisher('turtle2/cmd_vel', geometry_msgs.msg.Twist,queue_size=1)

    rate = rospy.Rate(50.0)
    listener.waitForTransform("/turtle2", "/turtle1", rospy.Time(), rospy.Duration(4.0))
    while not rospy.is_shutdown():
        try:
            now = rospy.Time.now()
            past = now - rospy.Duration(0.1)
            listener.waitForTransform("/turtle2", "/turtle1", now, rospy.Duration(1.0))
            (trans,rot) = listener.lookupTransform('/turtle2', '/turtle1', now)
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        (r, p, real_y) = tf.transformations.euler_from_quaternion([rot[0],rot[1],rot[2],rot[3]])
       
        # These values are in radians
        rospy.loginfo("The values from RPY are r->%s p->%s y->%s :",r,p,real_y)
        rospy.loginfo("--------------------Reading value Loop---------------------------")
        # PID control for rotation and traversal 
        Kp=1.2
        
        distance =   math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        
        error=real_y

        if(abs(error)>0.05):
            error = Kp*error

            rospy.loginfo("Roll: %s Pitch: %s Real_Yaw: %s  Error: %s", r,p, real_y, error )
            rospy.loginfo("--------------------In PID Loop!---------------------------")
            
            cmd = geometry_msgs.msg.Twist()
            cmd.linear.x = 0
            cmd.angular.z = error*4
            turtle_vel.publish(cmd)
            rate.sleep()

        #PID loop for linear control 
        
        distance =   math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        x_coord=trans[0]
        y_coord=trans[1]
        Kl=0.999
        real_lin= 3.866  #Original distance value goes here
        error_lin= real_lin-distance
        if(abs(error_lin)>0.05  ): 
            error_lin= Kl*error_lin
            rospy.loginfo("Error_lin: %s-------------------Inside the Linear velocity loop-----------",error_lin)
            cmd = geometry_msgs.msg.Twist()
            if (abs(distance)>3.9):
                cmd.linear.x = -error_lin*2
            elif (abs(distance)<3.7) : 
                cmd.linear.x = error_lin*2
            cmd.angular.z = 0
            turtle_vel.publish(cmd)
            rate.sleep()
        
        






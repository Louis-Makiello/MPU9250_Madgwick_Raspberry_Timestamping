#!/usr/bin/python

import roslib;
from visualization_msgs.msg import Marker
from std_msgs.msg import String
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from sensor_msgs.msg import Imu
from pyquaternion import Quaternion
import csv
import rospy

csvfile='/home/ubuntu/Desktop/Imudata.txt'

def Quaternion_NED_To_ENU(q):
        q_90 = Quaternion(axis=[0, 0, -1], degrees=-90)
        q_ENU = str(q_90*q)
        print(q_ENU)
        one=q_ENU.replace("i", "")
        two=one.replace("j", "")
        three=two.replace("k", "")
        four=three.replace("+","")
        quaterns=four.split(" ")
        #print(three)
        #print(quaterns)
        #q_ENU is w ix jy kz
        return quaterns


def callback(data):
        rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
        Number =data.data
        #print("Hello, would you like some cheese?")
        Quaternions=Number.split(",")
        quatx =float(Quaternions[1])
        quaty =float(Quaternions[2])
        quatz =float(Quaternions[3])
        quatw =float(Quaternions[0])
        q5 = Quaternion(quatw,quatx,quaty,quatz)
        quatlistENU=Quaternion_NED_To_ENU(q5)
        #print(quatlistENU)
        quatwE=float(quatlistENU[0])
        quatxE=float(quatlistENU[1])
        quatyE=float(quatlistENU[2])
        quatzE=float(quatlistENU[3])
        accelx=float(Quaternions[4]) #not actually a quaternion but whtvr
        accely=float(Quaternions[5]) #not actually a quaternion but whtvr
        accelz=float(Quaternions[6]) #not actually a quaternion but whtvr
        TimePPSsecs=float(Quaternions[10]) #not actually a quaternion but whtvr
        TimePPSnsecs=float(Quaternions[11]) #not actually a quaternion but whtvr
        EndLetter=(Quaternions[12]) #not actually a quaternion but whtvr
        #angvelx=float(Quaternions[7]) #not actually a quaternion but whtvr
        #angvely=float(Quaternions[8]) #not actually a quaternion but whtvr
        #angvelz=float(Quaternions[9]) #not actually a quaternion but whtvr
        #Imu_vizualisation = Marker()
        #Imu_vizualisation.header.frame_id = "imu_frame"
        #Imu_vizualisation.header.stamp = rospy.Time.now()
        #Imu_vizualisation.type = Marker.ARROW
        #Imu_vizualisation.pose.position.x = 0
        ##Imu_vizualisation.pose.position.y = 0
        #Imu_vizualisation.pose.position.z = 0
        #Imu_vizualisation.pose.orientation.x = quatx;
        #Imu_vizualisation.pose.orientation.y = quaty;
        #Imu_vizualisation.pose.orientation.z = quatz;
        #Imu_vizualisation.pose.orientation.w = quatw;
        #Imu_vizualisation.scale.x = 1
        #Imu_vizualisation.scale.y = 0.2
        #Imu_vizualisation.scale.z = 0.2
        #Imu_vizualisation.color.a = 1.0
        #Imu_vizualisation.color.r = 0.6
        #Imu_vizualisation.color.g = 0.0
        #Imu_vizualisation.color.b = 0.0
        imu_msg = Imu()
        imu_msg.header.frame_id="imu_frame"
        imu_msg.header.stamp.secs = TimePPSsecs#rospy.Time.now()
        imu_msg.header.stamp.nsecs = TimePPSnsecs
        imu_msg.orientation.x=quatxE
        imu_msg.orientation.y=quatyE
        imu_msg.orientation.z=quatzE
        imu_msg.orientation.w=quatwE
        imu_msg.orientation_covariance[0] = 0.0174 #roll
        imu_msg.orientation_covariance[4] = 0.0174#pitch
        imu_msg.orientation_covariance[8]= 0.06981 #yaw
      #  imu_msg.angular_velocity.x = angvelx
      #  imu_msg.angular_velocity.y = angvely
       # imu_msg.angular_velocity.z = angvelz
        #imu_msg.angular_velocity_covariance[0] = -1
        imu_msg.linear_acceleration.x = accelx/1000
        imu_msg.linear_acceleration.y = accely/1000
        imu_msg.linear_acceleration.z = accelz/1000
        imu_msg.linear_acceleration_covariance[0] = 3#0.00616664678 #-1 put 0.00616664678 as per datasheet
    	imu_msg.linear_acceleration_covariance[4]=3 #0.00616664678 # numbers here are experiment
 	imu_msg.linear_acceleration_covariance[8]=3#0.00616664678
        #publisher.publish(Imu_vizualisation)
        publisher2.publish(imu_msg)
        Time_for_txt_file=Quaternions[10]+"."+Quaternions[11]
        theIMUdata=[Time_for_txt_file,EndLetter,quatx,quaty,quatz,quatw]
        with open( csvfile, "a") as output:
                 writer=csv.writer(output, delimiter=',',lineterminator='\n')
                 writer.writerow(theIMUdata)


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    #rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('Imu_data', String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    #rospy.spin()



rospy.init_node('mpu9250_parser', anonymous=False)
#publisher = rospy.Publisher("Imu_vizualisation", Marker,queue_size=10)
publisher2 = rospy.Publisher("imu/data/", Imu,queue_size=10)  #topic is imu/data 

count = 0
listener()
rospy.spin()
#quatx=float(1)
#while not rospy.is_shutdown():
 #   listener()
 #   rospy.sleep(0.25)


    # Publish the MarkerArray
    

   # if __name__ == '__main__':
    #    listener()    

    #rospy.sleep(0.01)




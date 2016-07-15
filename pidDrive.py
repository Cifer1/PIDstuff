#!/usr/bin/env python
import rospy, math
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from PID import PIDController
import driveTrig

ANGLE_FROM_ZERO = 45
MIDDLE_INDEX = ANGLE_FROM_ZERO * int(1081/270)
BOTTOM_INDEX = MIDDLE_INDEX - 5
TOP_INDEX = MIDDLE_INDEX + 6

class pidDriver:
	    def __init__(self, d_des, speed):
		self.error = 0 # initializes the running error counter
		self.d_des = d_des # takes desired dist from wall
		self.speed = speed
		self.scan1 = [0]
		self.scan2 = [0]
		rospy.init_node("wall_PID",  anonymous=False)
		rate = rospy.Rate(10)
		pid = PIDController(rospy.Time.now().to_sec(), 0.15,0.00000001,0.12)
		#init the pub/subs
		self.drive = rospy.Publisher("/racecar/ackermann_cmd_mux/input/teleop",  AckermannDriveStamped,  queue_size=5)
		rospy.Subscriber("/racecar/laser/scan",  LaserScan,  self.callback)
		
		drive_msg = AckermannDriveStamped()
		drive_msg.drive.steering_angle = 0.0
		drive_msg.drive.speed = self.speed
		
		dist_trav = 50.0
		total = 0.0
		
		time = dist_trav/self.speed
		ticks = int(time * 10)
		for t in range(ticks):
		    self.error = 0
		    total1 = 0
	            total2 = 0
		    meanD_x = 0
		    meanD_y = 0
		    for i in self.scan1:
			print str(i)		        
			total1 += i
		    for i in self.scan2:
			total2 += i
		    meanD_x = total1/len(self.scan1)
		    meanD_y = total2/len(self.scan2)

		    #print str(self.scan1)
		    #print str(self.scan2)
		    if meanD_x != 0 and meanD_y != 0:
			print ("started trig")
			d_actual = driveTrig.findDist(meanD_x, meanD_y, 10)	
		    else: 
			self.drive.publish(AckermannDriveStamped())
			continue
	            self.error = self.d_des - d_actual 
		    print str(self.error)
		    pidVal = pid.update(self.error,  rospy.Time.now().to_sec())
		    pidVal = pidVal/abs(pidVal) * min(1.0,  abs(pidVal)) if pidVal!=0 else 0
		    print pidVal
		    drive_msg.drive.steering_angle = pidVal
		    self.drive.publish(drive_msg)
		    print "published"
		    rate.sleep()
		self.drive.publish(AckermannDriveStamped())
	    def callback(self,msg):
		self.scan1 = []
		self.scan2 = []	
		for i in range(175, 186):
			self.scan1.append(msg.ranges[i])
		for i in range(215, 226):
			self.scan2.append(msg.ranges[i])
	  	    
	    def shutdown(self):
		rospy.loginfo("Stopping robot")
		self.drive.publish(AckermannDriveStamped())
		rospy.sleep(1)

if __name__ == "__main__":
    #try:
	pidDriver(1.5,1.7)
    	rospy.spin()
   ## except:
	#rospy.loginfo("PID Driver terminated")

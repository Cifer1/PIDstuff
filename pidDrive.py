#!/usr/bin/env python
import rospy, math
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from PID import PIDController
import driveTrig

"""ANGLE_FROM_ZERO = 45
MIDDLE_INDEX = ANGLE_FROM_ZERO * int(1081/270)
BOTTOM_INDEX = MIDDLE_INDEX - 5
TOP_INDEX = MIDDLE_INDEX + 6 """

class pidDriver:
    def __init__(self, d_des, speed, dist):
		self.drive_cmd = AckermannDriveStamped()
		self.drive_cmd.drive.speed = speed		
		self.error = 0 # initializes the running error counter
		self.d_des = d_des # takes desired dist from wall
		self.dist = dist
		self.scan1 = []
		self.scan2 = []
		self.sideFlag = 0
			
		rospy.init_node("wall_PID",  anonymous=False)
		
		self.pid = PIDController(rospy.Time.now().to_sec(), 0.1,0.0, 0.07)#0.00000001,0.12)

		#init the pub/subs
		self.drive_pub = rospy.Publisher("/vesc/ackermann_cmd_mux/input/teleop",  AckermannDriveStamped,  queue_size=5)
		rospy.Subscriber("left_or_right", Int32, self.leftRight)
		rospy.Subscriber("/scan",  LaserScan,  self.callback)
		
		# START PUB FUNCTION
	    	self.publisher()
    def leftRight(self, msg):
	self.sideFlag = msg
	print str(self.sideFlag)
    def callback(self,msg):
		self.scan1 = []
		self.scan2 = []	
		if self.sideFlag == 1 or self.sideFlag == 0:			
			for i in range(175, 186):
				self.scan1.append(msg.ranges[i]) # gets 90 degree scans and adds them to scan1
			for i in range(215, 226):
				self.scan2.append(msg.ranges[i]) # gets 100 degree scans and adds them to scan2
		if self.sideFlag == 2:
			for i in range(895, 906):
				self.scan1.append(msg.ranges[i]) # gets 90 degree scans and adds them to scan1
			for i in range(855, 866):
				self.scan2.append(msg.ranges[i]) # gets 100 degree scans and adds them to scan2
		self.main(self.scan1, self.scan2)
    

    def main(self, scan1, scan2):
		    self.error = 0
		    total1 = 0 # total distance for averaging (from scan1)
		    total2 = 0 # total distance for averaging (from scan2)

		    meanD_x = 0 # average distance taken from scan1
		    meanD_y = 0 # average dist taken from scan2

		    for i in scan1: 
			total1 += i # adds each element of scan1 to total for averaging
		    for i in scan2:
			total2 += i # same as above but for scan2
		    meanD_x = total1/len(scan1) # average of scan1
		    meanD_y = total2/len(scan2) # average of scan2

		    if meanD_x != 0 and meanD_y != 0: # checks if vals are good
			#print ("started trig")
			d_actual = driveTrig.findDist(meanD_x, meanD_y, 10)

		    else:
			return
	            print "d_actual: " + str(d_actual)
		    self.error = self.d_des - d_actual 
		    print "error: " + str(self.error)

		    pidVal = self.pid.update(self.error,  rospy.Time.now().to_sec())
		    pidVal = pidVal/abs(pidVal) * min(1.0,  abs(pidVal)) if pidVal!=0 else 0
		   # print pidVal
		    self.drive_cmd.drive.steering_angle = pidVal

    def publisher(self):
	while not rospy.is_shutdown():
		self.drive_pub.publish(self.drive_cmd)
		rospy.Rate(10).sleep()

if __name__ == "__main__":
	pidDriver(0.61,2.0,20)
rospy.spin()

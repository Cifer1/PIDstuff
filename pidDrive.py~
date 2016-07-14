import rospy,  math
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from PID import PIDController

class pidDriver:
    def __init__(self, d_des, speed):
        self.error = 0 # initializes the running error counter
        self.d_des = d_des # takes desired dist from wall
        self.speed = speed
        self.scan = []
        rospy.init_node("wall_PID",  anonymous=False)
        rate = rospy.Rate(10)
        pid = PIDController(0.5)
        #init the pub/subs
        self.drive = rospy.Publisher("/vesc/ackermann_cmd_mux/input/teleop",  AckermannDriveStamped,  queue_size=5)
        rospy.Subscriber("scan",  LaserScan,  self.callback)
        
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = 0.0
        drive_msg.drive.speed = self.speed
        
        dist_trav = 5.0 
        total = 0.0
        
        time = dist_trav/self.speed
        ticks = int(time * 10)
        for t in range(ticks):
            total = 0
            meanD = 0
            for i in self.scan:
                total += i
            meanD = total/len(self.scan)
            self.error = self.d_des - meanD
            pidVal = pid.update(self.error,  ros.Time.now())
            pidVal = pidVal/abs(pidVal) * max(1.0,  abs(pidVal)) if pidVal!=0 else 0
            drive_msg.drive.steering_angle = pidVal
            self.drive.publish(drive_msg)
            print "published"
            rate.sleep()
        self.drive.publish(AckermannDriveStamped())
    def callback(self,msg):
        self.scan = []	
        for i in range(355, 366):
        	self.scan.append(msg.ranges[i])
    
    
if __name__ == "__main__":
    wallF(0.4,0.2,0.5,1.5)
    rospy.spin()

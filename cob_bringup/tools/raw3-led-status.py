#!/usr/bin/python
import roslib
roslib.load_manifest('cob_script_server')
import rospy
import math
import tf
from geometry_msgs.msg import Twist
from std_msgs.msg import ColorRGBA
from diagnostic_msgs.msg import DiagnosticArray



class status_node():
    def __init__(self):
        rospy.Subscriber("/base_controller/command_direct", Twist, self.new_velcommand)
        rospy.Subscriber("/diagnostics", DiagnosticArray, self.new_diagnostics)
        self.led_pub = rospy.Publisher('/light_controller/command', ColorRGBA)
        self.last_vel = rospy.get_rostime()
        self.on = False
        self.diag_err = False

    def new_velcommand(self, twist):
        if twist.linear.x != 0 or twist.linear.y != 0 or twist.angular.z != 0:
            self.last_vel = rospy.get_rostime()
    
    def new_diagnostics(self, diag):
        for status in diag.status:
            if(status.name == "//base_controller"):
                if(status.level != 0):## && self.last_base_diag == 0):
                    self.diag_err = True
                elif(status.level == 0):## && self.last_base_diag == 1):
                    self.diag_err = False
                #self.last_base_diag = status_level


    def trigger_led(self):
        if(self.diag_err):
            self.led_pub.publish(ColorRGBA(1,0,0,0.4))
        else:
            if ((rospy.get_rostime() - self.last_vel).to_sec() > 1.0):
                #print (rospy.get_rostime() - self.last_vel).to_sec()
                self.led_pub.publish(ColorRGBA(0,1,0,0.4))
            else:
                if(self.on):
                    self.on = False
                    self.led_pub.publish(ColorRGBA(1,1,0,0.05))
                else:
                    self.on = True
                    self.led_pub.publish(ColorRGBA(1,1,0,0.4))

        


if __name__ == '__main__':
    rospy.init_node('cob_status_node')
    sn = status_node()
    rate = rospy.Rate(2.0)
    while not rospy.is_shutdown():
        sn.trigger_led()
        rate.sleep()

#!/usr/bin/python
# -*- coding:utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from bottle import get,post,run,route,request,template,static_file
from AlphaBot import AlphaBot
from PCA9685 import PCA9685
import threading
import socket #ip
import os

# ROS initialization
rospy.init_node('robot_controller', anonymous=True)
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
pan_pub = rospy.Publisher('/pan_angle', Float64, queue_size=10)
tilt_pub = rospy.Publisher('/tilt_angle', Float64, queue_size=10)

# Initialize the AlphaBot and PWM
Ab = AlphaBot()
pwm = PCA9685(0x40)
pwm.setPWMFreq(50)

#Set servo parameters
HPulse = 1500  #Sets the initial Pulse
HStep = 0      #Sets the initial step length
VPulse = 1500  #Sets the initial Pulse
VStep = 0      #Sets the initial step length
pwm.setServoPulse(1,VPulse)
pwm.setServoPulse(0,HPulse)

def publish_cmd_vel(linear_x, angular_z):
    twist = Twist()
    twist.linear.x = linear_x
    twist.angular.z = angular_z
    twist.header.stamp = rospy.Time.now()
    cmd_vel_pub.publish(twist)

def pulse_to_radians(pulse):
    # Convert pulse width (500-2500) to radians (-pi/2 to pi/2)
    return (pulse - 1500) / 1000.0 * math.pi / 2.0

def publish_pan_tilt(pan, tilt):
    pan_msg = Float64()
    pan_msg.data = pulse_to_radians(pan)
    pan_msg.header.stamp = rospy.Time.now()
    tilt_msg = Float64()
    tilt_msg.data = pulse_to_radians(tilt)
    tilt_msg.header.stamp = rospy.Time.now()
    pan_pub.publish(pan_msg)
    tilt_pub.publish(tilt_msg)

@get("/")
def index():
	return template("index")
	
@route('/<filename>')
def server_static(filename):
    return static_file(filename, root='./')

@route('/fonts/<filename>')
def server_fonts(filename):
    return static_file(filename, root='./fonts/')
	
@post("/cmd")
def cmd():
    global HStep,VStep
    code = request.body.read().decode()
    speed = request.POST.get('speed')
    print(code)
    if(speed != None):
        Ab.setPWMA(float(speed))
        Ab.setPWMB(float(speed))
        print(speed)
    if code == "stop":
        HStep = 0
        VStep = 0
        Ab.stop()
        publish_cmd_vel(0, 0)
        print("stop")
    elif code == "forward":
        Ab.forward()
        publish_cmd_vel(+speed / 100, 0)
        print("forward")
    elif code == "backward":
        Ab.backward()
        publish_cmd_vel(-speed / 100, 0)
        print("backward")
    elif code == "turnleft":
        Ab.left()
        publish_cmd_vel(0, +30 / 100 / 0.05)
        print("turnleft")
    elif code == "turnright":
        Ab.right()
        publish_cmd_vel(0, -30 / 100 / 0.05)
        print("turnright")
    elif code == "up":
        VStep = -5
        print("up")
    elif code == "down":
        VStep = 5
        print("down")
    elif code == "left":
        HStep = 5
        print("left")
    elif code == "right":
        HStep = -5
        print("right")
    return "OK"
	
def timerfunc():
	global HPulse,VPulse,HStep,VStep,pwm
	
	if(HStep != 0):
		HPulse += HStep
		if(HPulse >= 2500): 
			HPulse = 2500
		if(HPulse <= 500):
			HPulse = 500
		#set channel 2, the Horizontal servo
		pwm.setServoPulse(0, HPulse)    
		
	if(VStep != 0):
		VPulse += VStep
		if(VPulse >= 2500): 
			VPulse = 2500
		if(VPulse <= 500):
			VPulse = 500
		#set channel 3, the vertical servo
		pwm.setServoPulse(1, VPulse)   
	
	publish_pan_tilt(HPulse, VPulse)
	
	global t        #Notice: use global variable!
	t = threading.Timer(0.02, timerfunc)
	t.start()
    
def camera():
    lastpath = os.path.abspath(os.path.join(os.getcwd(), "../"))
    print("lastpath = %s" %lastpath)
    campath = lastpath + '/mjpg-streamer/mjpg-streamer-experimental/'
    print("campath = %s" %campath)
    os.system(campath  + './mjpg_streamer -i "' + campath + './input_uvc.so" -o "' + campath + './output_http.so -w ' + campath + './www"') 

tcamera = threading.Thread(target = camera)
tcamera.setDaemon(True)
tcamera.start()
tcamera = threading.Thread(target = camera)
tcamera.setDaemon(True)
tcamera.start()

t = threading.Timer(0.02, timerfunc)
t.setDaemon(True)
t.start()

s = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
s.connect(('8.8.8.8',80))
localhost = s.getsockname()[0]
run(host = localhost, port = 8000)

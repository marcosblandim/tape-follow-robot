from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import numpy as np
import sys
import RPi.GPIO as GPIO
import time
import threading
# own
import lib.pwm_engine_lib as pe
import lib.image_processing_lib as ip

def manage_dist(min_dist):
	global flag_dist
	global th_status
	while th_status:
		dist = pe.get_distance()
		if dist < min_dist:
			flag_dist = 1
			pe.move(0,0)
		else:
			flag_dist = 0
		# print("Distance sensor value =", int(dist))
		time.sleep(0.1) #0.1

# variables
cam_resolution = (160, 128) # (160,128), (320,240), (160,120), (640, 480)
vision_height = 0.5 # 0.5 # in percentage, top to bottom
MaxDist = 20 # 15
MinDist = -20 # -30
last_motor_command = "DEFAULT"
first_frame_flag = 0 # show first frame only
min_dist = 10
flag_dist = 0
th_status = 1
show_data_rate = 0.5
high_speed = 70 # 60
low_speed = 10 # 10 
max_speed = 90 # 80 # used in variable speed algorithm
min_speed = 0 # 0

# get setup flags
(distance_sensor_flag, hsv_range_flag,
simple_calibration_flag, show_frames_flag,
servo_flag, variable_speed_flag, 
search_mode_flag, exit_flag) = ip.config_panel()

# end program
if exit_flag:
	print("\n-> Program ended.\n")
	raise SystemExit()

# initialize camera
# and grab raw camera's capture reference
camera = PiCamera()
camera.resolution = cam_resolution
camera.framerate = 8 # 8
camera.rotation = 180 # upside down camera
rawCapture = PiRGBArray(camera, size=cam_resolution)
time.sleep(0.1) # warmup camera.

# set servos
if servo_flag:
	pe.set_servo(pe.Center + 1.4,pe.Center) # (+1.6,+0)
	print("-> Servos setted up.\n")

# set HSV range
if hsv_range_flag:
	if not simple_calibration_flag:
		lower_color, upper_color = ip.calibrate_pi_cam(camera, rawCapture)
	else:
		lower_color, upper_color = ip.simple_calibration(camera, rawCapture)
else:
	lower_color, upper_color = (np.array([0,52,39]),np.array([48,212,170]))

# initiate distance thread
if distance_sensor_flag:
	thread_dist = threading.Thread(target=manage_dist, args=(min_dist,))
	thread_dist.start()
	print("-> Distance thread ON.\n")

# iterate through camera's frames
it = iter(camera.capture_continuous(rawCapture, format="bgr", use_video_port=True))

# show first frame
frame = (next(it)).array
cv2.imshow('Frame Result',frame)
rawCapture.truncate(0)

# control printing rate
t0 = time.time()

# follow line initialization notification
print("-> Follow line ON.\n")

while(True):

	if flag_dist:
		continue

    # grab and process frame
	frame = (next(it)).array
	frame_result, frame_filtered = ip.hsv_filter(frame, lower_color, upper_color)

	# get distance used in motor's control
	distance = ip.distance(frame_filtered, vision_height)

	# control motors
	if ((distance == cam_resolution[0]) & (last_motor_command != "OFF")): # frame lenght when nothing is find
		last_motor_command = "OFF"
		pe.move(0,0) # 40 -40

		# search mode
		if search_mode_flag:
			# turn servo, process frame and decide where to turn
			pe.set_servo(pe.Center+1.4,pe.Center-3)
			rawCapture.truncate(0)
			frame_search = (next(it)).array
			_, frame_filtered_search = ip.hsv_filter(frame_search, lower_color, upper_color)
			distance_search = ip.distance(frame_filtered_search, vision_height)
			pe.set_servo(pe.Center + 1.4,pe.Center) # (+1.6,+0)
			
			if(distance_search != cam_resolution[0]):
				print('-> Search mode: RIGHT\n')
				pe.move(50,-50) # 50,-50
			
			else:
				pe.set_servo(pe.Center+1.4,pe.Center+3)
				rawCapture.truncate(0)
				frame_search = (next(it)).array
				_, frame_filtered_search = ip.hsv_filter(frame_search, lower_color, upper_color)
				distance_search = ip.distance(frame_filtered_search, vision_height)
				pe.set_servo(pe.Center+1.4,pe.Center) # (+1.6,+0)
				
				if(distance_search != cam_resolution[0]):
					print('-> Search mode: LEFT\n')
					pe.move(-50,50) # -50,50
				
				else:
					print('-> Search mode: OFF\n')
					pe.move(50,-50) # 50, -50			

	elif variable_speed_flag:
		pe.move(int(((max_speed+min_speed)/2) + ((max_speed-min_speed)*distance/cam_resolution[0])),int(((max_speed+min_speed)/2) - ((max_speed-min_speed)*distance/cam_resolution[0])))

	elif ((MaxDist < distance < cam_resolution[1]) & (last_motor_command != "LEFT")):
		last_motor_command = "LEFT"
		pe.move(high_speed,low_speed)

	elif ((distance < MinDist) & (last_motor_command != "RIGHT")):
		last_motor_command = "RIGHT"
		pe.move(low_speed,high_speed)

	elif((MinDist < distance < MaxDist) & (last_motor_command != "FORWARD")):
		last_motor_command = "FORWARD"
		pe.move(high_speed,high_speed)

	# show stream
	if show_frames_flag:
		# show vision line. and image center in result
		frame_result[:,int(frame_result.shape[1]/2),:] = 255
		frame_result[int(frame_result.shape[0]*vision_height),:,:] = 255
		cv2.imshow('Frame Result',frame_result)
 
	# clear the stream in preparation for the next frame
	rawCapture.truncate(0)
 
	  # print data
	if (time.time() - t0) > show_data_rate:
		print("Distance =", distance, "; Last command =", last_motor_command, "; Distance flag =", flag_dist, "\n")
		t0 = time.time()

	# if the `q` key was pressed, break from the loop
	key = cv2.waitKey(1) & 0xFF
	if key == ord("q"):
		break

# show range used
print("-> Range used:", lower_color, upper_color, "\n")

# end program
cv2.destroyAllWindows()
if distance_sensor_flag:
	th_status = 0 
	thread_dist.join() 
	print("-> Thread killed.\n")
pe.clean()
# show functionalities used
print("-> Functionalities used: Distance sensor - {}; Manually calibration - {}; Simple calibration - {}; Show stream - {}; Set servos - {}; Variable speed - {}; Search mode - {}.\n".format(distance_sensor_flag, hsv_range_flag,
simple_calibration_flag, show_frames_flag,
servo_flag, variable_speed_flag, 
search_mode_flag, exit_flag))
print("-> Program ended.\n")

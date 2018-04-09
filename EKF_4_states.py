import numpy as np
import cv2
import platform
import sys
import time


# hsv tennis ball yellow
upper = np.array([70, 240, 255])
lower = np.array([20, 45, 130])

# hsv cochonnet red-orange ish
#upper = np.array([150, 105, 255])
#lower = np.array([0, 0, 0,])


np.set_printoptions(formatter={'float': lambda x: "{0:0.2f}".format(x)})

state = np.matrix('0.0;0.0;0.0;0.0') # x, y, xd, yd, 


# P and Q matrices for EKF
P = np.matrix('10.0,0.0,0.0,0.0; \
				0.0,10.0,0.0,0.0; \
					0.0,0.0,10.0,0.0; \
						0.0,0.0,0.0,10.0' )

								
Q = np.matrix('2.0,0.0,0.0,0.0; \
				0.0,2.0,0.0,0.0; \
					0.0,0.0,2.0,0.0; \
						0.0,0.0,0.0,2.0')
							
measurement = np.matrix('0;0')


debug_print = False


def find_ball(frame):

	x = 0
	y = 0
	radius = 0
	
	
	blur = cv2.blur(frame,(8,8))
	
	# set to hsv and gray
	hsv = cv2.cvtColor(blur,cv2.COLOR_BGR2HSV)
	grayBlur = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
	grayFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

	# threshold
	mask = cv2.inRange(hsv, lower, upper)
		
	# open + close to remove noise
	kernel = np.ones((20,20),np.uint8)
	opened = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
	kernel = np.ones((15,15),np.uint8)
	closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, kernel)



	# find contours
	cimg = grayFrame
	ret,thresh = cv2.threshold(closed,127,255,0)
	img, contours, _ = cv2.findContours(thresh, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	#cv2.drawContours(cimg, contours, -1, (255), 3)


	
	# The biggest contour is the one we want (only 1 item in tracking and nice background)
	if (len(contours) > 0):
		biggest_cnt = contours[0]
		biggest_area = cv2.contourArea(biggest_cnt)
		for i in range(0, len(contours)):
			area = cv2.contourArea(contours[i])
			if (area > biggest_area):
				biggest_cnt = contours[i]
				biggest_area = area
	
		(x,y),radius = cv2.minEnclosingCircle(biggest_cnt)
		center = (int(x),int(y))
		radius = int(radius)


	return grayFrame, grayBlur, mask, opened, closed, cimg, (x,y,radius)


def run_EKF_model(state, P, Q, dt):
	
	# model is
	# X(0) = X(0) + X(2)*dt
	# X(1) = X(1) + X(3)*dt	
	# X(2) = X(2) 
	# X(3) = X(3) 
	# it has no input, so Ju = 0

	state[0] = state[0] + dt*state[2]
	state[1] = state[1] + dt*state[3]
	state[2] = state[2]
	state[3] = state[3]

	# covariance matrix gets updated through
	# P = J*P*trans(J) + Q
	# where J = [1, 0, dt, 0;
	#			 0, 1, 0, dt;
	#			 0, 0, 1, 0;
	#			 0, 0, 0, 1]	
		
	J = np.matrix('1.0,0.0,0.0,0.0;\
				   0.0,1.0,0.0,0.0;\
				   0.0,0.0,1.0,0.0;\
				   0.0,0.0,0.0,1.0')
	J[0,2] = dt
	J[1,3] = dt

	P = J*P*(J.transpose()) + Q
	
	return state, P, J

def run_EKF_measurement(state,measurement, P):
	

	# Observation is (x,y) = (X(0), X(1))
	# so H is very simple...
	# H = [1, 0, 0, 0, 0, 0;
	# 		0, 1, 0, 0, 0, 0]

	# sigma_x and sigma_y are pretty decent...
	# R = [sigma_x, 0;
	#		0, sigma_y]


	H = np.matrix('1.0,0.0,0.0,0.0; \
				   0.0,1.0,0.0,0.0')
				 

	R = np.matrix('5.0,0.0;\
					0.0,5.0')
	
	z = measurement - H*state
	HPH = H*P*(H.transpose())
	S = HPH + R
	invS = np.linalg.inv(S)
	K = P*(H.transpose())*np.linalg.inv(S)

	
	state = state + K*z
	P = P - P*(H.transpose())*np.linalg.inv(S)*H*P
	
	
	if (debug_print == True):
		print ('running new measurement')
		print ('norm P is ')
		print np.linalg.norm(P)
		
		print ('z is ')
		print z.transpose()
	
		print ('HPH is ')
		print HPH
		print ('S is ')
		print S
		
		print ('invS is ')
		print invS
	
		print ('PH is ')
		print P*(H.transpose())
	
		print ('K ')
		print K
	
	
	
	return state, P
	
	






# print basic info
print('python ' + platform.python_version())
print('opencv ' + cv2.__version__)





# open camera
cap = cv2.VideoCapture(0)

# find a way to fix camera settings...
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 30)
print(cap.get(cv2.CAP_PROP_BRIGHTNESS))
print cap.get(cv2.CAP_PROP_CONTRAST)
#cap.set(cv2.CAP_PROP_BRIGHTNESS, 20)
#cap.set(cv2.CAP_PROP_CONTRAST, 0.1)


prev_time = time.time()        
i = 0

while(True):
	
	now_time = time.time()
	dt = now_time - prev_time
	
	
	# run the model every 0.01 s
	if (dt > 0.01):
		prev_time = now_time
	
		state, P, J = run_EKF_model(state, P, Q, dt)
	
	
	
	
	# read camera
	ret, frame = cap.read()
	if ret == True:
		# process
		grayFrame, grayBlur, mask, openImg, closedImg, cimg, (x,y,r) = find_ball(frame)
		measurement[0] = x
		measurement[1] = y
		if(measurement[0] != 0) and (measurement[1] != 0):
			state, P = run_EKF_measurement(state, measurement, P)
		
		if(x != 0):
			cv2.circle(cimg, (int(x), int(y)), 50, (255), 5)
			
		if(state[0] != 0):
			cv2.circle(cimg, (int(state[0]),int(state[1])), 20, (255), 3)
		
			
		# display
		display1 = np.concatenate((cv2.resize(grayFrame, (0,0), fx=0.5, fy=0.5), cv2.resize(grayBlur, (0,0), fx=0.5, fy=0.5)),axis=1)
		display2 = np.concatenate((cv2.resize(mask, (0,0), fx=0.5, fy=0.5), cv2.resize(openImg, (0,0), fx=0.5, fy=0.5)),axis=1)
		display3 = np.concatenate((cv2.resize(closedImg, (0,0), fx=0.5, fy=0.5), cv2.resize(cimg, (0,0), fx=0.5, fy=0.5)),axis=1)

		display = np.concatenate((display1,display2, display3),axis=0)

		cv2.imshow('all',display)
		
	# close
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

# clean up
cap.release()
cv2.destroyAllWindows()






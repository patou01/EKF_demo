# Description

This is the result of a quick Python learning experiment I made. It uses a webcam to track a standard yellow tennis ball. The method is very simple and not too robust. An Extended Kalman Filter (EKF) is overlaid to the webcam results in order to increase stability when the ball is not found on the image. 

Press q to exit the program.


# Image processing

This is a very simple, un-optimized detection. The process is as follows

1. Convert the image to HSV
2. Use a Gaussian blur
3. Threshold the picture
4. Open and close
5. Find contours
6. Find the biggest contour
7. The minimal enclosing circle approximates the ball. (x,y) of its center should relate closely to the one of the ball.


# Extended Kalman Filter

The EKF estimates 4 states (x,y,dx,dy) where dx and dy are the velocity of the ball. This EKF uses no inputs to the model prediction, as there is none (the ball is held in the user's hand). The webcam gives data only for x and y.



# Display 

The original image and various intermediate steps are displayed. On the final one, a small circle with the estimated position of the ball through the EKF is overlaid. A larger circle displays the minimal enclosing circle found by the image processing.


# Results

See [Youtube](https://youtu.be/uz0bcjCO1zg)

- No optimization of any ekf parameters, causes large lag.
- Brute forcing the image processing causes oscillations when the occlusion happens slowly.

# todo

- Make a setup that will allow to use a model prediction, eg freefall without initial motion (this requires some knowledge of physical environment). 
- Maybe later make a trajectory estimation for a throw.


# libraries

- python 2.7.12 
- opencv 3.3.1-dev
- numpy 1.11.0

# license

todo


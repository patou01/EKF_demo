# Description

This is the result of a quick Python learning experiment I made. It uses a webcam to track a standard yellow tennis ball. The method is very simple and not too robust. An Extended Kalman Filter (EKF) is overlaid to the webcam results in order to increase stability when the ball is not found on the image.


# Extended Kalman Filter

The EKF estimates 4 states (x,y,dx,dy) where dx and dy are the velocity of the ball. This EKF uses no inputs to the model prediction, as there is none (the ball is held in the user's hand). The webcam gives data only for x and y.



# todo

Make a setup that will allow to use a model prediction, eg freefall without initial motion (this requires some knowledge of physical environment). Maybe later make a trajectory estimation for a throw.

# license

todo


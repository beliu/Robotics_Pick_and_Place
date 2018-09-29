# Robotics_Pick_and_Place

I start by creating the Modified Danavit-Havaford (DH) Table to model the Kuka 210 Arm.
The image below shows the joints, end-effector, and links of the robot. The point of view is looking at the arm directly from the side, so that the arm, with all the joints at 0, would rest entirely within the X-Z plane of the base reference frame. All the other frame origins and axes are shown, along with the DH parameters *a* and *d*.

![Robot Model for Making the DH Table](/images/Robot_Model.jpg)

i            | alpha_i-1     | a_i-1         | d_i           | theta_i
------------ | ------------- | ------------- | ------------- | -------------
1            | 0             | 0             | 0.75          | theta1
2            | -pi/2         | 0.35          | 0             | theta2 - pi/2
3            | 0             | 1.25          | 0             | theta3
4            | -pi/2         | -0.054        | 1.5           | theta4
5            | pi/2          | 0             | 0             | theta5
6            | -pi/2         | 0             | 0             | theta6
EE           | 0             | 0             | 0.303         | 0

### Inverse Kinematics
Once you obtain the xyz coordinates of the wrist-center with respect to the base frame, then you can use geometry to calculate closed-form equations for the joint angles 1, 2, and 3.

In order to calculate the joint 2 angle, I refered to the diagrame below:

![Geometry to Calculate Joint 2](/images/Inverse_Kinematics.jpg)

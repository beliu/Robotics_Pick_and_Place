# Robotics_Pick_and_Place

I start by creating the Modified Danavit-Havaford (DH) Table to model the Kuka 210 Arm.
The image below shows the joints, end-effector, and links of the robot. The point of view is looking at the arm directly from the side, so that the arm, with all the joints at 0, would rest entirely within the X-Z plane of the base reference frame. All the other frame origins and axes are shown, along with the DH parameters *a* and *d*.

![Robot Model for Making the DH Table](/images/Robot_Model.png)

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

First, I refered to the diagram below to calculate the joint 1 angle:

![Geometry to Calculate Joint 1](/images/Inverse_Kinematics_q1.png)

The diagram depicts the Kuka Arm with just Joint 1 and the Wrist center, looking from a top-down view. Therefore, the entire arm is captured in the X-Y plane of Joint 1's reference frame. The angle theta 1 is simply the inverse tan of the y and x component of the wrist center, w.r.t. the base reference frame.

![Theta 1](/images/theta1.png)

In order to calculate the joint 2 angle, I refered to the diagram below:

![Geometry to Calculate Joint 2](/images/Inverse_Kinematics_q2.png)

The diagram depicts the Kuka Arm with Joint 2 rotated at an angle of positive theta2 along its z-axis. The other joints have not rotated, and are therefore at 0. Note that the length *A*, that is, the length that connects J3 to the WC, can be calculated using the parameters from the URDF file. Specifically, I use a_3 and d_4 to calculate *A*. 

![Calculating Distance from J3 to WC](/images/Calculate_A.png)

The length *C*, from J2 to J3, comes directly from the URDF file.
The length *B* is calculated from the coordinates of the wrist center, w.r.t. the base reference frame. We can use the Pythagorean Theorem to calculate *B*, provided we know the lengths of the legs. Those lengths are called out in the diagram. Also, once we have the lengths of the legs of the right triangle whose hypotenuse is *B*, we can also calculate the angle *beta* shown in the diagram. 

After we have the lengths of the triangle *ABC*, we can use the Law of Cosines to calculate the angle *a*. Therefore, from the diagram, we see that 

![Theta 2](/images/theta2.png)

Next, I calculate the closed-form equation for Joint 3. To do so, I refer to the diagram below.

![Geometry to Calculate Joint 3](/images/Inverse_Kinematics_q3.png)

In the configuration above, I have Joint 2 at 0 degrees and Joint 3 rotated at an angle of theta3 in the positive direction along its z-axis. The two perpendicular dotted gray lines centered at Joint 3 represent the original XY-axes before rotation and the dotted black lines represent the new XY-axes after a rotation of theta3. From the diagram, we see that 

![Theta 3](/images/theta3.png) 

where b is the angle between legs A and C and gamma is the angle the link length *a_3* and the link displacement *d_4*. 

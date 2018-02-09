## Project: Kinematics Pick & Place

[//]: # (Image References)

[image11]: ./misc_images/image11.jpg
[image12]: ./misc_images/image12.jpg
[image13]: ./misc_images/image13.jpg
[image14]: ./misc_images/image14.jpg
[image15]: ./misc_images/image15.jpg
[image16]: ./misc_images/image16.jpg
[image17]: ./misc_images/image17.jpg
[image18]: ./misc_images/image18.jpg

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

![alt text][image12]
![alt text][image11]


The description about how obtained the table :

α0 is the twist angle between Z0 and Z1 measured about X0 in a right hand sense. Z0 and Z1 are collinear , so α0=0
A0 is the distance from Z0 to Z1 measured along X0 . Z0 and Z1 are collinear, so a0 = 0
The link offset d1 is the sign distance from X0 to X1, measured along Z1 . d1 = d1
Joint 1 measures the angle between X0 and X1 about the Z1. Since X0 and X1 are paralle and Joint 1 is revolute , so it is θ1

α1 is the twist angle between Z1 and Z2 measured about X1, α1= -pi/2.
A1 is the distance from Z1 to Z2 measured along X1. So it's a1
The link offset d2 is the sign distance from X1 to X2, measured along Z2. Its 0.
Joint 2 measures the angle between X1 and X2 about the Z2. It is θ2 -pi/2. 

α2 is the twist angle between Z2 and Z3 measured about X2. Z2 and Z3 are  parallel , so α2=0.
A2 is the distance from Z2 to Z3 measured along X2. It's a2.
The link offset d3 is the sign distance from X2 to X3, measured along Z3. d3=0.
Joint 3 measures the angle between X2 and X3 about the Z3. It's θ3.

α3 is the twist angle between Z3 and Z4 measured about X3. Z3 and Z4 are always orthogonal measured about X3 , so α3=-pi/2.
A3 is the distance from Z3 to Z4 measured along X3. It's a3.
The link offset d4 is the sign distance from X3 to X4, measured along Z4. It's d4
Joint 4 measures the angle between X3 and X4 about the Z4. It's θ4.

α4 is the twist angle between Z4 and Z5 measured about X4. Z4 and Z5 are always orthogonal measured about X4 , so α4= pi/2.
A4 is the distance from Z4 to Z5 measured along X4. a4=0.
The link offset d5 is the sign distance from X4 to X5, measured along Z5. d5=0.
Joint 5 measures the angle between X4 and X5 about the Z5.It's θ5.

α5 is the twist angle between Z5 and Z6 measured about X5.  Z5 and Z6 are always orthogonal measured about X5 so α5=-pi/2.
A5 is the distance from Z5 to Z6 measured along X5. a5=0.
The link offset d6 is the sign distance from X5 to X6, measured along Z6. d6=0.
Joint 6 measures the angle between X5 and X6 about the Z6. It's θ6.

α6 is the twist angle between Z6 and ZG measured about X6. Z6 and ZG are collinear , so α6=0
A6 is the distance from Z6 to ZG measured along X6. a6=0.
The link offset d7 is the sign distance from X6 to XG, measured along ZG. d7=dG.
Joint 7 measures the angle between X6 and XG about the ZG.  θ7=0.

![alt text][image13]
α = arm twist angle

a = arm link length

d = arm link offset

θ = arm joint angle

Joint 1
* a0 = 0 (base link)
* d1 = link2(z) = 0.75

Joint 2
* a1 = link2(x) = 0.35
* d2 =0  (X1 and X2 are perpendicular)

Joint 3
* a2 = link3(z) = 1.25
* d3 =0 (X2 and X3 are coincident)

Joint 4
* a3 = link3(z) - link5(z) = 2–1.9464 = 0.0536
* d4 = link5(x)- link3(x) = 1.8499–0.3485= 1.5014

Joint 5
* a4 = 0 (O4 and O5 are coincident)
* d5 = 0 (X4 and X5 are coincident)

Joint 6
* a5 = 0 (O5 and O6 are coincident)
* d6 =0 (X5 and X6 are coincident)

Joint 7 (Gripper Joint)
* a6 = 0 (Z6 is coincident with Z7)
* d7 = link(gripper x)-link5(x)= 2.1529–1.8499 = 0.303

Modified DH parameters

![alt text][image14]

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

The homogeneous transform from frame i-1 to frame i is constructed as a sequence of four basic transformations, two rotations and two translations as follows: 

![alt text][image15]

From the DH paramater table the individual trsnaformation matrices is :
* 1.Matrix([[cos(q1), -sin(q1), 0, 0], [sin(q1), cos(q1), 0, 0], [0, 0, 1, 0.750000000000000], [0, 0, 0, 1]])
* 2.Matrix([[cos(q2 - 0.5*pi), -sin(q2 - 0.5*pi), 0, 0.350000000000000], [0, 0, 1, 0], [-sin(q2 - 0.5*pi), -cos(q2 - 0.5*pi), 0, 0], [0, 0, 0, 1]])
* 3.Matrix([[cos(q3), -sin(q3), 0, 1.25000000000000], [sin(q3), cos(q3), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
* 4.Matrix([[cos(q4), -sin(q4), 0, -0.0540000000000000], [0, 0, 1, 1.50000000000000], [-sin(q4), -cos(q4), 0, 0], [0, 0, 0, 1]])
* 5.Matrix([[cos(q5), -sin(q5), 0, 0], [0, 0, -1, 0], [sin(q5), cos(q5), 0, 0], [0, 0, 0, 1]])
* 6.Matrix([[cos(q6), -sin(q6), 0, 0], [0, 0, 1, 0], [-sin(q6), -cos(q6), 0, 0], [0, 0, 0, 1]])
* 7.Matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.303000000000000], [0, 0, 0, 1]])


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

Find wrist center first:
Use the quation below  calculates the wrist center :
![alt text][image16]

From the DH parameter table,  the griper link offset d = 0.303. 

Imply that WC = EE position - (0.303) * Rrpy[:,2],

which 
Rrpy =EE rotation matrix * R_corr 

and 
R_corr = a correctional rotation composed by a rotation on the Z axis of 180° (π) followed by a rotation on the Y axis of -90 (-π/2).

After we get WC, we can calculates the angles from θ1 to θ6.

![alt text][image17]

θ1 = atan2(y of wc, x of wc)
r = sqrt(wx**2+wy**2) - 0.35 # a1: 0.35

![alt text][image18]

x = sqrt(0.96*0.96+0.054*0.054)=sqrt(0.9216+0.002916=0.924516)=0.962

c=0.962

b = A =1.5

a = 0.54

angle_x = acos((b * b + c * c -a * a) / (2 * b * c))

theta3 = pi/2 - (b + angle_x)

Using the individual DH transforms R0_6 = R0_1*R1_2*R2_3*R3_4*R4_5*R5_6 

Using the individual DH transforms R0_6 = R0_1*R1_2*R2_3*R3_4*R4_5*R5_6 

R3_6 = Invert of R0_3 * R0_6

theta4 = atan2(nR3_6[2,2], -nR3_6[0,2])

theta5 = atan2(math.sqrt(nR3_6[0,2] * nR3_6[0,2] + nR3_6[2,2]*nR3_6[2,2]),nR3_6[1,2])

theta6 = atan2(-nR3_6[1,1], nR3_6[1,0])


### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]



## Project: Kinematics Pick & Place

[//]: # (Image References)

[image11]: ./misc_images/image11.jpg
[image12]: ./misc_images/image12.jpg
[image13]: ./misc_images/image13.jpg

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

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

![alt text][image13]


    α = arm twist angle
    a = arm link length
    d = arm link offset
    θ = arm joint angle

Joint 1
    a0 = 0, since this is the base link.
    
    d1 = link2(z) = 0.75
    
Joint 2
    a1 = link2(x) = 0.35
    
    d2 =0, since X1 and X2 are perpendicular.
    
Joint 3
    a2 = link3(z) = 1.25
    
    d3 =0, since X2 and X3 are coincident.
    
Joint 4
    a3 = link3(z) - link5(z) = 2–1.9464 = 0.0536
    
    d4 = link5(x)- link3(x) = 1.8499–0.3485= 1.5014
    
Joint 5
    a4 = 0, since O4 and O5 are coincident.
    
    d5 =0, since X4 and X5 are coincident.
    
Joint 6
    a5 = 0, since O5 and O6 are coincident.
    
    d6 =0, since X5 and X6 are coincident.
    
Joint 7 (Gripper Joint)
    a6 = 0, since Z6 is coincident with Z7.
    
    d7 = link(gripper x)-link5(x)= 2.1529–1.8499 = 0.303
    

Modified DH parameters :
DH = {   alpha0: 0,      a0: 0,      d1: 0.75,    q1: q1,

         alpha1: -pi/2,  a1: 0.35,   d2: 0,       q2: q2-pi/2,
         
         alpha2: 0,      a2: 1.25,   d3: 0,       q3: q3,
         
         alpha3: -pi/2,  a3: 0.0536, d4: 1.5014,  q4: q4,
         
         alpha4: pi/2,   a4: 0,      d5: 0,       q5: q5,
         
         alpha5: -pi/2,  a5: 0,      d6: 0,       q6: q6,
         
         alpha6: 0,      a6: 0,      d7: 0.303,   q7: 0}


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

And here's where you can draw out and show your math for the derivation of your theta angles. 

![alt text][image2]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]



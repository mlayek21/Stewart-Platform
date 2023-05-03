# Stewart Platform Model Description:

This document provides details on our Stewart Platform model, including its URDF file, inverse kinematics, and real-world physics simulation.

## Overview

The Stewart Platform is a type of parallel manipulator with six degrees of freedom. It consists of a fixed base, a moving platform, and six legs that connect the base and the platform. The legs are actuated by linear actuators, which allows the platform to move in three translational and three rotational directions.

## Details

- **Base:** The base of the model is made of aluminum material.
- **Platform:** The platform of the model is also made of aluminum material.
- **Cylinder Material:** The cylinder material is made of aluminum.
- **Universal Joint Pin:** The universal joint pin is made of steel.
- **Piston Rod:** The piston rod is made of titanium.
- **Model Weight:** The total weight of the model is 37.5 kg.
- **Base and Platform Radius:** The base and platform have a radius of 200 mm.
- **Base to Platform Height:** The distance between the base and platform is 257.5 mm.

![Stewart Platform](https://github.com/mlayek21/Stewart-Platform/blob/main/Files/Stewert%20v14.png)

## URDF File Details

- This repository includes a URDF file that describes the Stewart Platform model. The URDF file contains information on the robot's links, joints, and sensors, as well as its visualization and collision properties. However, to create the URDF of the parallel mechanism, we first need to convert it into an open chain and then compile it as a URDF.

- Once the URDF file is created, the model can be assembled in a physics simulator by adding constraints. The simulator allows us to simulate the motion and behavior of the platform in real-world physics.

- In the case of the Stewart Platform model, it is necessary to ensure that all the joints are properly connected to form a parallel chain. The image below shows an example of the joint connections in the Stewart Platform model.
  
  <a href="url"><img src="https://github.com/mlayek21/Stewart-Platform/blob/main/Files/URDF.jpeg" align="center" height="600" width="800" ></a>

By simulating the model in a physics simulator, we can study its behavior and optimize its performance for various applications.


# Inverse Kinematics of Stewart Platform:
The inverse kinematics of a Stewart Platform is the process of determining the joint angles required to position the platform in a specific orientation. Since the platform has six degrees of freedom, six equations are required to determine the joint angles. The inverse kinematics of the Stewart Platform can be solved using geometric, analytical, or numerical methods. The solution to the inverse kinematics problem is important for precise control of the platform, which is essential in applications such as flight simulators, motion platforms, and virtual reality systems. However, for certain applications that require the platform to have a reduced degree of freedom, such as RPR 3dof, we can restrict the translational motion of the platform. This simplifies the inverse kinematics problem and allows for precise control of the platform with fewer degrees of freedom.

## Base and Platform Anchors

Standard notation for the fundamental parameters that determine the mechanical configuration is

- $r_B\to$ Radius of Base (Bottom)

- $r_P\to$ Radius of Platform (Top)

- $\gamma_B\to$ Half of angle between two anchors on the base

- $\gamma_P\to$ Half of angle between two anchors on the platform

We may define $\psi_B \in R^{6 \times 1}$ & $\psi_P\in R^{6 \times 1}$ and the polar coordinates of the anchors on a unit circle radius using these $\gamma_B$ &  $\gamma_P$. These are derived from the gamma values of $B$ and $P$.

If we have $r_B$ and $r_P$, then we may define as the coordinates of the anchors in their respective local frames in cartesian space, which are $B \in R^{6 \times 3}$ and $P\in R^{6 \times 3}$. For instance, an illustration of the anchor points on the base B may be found below.


![base platform dimention](https://github.com/mlayek21/Stewart-Platform/blob/main/Files/output1.png)

## Positioning Oneself at Home

The gap between the base and the platform at the starting point,, must then be specified. Your resting linear actuator length is. Let's say it's the base plate radius.

Using the usual notation, we must additionally define the rotation matrices.


$$ R_z{(\theta)}=
   \begin{bmatrix} 
   \cos{\theta} & -\sin{\theta} & 0 \\
   \sin{\theta} & \cos{\theta} & 0 \\
   0 & 0 & 1 \\
   \end{bmatrix} $$
   
$$ R_y{(\theta)}=
  \begin{bmatrix} 
  \cos{\theta} & 0 & \sin{\theta} \\
  0 & 1 & 0 \\
  -\sin{\theta} & 0 & \cos{\theta} \\
  \end{bmatrix} $$

$$ R_x{(\theta)}=
  \begin{bmatrix} 
  1 & 0 & 0 \\
  0 & \cos{\theta} & -\sin{\theta} \\
  0 & \sin{\theta} & \cos{\theta} \\
  \end{bmatrix} $$


## Using Linear Actuators to Determine Inverse Kinematics

We may now begin working on the inverse kinematics problem.

Using the target translation vectors $T = (t_x,t_y,t_z)^T$ and the rotation vector $\theta = (\theta_x, \theta_y, \theta_z)^T$, determine the required leg length.

After the plate has been rotated and translated as desired, all that remains is to determine the new locations of the various anchors.

Given that each leg's job is to establish a connection between the base and the platform's anchor, the required vector (direction and length) for each leg is simply the leg's location in 3D space with respect to its corresponding base anchor.
$$l = T+H+p+R(\theta)-B$$
Where, $T$ and $H$ are in $R^{3 \times 1}$ replicated 6 times to have dimensions $R^{3 \times 6}$ to facilitate matrix calculations.

It's possible to interpret this as,

$$l = Desired~~Translation+displacement(Base~~Center,Home~~Pos) + Coordinate~~Rotation(globalframe)$$

A leg's length is simply the leg vector's magnitude.
$$|l| = (l_{k,x}^2+l_{k,y}^2+l_{k,z}^2)^{0.5}$$

Simply adding the displacement of each leg's anchor at the ground yields the leg's position relative to the global frame's centre of base.

And that's only to figure out the inverse kinematics of linear actuator-driven Stewart platforms.

![IK](https://github.com/mlayek21/Stewart-Platform/blob/main/Files/output4.png) ![IK1](https://github.com/mlayek21/Stewart-Platform/blob/main/Files/output2.png)

# Linear Actuators
The linear actuator function converts servo motion to linear actuation using PWM signals. PWM stands for Pulse-Width Modulation, which is a technique for controlling the amount of power delivered to a device by rapidly turning the power on and off. By varying the duty cycle (the proportion of time that the power is on) and the frequency of the pulses, we can control the position, speed, and force of the linear actuator.

The linear actuator function uses the following formula to calculate the actuation step size:
```
frequency = 50                                   
max_force = 1000                                 
steps = int(actuation_duration * frequency)  
pwm_period = 1.0 / frequency                  
duty_cycle = 0.5                            
pwm_high_time = pwm_period * duty_cycle 
actuation_step = [l / steps for l in linear_distance]
pwm_signal = i * pwm_period % pwm_period < pwm_high_time
```
where 'linear_distance' is an array of the desired linear distances to be covered by each actuator, 'steps' is the number of PWM steps required to cover the distance in the given actuation duration, and 'actuation_step' is the step size for each actuator.

The function then generates PWM signals with a frequency of 50 Hz and a duty cycle of 50%, and actuates the linear actuators according to the desired distance and duration. The function also applies a maximum force of 1000 N.M to each actuator to ensure stability and safety.

By using this function, users can easily convert servo motion to linear actuation and control the position, speed, and force of the linear actuators in their applications.

# Simulation:
- **Test 1:** To simulate RPR 3DOF motion, we used a 6-DOF Stewart platform with the platform locked to a 100mm displacement towards the Z-axis with respect to the platform reference frame. We then simulated a sequence of motions as follows:

      - Yaw rotation of 30 degrees in 6 seconds
      - Roll rotation of 20 degrees in 2 seconds
      - Pitch rotation of 15 degrees in 2 seconds

    https://user-images.githubusercontent.com/110429424/235838761-e91a4729-4787-4fc8-832e-aa768ffc2209.mp4


- **Test 2:**

      - 30 degrees yaw in 3 seconds and return back to home position within 3 seconds
      - 15 degrees pitch in 2 seconds return back to home position within 2 seconds
      - 15 degrees roll in 1 seconds return back to home position within 1 seconds 

     https://user-images.githubusercontent.com/110429424/235838624-06085101-13ff-43ae-9283-69601d753f85.mp4
     
     
- **Test 3:** In the 6DOF simulation, we designed a custom environment and utilized custom function to generate a spiral trajectory for the platform. Custom Python scripts were used to generate forces and torques for each actuator to achieve accurate motion. This simulation highlights the versatility and precision of the Stewart platform for robotics and automation.     

   https://user-images.githubusercontent.com/110429424/235838888-71b334f0-31dd-44c2-8c3f-d56a73b0f7a4.mp4


     




- **Conclusion**: 
  - The simulation was performed using a custom Python script that utilized the PyBullet physics engine for simulating the motion of the platform. The script included control algorithms that generated the required forces and torques for achieving the desired motion. We also designed a custom simulation environment that closely resembled the real-world setup of the platform.

  - During the simulation, we monitored the position and orientation of the platform at each time step and recorded the resulting motion trajectories. The simulation results showed that the platform was able to achieve the desired RPR 3DOF motion accurately and smoothly.

  - Overall, this simulation provides valuable insights into the behavior of the RPR platform and can be used to optimize its design and control algorithms for real-world applications.
```
Author: Monojit Layek
Licence: This model is provided under the MIT License. Feel free to use and modify the model as you wish.
```

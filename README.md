# Tomato-Harvester
*4-dof Autonomous tomato harvester robotic manipulator featuring a MIMO LQG control system and computer vision, implemented in MATLAB/Simulink.*

## Project Overview
In this project, I developed the mechanical design, electromechanical model, and control architecture for an autonomous tomato harvester robotic arm. Using voltage control across two DC motors and a DC linear actuator, I was able to precisely position the end-effector using a Linear Quadratic Regulator (LQR) controller. Of the 9 total states, the 3 positions ($\phi, \theta, r$) were the only ones measured so a Kalman Filter was used to estimate the remaining states.

## Key Features
* **Precise Control:** Achieved end-effector position error magnitude under 3mm at max extension.
* **Optimal State Estimation:** Implemented a Kalman Filter to estimate unmeasured states (motor currents and velocities) from the three measured states.
* **System Identification:** Applied grey-box system identification to accurately obtain unkown model parameters using experimental data.
* **Custom Fabrication:** Designed many hardware elements using Design for Manufacturing (DFM) principles and machined components on a vertical mill.

## Visuals
### 1. Control System Demonstration
Closed-loop system moving to a pseudo tomato location, closing gripper, retracting, opening gripper, and returning to the origin.

<p align="center">
  <img src="./media/control_sys_test_hardware.webp" width="35%" alt="Control System Demonstration"/>
</p>

### 2. Simulink Control Architecture
The control system is built in Simulink/Simulink Desktop Real-Time and operates as a state machine. It captures images on both cameras, identifies tomatoes and triangulates positions, updates setpoint and moves to tomato location, closes gripper, updates setpoint and retracts, opens gripper, updates setpoint and returns to origin, then repeats.

<p align="center">
  <img src="./media/simulink_control_sys.png" width="49%" alt="Control System"/>
  <img src="./media/simulink_state_machine.png" width="49%" alt="State Machine Logic"/>
</p>

### 3. Controller Performance
The following figure shows the control input voltage (top) and the corresponding position and set point (bottom) for each of the three axes of a test case.

<p align="center">
  <img src="./media/v_vs_pos.png" width="75%" alt="Control input vs Position"/>
</p>

### 4. Mechanical Design
The mechanical design was chosen to resemble spherical coordinates using two brushed DC motors and a screw linear actuator for the main motion and a stepper motor for the gripping mechanism.

<p align="center">
  <img src="./media/actual_assembly.png" width="40%" alt="Actual Assembly"/>
  <img src="./media/cad_assembly_2.png" width="40%" alt = "CAD Assembly"/>
</p>

### 5. Electromechanical Dynamics Model
Derived equations of motion for the electrical and mechanical system.

<p align="center">
  <img src="./media/simulink_EOMs.png" width="35%" alt="Equations of Motion"/>
</p>

## Skills & Software Used
* **Software:** MATLAB, Simulink, Simulink Desktop Real-Time, SolidWorks
* **Hardware:** oscilloscope, rotary encoders, data acquisition card, motor controllers, microcontrollers, vertical mill
<!--**Concepts:** -->

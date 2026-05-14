# Tomato-Harvester
*Autonomous tomato harvester mechanical arm implemented in MATLAB/Simulink.*

## Project Overview
In this project, I developed a Linear Quadratic Gaussian (LQR controller + Kalman Filter state estimator) control system for a spherical-coordinates inspired robotic arm. I used voltage control to precisely position the end effector using 3 DC motors by utilizing the dynamics model of the motors and mechanical system. Of the 9 total states, the 3 positions ($\phi, \theta, r$) were the only ones measured so a Kalman Filter was used to estimate the remaining states (motor currents, velocities). 

## Key Features
* Estimated impulsive maneuvers and drag.
* Accounted for $J2/J3$ and lunar third-body effects.
* Handled simulated range and range-rate measurements.

## Visuals
Mechanical Design

<p float="left">
  <img src="./media/actual_assembly.png" height="300" alt="Actual Assembly" />
  <img src="./media/cad_assembly.png" height="300" alt = "CAD Assembly"/>
</p>

Control System

<p float="left">
  <img src="./media/simulink_control_sys.png" height="200" alt="Control System"/>
  <img src="./media/simulink_state_machine.png" height="200" alt="State Machine Logic"/>
</p>

Electromechanical Dynamics Model

<img src="./media/simulink_EOMs.png" height="50%" alt="Equations of Motion">

## Skills Used
**Software:** MATLAB
**Concepts:** State Estimation, Trajectory Optimization

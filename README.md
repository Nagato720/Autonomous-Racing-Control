# F1 Vehicle Dynamics & SIL Control Architecture

This repository contains the MATLAB codebase and technical documentation for a Software-in-the-Loop (SIL) control architecture designed to navigate a rear-wheel-drive (RWD) F1 vehicle through the Circuit of the Americas. 

**Full Technical Analysis:** [Read the Final Report PDF Here](Final_Report.pdf)

## Project Highlights
* **Optimized Trajectory:** Developed a curvature-based velocity profile and "out-in-out" geometric racing line using Modified Akima (`makima`) splines, achieving a zero-violation closed-loop lap time of **178.30 seconds**.
* **Decoupled Control System:** Implemented a discrete LQR lateral steering controller paired with a PID + Feedforward longitudinal speed controller to respect absolute tire friction-circle limits (5000 N).
* **Forensic Systems Analysis:** Conducted a deep-dive diagnostic into continuous ODE solver mathematics, successfully isolating an integration divergence caused by Zero-Order Hold (ZOH) data truncation within the Runge-Kutta (RK4) sub-steps.

## Repository Contents
* `car_RWD_with_control.m` - The primary discrete LQR and PID control algorithms featuring persistent memory allocation for digital ECU simulation.
* `Derby_Path.m` - Spatial pathing and velocity profile generation logic.
* `CarSimRealTime.m` - The continuous ODE solver and physics environment wrapper.
* `F1CarData.mat` - Static vehicle parameters (mass, aero drag, physical actuator limits).
* `Final_Report.pdf` - Comprehensive breakdown of controller tuning, mathematical constraints, and simulation telemetry.

## Technologies Used
* MATLAB / Simulink
* Linear Quadratic Regulator (LQR) Control Theory
* PID & Feedforward Control
* Kinematic Bicycle Modeling
* Numerical Integration (RK4) Diagnostics

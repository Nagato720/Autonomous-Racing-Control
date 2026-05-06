# F1 Vehicle Dynamics & SIL Control Architecture

This repository contains the MATLAB codebase and technical documentation for a Software-in-the-Loop (SIL) control architecture designed to navigate a rear-wheel-drive (RWD) F1 vehicle through the Circuit of the Americas. 

📄 **[Read the Full Technical Report Here](Final_Report.pdf)** *(Note: Click "View raw" or download the PDF to ensure the internal document links work correctly).*

## Project Highlights
* **Optimized Trajectory:** Developed a curvature-based velocity profile and "out-in-out" geometric racing line using Modified Akima (`makima`) splines, achieving a zero-violation closed-loop lap time of **172.15 seconds**.
* **Decoupled Control System:** Implemented a discrete LQR lateral steering controller paired with a PID + Feedforward longitudinal speed controller to respect absolute tire friction-circle limits (5000 N Front, 5500 N Rear).
* **Forensic Systems Analysis:** Conducted a deep-dive diagnostic into continuous ODE solver mathematics, successfully isolating an integration divergence caused by Zero-Order Hold (ZOH) data truncation within the Runge-Kutta (RK4) sub-steps.

## Repository Contents
* `Final_Report.pdf` - Comprehensive technical breakdown of controller tuning, mathematical constraints, and simulation telemetry.
* `Derby_LapTimeMain.m` - The primary ODE solver wrapper and physics environment script used to execute the simulation.
* `Steering_control.m` - The discrete LQR lateral control algorithm featuring persistent memory allocation for digital ECU simulation.
* `Speed_control.m` - The discrete PID + Feedforward longitudinal speed control algorithm.
* `Derby_Path_Generation.m` - The core logic for spatial pathing and velocity profile generation.
* `Derby_Path.mat` - The generated and highly optimized trajectory data array.
* `The_Derbmobile.mat` - Resultant vehicle performance metrics.
* `Utilities/` - Directory containing supporting functions and baseline dependencies.

## How to Run the Simulation
1. Clone this repository or download all files into a single local directory.
2. Open MATLAB and navigate to the directory.
3. Open and run **`Derby_LapTimeMain.m`** to execute the simulation, evaluate the controllers, and generate the final telemetry plots. 

## Technologies Used
* MATLAB / Simulink
* Linear Quadratic Regulator (LQR) Control Theory
* PID & Feedforward Control
* Kinematic Bicycle Modeling
* Numerical Integration (RK4) Diagnostics

## Acknowledgments
Special thanks to **Professor Chaozhe He** for providing the foundational vehicle dynamics wrapper (`CarSimRealTime.m`), baseline physics equations, and base vehicle parameters that made this Software-in-the-Loop (SIL) control project possible.

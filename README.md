# Model Predictive Path Integral Control

## General Description
This project implements a predictive path control based on the Model Predictive Path Integral (MPPI) method applied to a single-track vehicle model. The controller optimizes the vehicle's trajectory while navigating through a map at high speed.

## Simulation Instructions
1. **Main File:**
   To run the simulations, open and execute the `RUN_MPPI.m` file.

2. **Parameter Adjustment:**
   Before running, adjust the parameters at the beginning of the `RUN_MPPI.m` file as needed (e.g., initial conditions, time horizon, etc.).

## Changing Maps
1. **Static Maps:**
   In the `MPPI_nav_controller` file, under the comment:
   ```matlab
   % Creates the map and a matrix with the cost of each grid square
   ```
   Uncomment the line of the desired map.

2. **Dynamic Maps:**
   Under the comment:
   ```matlab
   % Begin MPC iterations
   ```
   Uncomment the corresponding lines to activate dynamic maps.

With these configurations, you can explore different environments and analyze the controller's behavior under various conditions.


GitHub:https://github.com/PachiElCreador/MPPI-for-automous-navigation

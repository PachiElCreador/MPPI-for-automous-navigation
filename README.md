# Model Predictive Path Integral Control (MPPI)

## Overview
This project implements a **Model Predictive Path Integral (MPPI) Control** framework for autonomous navigation and control in complex environments. It is based on **Information Theoretic MPC** principles and designed to handle stochastic dynamics, non-linear cost functions, and high-velocity navigation tasks.

The core functionality includes creating dynamic maps, sampling-based trajectory optimization, and visualizing the results through simulation. The approach leverages sampling-based optimization for real-time control applications, such as aggressive autonomous driving and navigation in challenging environments.

---

## Features
- **Dynamic Map Creation**: Multiple map generation options with adjustable obstacle configurations.
- **Path Planning and Optimization**: Sampling-based optimization to find the optimal trajectory.
- **Visualization**: Real-time simulation visualization and video recording of results.
- **Cost Function Customization**: Support for running and terminal cost functions to optimize different objectives.
- **Stochastic Control**: Includes system noise modeling and robust control design.

---

## File Descriptions

### Core Implementation
- **`MPPI_nav_controller.m`**: Core implementation of the MPPI control algorithm. This script manages trajectory sampling, cost evaluation, and state updates during the simulation.
- **`RUN_MPPI.m`**: Entry point for running simulations. Configures parameters, initializes the controller, and executes the navigation tasks.

### Map Generation
- **`createMap.m`**: Generates a square grid map with customizable obstacles and cost layers.
- ![image](https://github.com/user-attachments/assets/2d7b0cf1-cf0b-484b-bbf2-89f83b7d8dbe)

- **`dynamic_map.m`**: Creates a map with dynamic obstacles, including cost adjustments based on proximity to obstacles.
- ![image](https://github.com/user-attachments/assets/5bee1561-6f04-4297-bb4e-818db49d0227)

- **`Infinite.m`**: Defines a map with infinite-like patterns and customizable obstacle regions.
- ![image](https://github.com/user-attachments/assets/932faa21-165b-433c-b319-2736dcbb324e)

- **`LaberintoS_2.m`**: Creates a maze-like map for testing path planning in constrained environments.
- ![image](https://github.com/user-attachments/assets/679bc7d5-d424-466f-9e72-e366ffe26632)
- 

- **`Oval.m`**: Generates an oval-shaped map with inner and outer obstacle boundaries.
- 


### Utility Functions
- **`ode1.m`**: Implements the forward Euler method for solving ordinary differential equations (ODEs) used in system dynamics.

### Supporting Documents
- **`Information Theoretic MPC.pdf`**: A detailed explanation of the Information Theoretic MPC framework.
- **`Theory and Application to Autonomous Driving.pdf`**: A paper exploring the application of the framework to aggressive autonomous driving.

---

## Usage
1. **Setup**:
   - Ensure you have MATLAB installed with the necessary toolboxes (e.g., Robotics Toolbox).

2. **Run Simulation**:
   - Configure parameters in `RUN_MPPI.m`.
   - Run the script to execute the MPPI algorithm on a selected map.

3. **Map Selection**:
   - Modify the map initialization in `MPPI_nav_controller.m` by uncommenting the desired map function.

4. **Visualize Results**:
   - Simulation outputs include visualizations and a video file (`simulation_output.mp4`).

---

## Configuration
Key parameters are defined in `RUN_MPPI.m`:
- **`param.timesteps`**: Horizon length for trajectory optimization.
- **`param.samples`**: Number of sampled trajectories per iteration.
- **`param.iterations`**: Maximum iterations for optimization.
- **`param.xmeasure`**: Initial state configuration.
- **`param.x_desired`**: Target state configuration.
- **`param.lambda`**: Temperature parameter controlling exploration.

---

## References
This project was inspired by the following work:
- Grady Williams et al., *Information Theoretic MPC for Model-Based Reinforcement Learning* (2017).
- Grady Williams et al., *Information Theoretic MPC: Theory and Applications to Autonomous Driving* (2017).

---

## Acknowledgments
Thanks to the authors of the referenced works.




GitHub:https://github.com/PachiElCreador/MPPI-for-automous-navigation

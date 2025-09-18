# Vanguard Flight Simulation 

This repository contains a MATLAB+SIMULINK flight simulation environment
for the Vanguard vehicle. It models rocket dynamics, controls, and
environmental effects.

## Repository Structure
### Key Components

- **Models/**  
  Modular Simulink subsystems (aircraft dynamics, sensors, actuators, environment, control logic).  

- **Plotting/**  
  Post-processing scripts for visualizing outputs (altitude, attitude, velocity, forces, etc.).  

- **Utils/**  
  General utilities such as math helpers, data handling, or interpolation functions.  

- **FlightSimulation.slx**  
  The main fella, the big cheese, **THE** Simulink model.

- **RunSimulink.m**  
  Script that sets up simulation parameters, runs the Simulink model, and saves simulation outputs to SimOut struct.

- **setupEnv.m**  
  Adds all necessary folders to the MATLAB path and prepares the workspace
---

## Getting Started

1. Open MATLAB.
2. Ensure you have the "Aerospace Toolbox", "Navigation Toolbox", and "Aerospace Blockset" add-ons
3. Run `setupEnv.m` to configure the simulation environment.
    - Anytime you add a new subfolder, this **NEEDS** to be re-run to add 
      it to path
4. Configure any models/parameters in `RunSimulink.m` as needed.
5. Start the simulation with `RunSimulink.m`
6. Utilize the plotting scripts in `Plotting/` to analyze the results!
Note: plotSim takes in (SimOut) as arguments: run ``plotSim(SimOut)`` in the command window
Note: plotNav takes in (SimOut, params.navInds) as arguments: run ``plotNav(SimOut, params.navInds)`` in the command window

## Typical Workflow - Adding new vehicle
1. Create new Vehicle "Kinematics" model in the Kinematics folder, following
    the current naming scheme for variables **is** a necessity.
2. Create new Aerodynamic model following the current template to load 
    aerodynamic data. There are two implementations, one loading ANSYS CFD results
    and another loading RASAERO and OpenRocket data. CFD results will always
    be more accurate but may not have as many points as OpenRocket/RASAERO.
    The interpolation functons will take care of anything you need.
3. Upload any new motor configuration files to `Models\Motor\EngineData`
    - The simulation inputs .RSE files which can be obtained for your motor
    from [ThrustCurve.org](https://www.ThrustCurve.org)

## Typical Workflow - Configuring new sensors
Currently, this is a hardcoded process... `Models\Sensors\Params` has some
examples of sensors used on the 
[Vanguard V2.0 Flight Computer](https://github.com/Frostydev-Avionics/Vanguard-PCB).
These sensors are then configured and used in the Simulink model in the
Flight Computer block, and added to the SensorBus for use elsewhere
in the model.

## Typical Workflow - Buses
Simulink is a wonder to work with, and sometimes requires **STRICT** definition
of recursively delayed signals that contain buses. These bus definitions
can be found in the `Initialization` folder and if you are adding **ANYTHING**
to a Bus, check if there is a strict initialization for it. Currently,
the navigator bus is the only one this is required for.

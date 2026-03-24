# Regressor-based Adaptive Controller - ArduPilot

A custom attitude controller for multirotor platforms built on ArduPilot's custom controller plugin. Implements a regressor based adaptive control law that estimates the inertial parameters online.

## How it works

The controller computes torque commands using a sliding surface defined over attitude and angular rate error. An adaptation law updates inertia estimates `a_hat` at runtime using a projection-based algorithm that keeps the estimates within user-defined limits.

All controller states are logged via ArduPilot's own logger (messages `CCL0`–`CCL3`).

Official Ardupilot's documentation on custom attitude controller (https://ardupilot.org/dev/docs/copter-adding-custom-controller.html)

## Setup

**Requirements**:

- Ubuntu 22.04.
- Gazebo environment, from Ardupilot's official documentation (https://ardupilot.org/dev/docs/sitl-with-gazebo.html)

Clone the repository and initialize submodules.

```bash
git clone --recurse-submodules https://github.com/JoC2000/Ardupilot_AdaptiveController.git
```

Enter the repository directory and run the setup script, it will put the new and modified files into `ardupilot`. This modifies the custom controller plugin to add the adaptive controller.
```bash
cd Ardupilot_AdaptiveController
chmod +x scripts/setup.sh
./scripts/setup.sh
```

Reload shell, then:

```bash
source ~/.profile
```

## SITL with Gazebo

```bash
cd ardupilot
./waf configure --board sitl --debug
./waf copter
```

After building the firmware, you will need 2 terminal windows in order to run the simulation with mavproxy as GCS and Gazebo.

On the first terminal window run:

```bash
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console
```

On the second terminal window run:

```bash
cd gz_ws/src/ardupilot_gazebo/worlds
gz sim -v4 -r iris_runway.sdf
```

For instructions on flying with SITL refer to official Ardupilot's documentation on SITL (https://ardupilot.org/dev/docs/using-sitl-for-ardupilot-testing.html)

## Use on real hardware
Must flash firmware according to specific flight controller board. For the specs of the FC board, at least 2MB of flash memory is required.

For this example it is configured on a Cube Orange Plus with an extra option to enable the custom control plugin.
```bash
cd ardupilot
./waf configure --board CubeOrangePlus copter --enable-custom-controller
./waf copter --upload
```

## Enabling the Controller

Using mavproxy console or other GCS, set the parameters in order to select the adaptive controller.

```
param set CC_TYPE 3
param set CC_AXIS_MASK 7
param set RC6_OPTION 109
reboot
```
**Note**: You can use any other RCX_OPTION depending on how you have set up your RC.

After rebooting the vehicle, in order to activate the custom controller you can just set the RC6 with commands in the mavproxy terminal.
It is recommended to enable/disable the custom controller once the vehicle has completed takeoff.

```bash
rc 6 2000 #Enables custom controller
rc 6 1000 #Disables custom controller
```

After enabling the controller you can then fly the drone in `GUIDED`, `LOITER`, `AUTO` or any other supported flight mode.


## Tuning Parameters

All parameters are prefixed with `CC3_`:

| Parameter | Description | Default |
|---|---|---|
| `GUESS_R/P/Y` | Initial inertial estimate per axis | 0.03 / 0.02 / 0.01 |
| `AH_MIN_R/P/Y` | Minimum inertial limit per axis | 0.01 |
| `AH_MAX_R/P/Y` | Maximum inertial limit per axis | 0.1 |
| `L_SLIDING_R/P/Y` | Sliding surface lambda gain per axis | 0.87 / 0.73 / 0.45 |
| `K_ROLL/PITCH/YAW` | Kd gain per axis | 0.045 / 0.045 / 0.023 |
| `P_ROLL/PITCH/YAW` | Adaptation rate per axis | 0.08 / 0.08 / 0.05 |

## Repository Structure

```
Ardupilot_AdaptiveController/
├── ardupilot/                          # ArduPilot submodule
├── patches/
│   └── controller.patch                # Git diff of all changes to ardupilot/
├── scripts/
│   └── setup.sh                        # One-step setup script
└── src/                                # Custom source files (overlaid onto ardupilot/)
    ├── libraries/
    │   ├── AC_CustomControl/           # ArduPilot backend integration
    │   │   ├── AC_CustomControl_Adaptive.cpp/h
    │   │   ├── AC_CustomControl_config.h
    │   │   └── LogStructure.h
    │   ├── AC_CustomControl_Adaptive/  # Controller algorithm
    │   │   └── Custom_Att_Controller.cpp/h
    │   └── AP_Logger/                  # Modified log structure
    │       └── LogStructure.h
    └── Tools/
        └── ardupilotwaf/               # Enable adaptive control integration
            └── boards.py
```

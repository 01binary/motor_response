# DC Motor Characterization Tool

The `sample` utility in this package will help you record the response of a DC motor to a sequence of step inputs so that the resulting data can be used to mathematically model the motor and determine the PID gains which can be used to control it.

Once the PID gains are determined, the `tune` utility will let you use the same fixture to send commands to the motor and verify it responds as expected.

## Background

Closed loop control of DC motors can be done with a PID controller, which is configured with three gain constants it's named after: *proportional*, *integral*, and *derivative*.

* **Proportional gain** specifies how much effort to apply in response to a position *error* (the difference between current and desired position). This could also be thought of as a spring "stiffness" constant used when physically simulating springs and rubberbands.
* **Integral gain** specifes how much effort to add when the error does not decrease quickly enough (i.e. this is based on the "past" history). This scaling factor is applied to the sum of the error that keeps increasing over time.
* **Derivative gain** specifies how much effort to subtract based on a "future" prediction of what will happen to the motor position. This could be thought of as a spring "damping" constant used to quiet the springs from continuosly oscillating when they reach the target position, overshoot, then overshoot again on the way back, and so on. This scaling factor is applied to the slope of the curve representing the motion of the motor, which is how it's able to "predict" that a motor will overshoot the desired position and start decreasing the effort before it happens.

These three constants could be tuned by hand, simply by trying different values to find a combination that satisfies a given application.

A more scientific way of doing this is to "characterize" the DC motor with a *linear transfer function* to approximate it mathematically, and then use the function to find PID gains that will achieve the desired response.

> For anyone interested in theory, the transfer function is a laplace transform of a differential equation that describes what the motor will do when current is applied.

While capturing data to characterize a motor requires running it at different velocities and recording the results, tuning it can be done virtually in MATLab Control Toolbox, where you can adjust stiffness and damping parameters and watch what would happen to the motor position graphed over time.

Tuning a running motor while adjusting the PID gains is far more stressful to the motor, as it could cause high-frequency oscillation (the motor being spun really hard forward, then really hard back again with no break in between, over and over again).

## Process

To characterize the motor:

* Install MATLab with System Identification Toolbox and Control System Toolbox
* Setup a fixture with a motor connected to an H-bridge driver that can receive PWM commands
* Attach an absolute encoder to the motor shaft to measure the position
* Configure the `sample` utility with a list of commands to execute on the motor (i.e. spin at this velocity, spin at another velocity, stop, then spin the other way at another velocity)
* Run the `sample` utility to create a `.csv` file recording the velocity and position over time
* Import the `.csv` file into MATLab using **Import Data** command as *column vectors*
* Run **System Identification** App in MATLab
* Choose the imported position and velocity streams
* Estimate using a *Discrete Transfer Function*
* Export the resulting linear transfer function to MATLab workspace
* Run **PID Tuner** App in MATLab
* Import the linear transfer function from workspace
* Choose a PID Controller type
* Drag the *Response Time* (the spring stiffness) and the *Transient Behavior* (the spring damping) sliders and observe changes predicted to the motor position over time
* Record the resuling *Controller Parameters* (the Proportional, Integral, and Derivative gains)

To tune the motor:

* Configure the controller parameters and the goal tolerance in `pid.yaml` configuration file in this package
* Launch the `tune` tool which will listen for motor position commands and send velocity commands to the motor to achieve the commanded position by applying the PID gains calculated in the previous step.

## Sample

The sampling utility can be launched like this:

```
roslaunch motor_response launch/sample.launch
```

Configuration settings are in `/config/settings.yaml` and `/config/steps.yaml`.

## Tune

The tuning utility can be launched like this:

```
roslaunch motor_response launch/tune.launch
```

Configuration settings are in `/config/settings.yaml` and `/config/pid.yaml`.

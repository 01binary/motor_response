![logo](./logo.svg)

# DC Motor Characterization Tool

If you are using DC motors which can only be controlled by PWM velocity commands rather than sophisticated servos with built-in controller that accepts position commands, you will need a way to control them.

The `sample` utility in this package will help you record the response of a DC motor to a sequence of step inputs so that the resulting data can be used to mathematically model the motor and determine the PID gains which can be used to control it.

Once the PID gains are determined, the `tune` utility will let you use the same fixture to send commands to the motor and verify it responds as expected, while changing PID gains on the fly.

> Both utilities require an installation of ROS (Robot Operating System) but the advantage is that both step input and test commands can be sent directly from 3D simulation running in **RViz**, enabling you to move the simulated robot arm and watch how the real thing responds.

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

To characterize the motor using a Discrete or Continuous Linear Transfer Function:

* Install MATLab with System Identification Toolbox and Control System Toolbox
* Setup a fixture with a motor connected to an H-bridge driver that can receive PWM commands
* Attach an absolute encoder to the motor shaft to measure the position
* Configure the `sample` utility with a list of commands to execute on the motor (i.e. spin at this velocity, spin at another velocity, stop, then spin the other way at another velocity)
* Run the `sample` utility to create a `.csv` file recording the velocity and position over time
* Import the `.csv` file into MATLab using **Import Data** command as *column vectors*
* Run **System Identification** App in MATLab
* Choose the imported position and velocity streams
* Estimate using a *Discrete Transfer Function* or *Continuous Transfer Function*
* Poles and Zeros can be modified to get a better fit
* Export the resulting linear transfer function to MATLab workspace
* Run **PID Tuner** App in MATLab
* Import the linear transfer function from workspace
* Choose a PID Controller type
* Drag the *Response Time* (the spring stiffness) and the *Transient Behavior* (the spring damping) sliders and observe changes predicted to the motor position over time
* Record the resuling *Controller Parameters* (the Proportional, Integral, and Derivative gains)

Alternatively, if using a Non-Linear characterization (called `nlarx1` in this example), it can be linearized as follows in MATLab:

```
[x,u] = findop(nlarx1, 'steady', 1, NaN);
sys = linearize(nlarx1, u, x)
```

Then the above steps can be followed starting from "Run PID Tuner".

To tune the motor:

* Configure the controller parameters and the goal tolerance in `pid.yaml` configuration file in this package
* Launch the `tune` tool which will listen for motor position commands and send velocity commands to the motor to achieve the commanded position by applying the PID gains calculated in the previous step.

## Install

```
cd ~/catkin_ws/src
rosdep install -y --from-paths . --ignore-src --rosdistro noetic
```

## Build

To build the project with ROS installed, execute the following within catkin workspace:

```
catkin_make
```

The analog read and write duties are handled by an Arduino ROS node in `./src/analog.ino`.

To build it, first generate ROS message headers for Arduino:

```
sudo apt-get install ros-${ROS_DISTRO}-rosserial-arduino
sudo apt-get install ros-${ROS_DISTRO}-rosserial

rosrun rosserial_arduino make_libraries.py <Arduino libraries path>
```

> The Arduino libraries are usually in `~/Arduino/libraries`. If you installed Arduino IDE as a *snap*, you could also try looking in `~/snap/arduino`.

Then open `analog.ino` in Arduino IDE, build, and upload to Arduino connected to your PC.

This node will listen on `/pwm` ROS topic for PWM commands and publish analog readings to `/adc` ROS topic.

## Sample

The sampling utility can be launched like this:

```
roslaunch motor_response sample.launch
```

Configuration settings are in `/config/joint.yaml` and `/config/sample.yaml`.

> Both `sample.launch` and `tune.launch` will attempt to start the Arduino serial node in this package on `/dev/ttyACM0`, which requires the steps above to be completed (`analog.ino` built and uploaded). See [Build](#build) topic for more information.

## Tune

The tuning utility can be launched like this:

```
roslaunch motor_response tune.launch
```

Configuration settings are in `/config/joint.yaml`.

> Both `sample.launch` and `tune.launch` will attempt to start the Arduino serial node in this package on `/dev/ttyACM0`, which requires the steps above to be completed (`analog.ino` built and uploaded). See [Build](#build) topic for more information.

To send a position command through the PID loop:

```
rostopic pub /tune/command std_msgs/Float64 'data: 0.0' -1
```

To send PWM commands directly to the motor, send a `Pwm` message. The channel index is the same as in your configuration, and value can be between `0` and `255`.

```
rostopic pub /pwm motor_response/Pwm "{ channels: [{ channel: 0, value: 0 }, { channel: 1, value: 0 } ] }" -1
```

To read analog values directly from encoders, echo an `Adc` message. The channel index is the same as the `input` configuration setting:

```
rostopic echo /adc
```

This will output a stream of messages with `12` encoder values, each between `0` and `1024` (`12` bit).

## Settings

Shared settings include:

* `rate` (Hz) - The recording rate for the `sample` tool, or the playback rate for the `tune` tool.
  > A rate of `40` Hz will execute the control logic every `0.025` of a second (divide `1` by `40`).

### Output settings

* `outputTopic` - The ROS topic for sending PWM commands to the bi-directional motor driver that takes LPWM and RPWM digital inputs to run the motor forward or in reverse.
* `lpwm` - The `0`-based output channel on Arduino (see below)
* `rpwm` - The `0`-based output channel on Arduino (see below)

  The following channels are available:
  - channel `0` - `~D3` (digital PWM pin `3`, at `980` Hz)
  - channel `1` - `~D11` (digital PWM pin `11`, at `980` Hz)
  - channel `2` - `~D5` (digital PWM pin `5`, at `490` Hz)
  - channel `3` - `~D13` (digital PWM pin `13` at `490` Hz)

* `minPwm` - The minimum PWM pulse width (`0` - `255`). Set this to the minimum pulse width that will still turn the motor with the load attached from a stand-still position. The application will still send `0` to stop the motor, but it will not send anything between `0` and this value, jumping straight to the minimum PWM.
* `maxPwm` - The maximum PWM pulse width (`0` - `255`). This can be used to restrict the maximum pulse width that can be sent to the motor driver.
* `minVelocity` - The floating point value mapped to the minimum PWM.
* `maxVelocity` - The floating point value mapped to the maximum PWM.

  > Mapping a digital pulse width value to an abstract "velocity" enables you to specify velocity in the most convenient units for your application like radians per second or meters per second. You can measure how fast the motor is moving at certain min and max PWM values to create this mapping between pulse width and real-world velocity.

### Input Settings

* `inputTopic` - The ROS topic for receiving the readings from the absolute encoder. This also uses the Arduino serial node implemented by the `analog` module in this package. Readings are received directly as digital pin values between `0` and `1024`
* `input` - The input channel (pin where the absolute encoder position terminal is attached). The channels are mapped as follows on the Arduino:
  * channel `0` - `A0` ADC pin
  * channel `1` - `A1` ADC pin
  * channel `2` - `A2` ADC pin
  * channel `3` - `A3` ADC pin
  * channel `4` - `A4` ADC pin
  * channel `5` - `A5` ADC pin
  * channel `6` - `A6` ADC pin
  * channel `7` - `A7` ADC pin
  * channel `8` - `A8` ADC pin
  * channel `9` - `A9` ADC pin
  * channel `10` - `A10` ADC pin
  * channel `11` - `A11` ADC pin
* `minReading` - The minimum encoder reading to clamp the values to (`0` readings stay `0`)
* `maxReading` - The maximum encoder reading to clamp the values to.
* `minPos` - The minimum floating-point value to report when the encoder reads `minReading` (when the encoder reports `0` the position is still `0`)
* `maxPos` - The maximum floating-point value to report when the encoder reads `maxReading`.

> The encoder position is mapped from digital signal values (`minReading` to `maxReading`) to abstract units (`minPos` to `maxPos`) to make it simple to interpret encoder position in radians or meters.

* `threshold` - The minimum distance between readings to consider them different when smoothing the digitized analog signal from the encoder
* `average` - The number of digital samples to average (moving average filter on top of the threshold filter).

> The analog readings from the absolute encoder pass through a *histogram* filter which organizes the samples into buckets as long as they are all within the specified threshold of each other. Then it picks the bucket with the highest sample count as the "current" value of the encoder, and further averages this value across the specified number of samples. This has the effect of stabilizing the jitter of analog readings, and it's done with much simpler logic than, for example, a Butterworth filter.

### Control Settings

Rather than requiring you to write a sequence of velocity commands to send to the motor (which is still a primary way to use this tool) the `sample` tool can listen to the specified trajectory `controlTopic`, find a velocity commanded for a particular `joint` in the robot arm, and then play back the whole trajectory on the motor while recording what the motor is doing.

This lets you move a simulated robot arm in RViz complete with inverse kinematics, and then when you click Execute on the trajectory it will get sent to the motor. Depending on your application, it could be quite useful to sample the motor responding to real trajectories that will get sent when your robot is working rather than a sequence of random step inputs.

Likewise, when testing the PID values calculated by MATLab the `tune` tool will listen to the same `controlTopic` and execute the trajectories it receives on the motor, this time reading both the commanded position and velocity, and passing them through a PID controller to generate the pulse width commands for the motor driver.

### Recording

The output from the `sample` tool will be continuously written to the `.csv` file specified by `log` setting, with the following columns:

| time (sec) | velocity | LPWM | RPWM | position | reading |
|-|-|-|-|-|-|

Both the raw digital signals (`reading`, `LPWM`, `RPWM`) and the abstract mapped values (`velocity` and `position`) are recorded.

### Example Configuration

Here's an example of configuring a "base" non-continuous revolute joint of a robot arm for sampling of its motor response. Let's say the URDF for this joint looks like this:

```
<joint name="base" type="revolute">
    <origin rpy="1.5708 0 0" xyz="0 0 0.069" />
    <axis xyz="0 1 0" />
    <parent link="base_link" />
    <child link="shoulder_link" />
    <limit effort="100" velocity="1.0" lower="-1.4929" upper="1.4929" />
    <dynamics damping="0.7" friction="1.4" />
</joint>
```

In our application, we observe that the hollow shaft encoder attached to the motor shaft reads the value of `293` when the robot arm is rotated all the way left, and `790` when the arm is rotated all the way to the right (both in range of `0` to `1024`, the readings from Arduino analog pins).

This maps to minimum rotation of `-1.4929` radians to `1.4929` radians according to the URDF above, therefore we set the input settings as follows:

```
minReading: 293
maxReading: 790
minPos: -1.4929
maxPos: 1.4929
```

Next we observe that the motor (with the load attached) will not move if the pulse width sent to its driver is anything less than `64` (out of `255`). It moves at maximum velocity of `255` just fine.

This maps to the `0.0` to `1.0` velocity on the URDF joint, so that when we command "maximum" velocity we get a pulse width of `100%` sent to the motor, and when we command the "minimum" velocity we get a pulse width of `25%` because `64` is `25%` of `255`. The `0` velocity will still get us `0` pulse width (zeros always map to each other) so that the motor can be stopped.

```
minPwm: 64
maxPwm: 255
minVelocity: 0.0
maxVelocity: 1.0
```

Finally, we run the Arduino analog node and watch the digitized values from the encoder. Let's say we notice that when the encoder is totally still, the values are no more than `8` units apart from each other. When the encoder starts to move, the difference quickly jumps to something greater than `8`. We then set this as the filter `threshold`.

Finally, we set a small enough `average` value that there is no delay in reading the encoder (it can be read at least a few times a second) while the final output is more stable than if we didn't average at all.

Set the `rate` to the same frequency you plan to run your robot at in the real application.

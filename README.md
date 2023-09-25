# Motor Response

## Configuration

	- pwm topic
	- sensor channel
	- motor lpwm channel
	- motor rpwm channel
	- max velocity
	- min velocity
	- min pwm
	- max pwm

Can read CSV file or its own configuration for commands to send:

	velocity, duration

## Published Topics
	
Publishes to PWM output motor commands

## Subscribed Topics
	
Subscribes to PWM input for sensor reading

## Action Server

When in "play mode" (action server), it will continuously read sensors and output sensor time stamps.

It will also execute all commands in the file, waiting specified delay time between.

When it executes all commands, it stops.
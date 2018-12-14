# Gyroscopic Car Simulator

## Sprint One:
The first sprint used parts from our class meant for Lab 3, and re-purposed them as a test-bed for our initial work.
Using an Arduino Uno and a Flysky transmitter and receiver, I made a simple RC car setup.
The Arduino talked to the Flysky receiver through its serial connection, which posed an interesting challenge.
The serial connection could either be used to connect to the receiver or to communicate though USB to the computer, so debugging wasn't possible though normal means when testing the Flysky receiver.
The solution was to use a Morse code library in combination with an Android app called [Morse Code Agent](https://play.google.com/store/apps/details?id=com.erdatsai.morsecodeagent) which can decode Morse code from lights using the phone camera.

![Using Morse code to find out Flysky transmitter axis ranges](media/morse-serial.png){:class="img-responsive"}

[![Video of transmitter changing LED brightness](http://img.youtube.com/vi/CWW765NBaEE/0.jpg)](http://www.youtube.com/watch?v=CWW765NBaEE "Flysky Controls")

[![Video of RC car](http://img.youtube.com/vi/5P076bGsT1k/0.jpg)](http://www.youtube.com/watch?v=5P076bGsT1k "Flysky RC Car")


## Sprint Two:
The second sprint I focused on making a Python-powered physics simulator for the car.
The simulator uses [pybullet](https://pybullet.org) with an [URDF](http://wiki.ros.org/urdf/XML) model for the car.
The early simulator was for sanity-checking simple physics concepts, as our math seemed sketchy.
This sanity-check served useful, as our torque calculations were wrong, and the wheels were spinning in the wrong direction.

Writing the URDF file was more work than I wanted it to be, especially if I was trying to change parameters of the locations of wheels, as the XML format repeats the same numbers throughout the file.
I started creating a python program to automate this process, allowing for quicker iteration on parameters.


## Sprint Three:
I switched contexts back to trying to work on the physical robot, and worked with the team to make a better test setup.
The setup we were using was based on suspending the car from a string in a PVC stand, which posed a problem because the suspension point was above the center of mass (COM).
I threaded a rod through the vehicle below the COM to naturally destabilize the vehicle.

![Robot during sprint 3](media/testing-robot.jpg){:class="img-responsive"}

Then an attempt at tuning a PID loop was made, relatively unsuccessfully.
The motors had trouble spinning up fast enough to fully counteract the effects of gravity on the see-saw setup, and quickly reached saturation, which was undesired.
Because of the motors we used, our saturation speeds were absurdly high, around 8000 RPM when measured by a tachometer (not seen).

![Tachometer reading after a test](media/tachometer.jpg){:class="img-responsive"}

[![Video of car spinning up motors](http://img.youtube.com/vi/asY8iQv4Igg/0.jpg)](http://www.youtube.com/watch?v=asY8iQv4Igg "Slow Motion - Motors Spinning Up")


## Last Minute Work
I went all-in on work on the simulator to try to create a control algorithm for the robot.
My intention was to create an algorithm that wouldn't overshoot stabilization, by solving the dynamics problem using pybullet.

pybullet's documentation was adequate, but there were some undefined behaviors in how the `calculateInverseDynamics` function that slowed my progress significantly.
I switched to trying to understand linearizing the system.
I was able to generate jacobians that represented the system when actuating the wheels, but I ran out of time.
In hindsight, I spent too much time trying to get pybullet to do the hard work for me, and didn't spend enough time learning the mathematics behind what I was trying to accomplish.
I believe trying to rewrite the PID system and tuning it using the simulator would have been the best way to get to a reasonable set of parameters quickly.

In addition to the physics calculations, I completed a version of an URDF generator for the robot that allowed me to quickly test a version of the car that had two wheels on separate 45° axes.

![Picture of simulator working with an interesting wheel configuration](media/simulator.png){:class="img-responsive"}

# Autonomous Fuzzy Control and Navigation of Quadcopters #

This project addresses the design and the evaluation of an evolving autonomous
fuzzy controller applied to the stabilization and the navigation of a
quadcopter.

## References ##

The fuzzy controller used in this project is an implementation of the
Self-Evolving Parameter-Free Rule-Based Controller (SPARC) - an adaptive
approach introduced in (Sadeghi-Tehran et al., 2012) - where no initial control
rule or parameters are required prior to the initialization of the system.

Details about the control theory involved, implementation, evaluation of
results and comparison to classical controllers can be found in [our article](
http://www.sciencedirect.com/science/article/pii/S2405896316302889), published
in the conference proceedings of [IFAC ICONS
2016](http://icons2016.univ-reims.fr/).

## Demonstration Video ##

The following video shows the application of our controllers in the
stabilization and the navigation of a quadcopter in a simulated environment.

The vehicle starts with an empty set of rules, what means that it doesn't even
know how to balance itself in the beginning of the mission. The controllers
need to adapt and evolve in real time in order to minimize the error between
the performed path and a predetermined reference.

Click the gif below to see the full video on youtube.

[![quad_gif](https://cloud.githubusercontent.com/assets/10624503/20035289/172f9ac8-a3c5-11e6-82ac-632131724891.gif)](https://www.youtube.com/watch?v=rIEcj6SDO7k)

## Dependencies ##

The main Python application and the V-REP robotics simulator Lua scripts have
been tested solely on Mac OS X 10.10/10.11. However, since the main application
has been written to be system independent, and V-REP runs also in
Linux/Windows, the transition to these other operating systems should be
smooth. 

    Working implementation 
    * Python 2.7 (Main application)
    * V-REP (Robotics simulator)

    Not tested
    * Matlab (Quadcopter model and controller)

    Future Support
    * Gazebo 7.1 (Robotics simulator)

## Repository Organization ##

Most of the working code has been written in Python 2.7, except for the V-REP
scripts that have been written in Lua and the future Gazebo plugins that will
be written in C++. A matlab quadcopter model and a matlab fuzzy controller have
also been written for preliminary tests, but they have not been tested further
or documented since then.

The repository has been organized as follows:

[TODO]

## Running the simulation ##

To run the simulation, you can choose to open one of the sample V-REP worlds
that we provide in vrep/worlds or create your own by simply importing the
provided quadcopter V-REP model from vrep/models/quadcopter.model.

[TODO]


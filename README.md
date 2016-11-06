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

A quick demonstration video showing the result of the application of the
evolving fuzzy controllers to the navigation and stabilization of our
quadcopter in a simulated environment.

The quadcopter starts with an empty set of rules, what means that it does not
know how to balance and to navigate itself in the beginning of the mission. The
controllers need to adapt and evolve in real time in order to minimize the
error between the quadcopter path and a predetermined reference.
Click the gif below to see the full video on youtube.

[![quad_gif](https://cloud.githubusercontent.com/assets/10624503/20035289/172f9ac8-a3c5-11e6-82ac-632131724891.gif)](https://www.youtube.com/watch?v=rIEcj6SDO7k)


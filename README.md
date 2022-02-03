# Malloy Aeronautics GNC Applications

![iq](docs/imgs/MA.jpeg)

This package is forked from Intellegent Quads github page. It contains a collection of software designed to develop GNC features for drones produced by Malloy Aeronautics.

---

## Guidance Navigation and Control Functions

### gnc_function.hpp / py_gnc_functions.py
This page is forked by intellegent quads and updated
The intelligent quads gnc_functions are collection of high level functions to help make controlling your drone simple. You can find functions for interpreting state estimation, commanding waypoints, changing modes and more. The documentation for using these functions is shown below. 

[gnc_functions.hpp documentation](https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/GNC_functions_documentation.md)

[py_gnc_functions.py documentation](docs/py_gnc_functions.md)

---

## Example Code (C++)

### avoidance_sol.cpp
Example obstacle avoidance program utilizing the potential field method.

### gnc_tutorial.cpp
Simple waypoint mission that commands a drone to fly a square pattern. 

### sr_sol.cpp 
Simple search and rescue program that will fly a search pattern until yolo detects a person. This will trigger the drone to land to deliver the rescue supplies. 

### subscriber_sol.cpp
Example program showing how to use a ROS subscriber to take input into your drone's guidance node.


### [*Python Version of Example code.*](docs/py_gnc_functions.md)

## Related Repos

[iq_sim](https://github.com/Intelligent-Quads/iq_sim) - Repo hosting the simulation worlds designed to help develop drone gnc missions




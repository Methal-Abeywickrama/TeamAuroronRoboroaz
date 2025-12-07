---
description: create a file to debug the robot
---

create a new header or a suitable file to debug a pid control value sets in webot epuck . include it with these functions that runs when called - 
1. a function to call , when called it will edit the text file debug report with all the necessary data about the robot in each time stamp.look the the already existing debug report file that takes suitable info to get to know the needed info in debug file. analyize the debug info file and a create a artifact containing two parts ,1- what happend to the robot ,which is decided by that analzing 2- possible fixes suggestion ,made by thorough investigation .
2.another function that allows the robot to simulate many possible combined value sets of kp,kd,ki and other important k values by the use of supervisor mode in webots- when user ask to call and run the pid value deciding function ,initiate function and  ask user input for how to create the data sets and what kind of a simulation to do(what user needs) ,then depending on that information edit the function as suited to do all that simulation work. after a simulation run of possible configurations of k values write the results to a txt file ,and highlight the best results that completed the simulation after the end of it.

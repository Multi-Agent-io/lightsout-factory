# Lightsout Factory

![Factory Image](./misc/factory.png)

"Lightsout Factory" is the ROS based project of [Robonomics platform][db1] by Airalab. This project is model of an economically autonomous plant. Factory can produce different types of products depending on the current market situation. 

Example of how it works you can [find here][yu1].

## Description

The factory contains a warehouse of raw materials and a warehouse of goods. Having 4 produce lines, factory is capable of modeling manufacturing process of real 4-line factory, each line produces special type of production, giving an opportunity to set environment for economical experiments.

## Goal

The purpose of the experiment is to create an algorithm of factory which monitors "**supply and demand**" situation on market and adaptively produces high-demand goods at optimal rate. As result, the algorithm should minimize both overproduction of goods and their shortage.

## Lightsout crates
Structure of **Lightsout Factory** followed:

- app - Software part of project. Represents a python wrapper under ModBus protocol and ROS messages. For more information checks [ReadMe][db2]. 
- firmware - Firmware part of project. Contain all necessary files for SIEMENS controllers. For more information checks [ReadMe][db3].
- launch - Launch files. They need to start ModBus server on ubuntu. For more information checks [ReadMe][db4].

## Requirements
- Ubuntu 20.04. Available to install server version on RPi or LattePanda.
- ROS Noetic. Install [here][db5]
- SIEMENS controllers - Siemens PLC S7-1200 and 6 additional PLC blocks (SM-1223)




[db1]: <https://robonomics.network/>
[db2]: <app/README.md>
[db3]: <firmware/README.md>
[db4]: <launch/README.md>
[db5]: <http://wiki.ros.org/noetic/Installation>
[yu1]: <https://www.youtube.com/playlist?list=PL009YD81fX3LoJHFsSpESREtFYq7dqAUl>
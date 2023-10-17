# Suplementary material for manuscript submitted to Applied Artificial Intelligence:
## Autonomous behavior selection for self-driving cars using probabilistic logic factored Markov decision processes

This repository contains the required software to reproduce the results presented in the manuscript paper "Autonomous behavior selection for self-driving cars using probabilistic logic factored Markov decision processes". Such results can be summarized as follows:

* Experiment 1: Test in an environment with 5 static cars
* Experiment 2: Test in an environment with 10 static cars
* Experiment 3: Test in an environment with 5 moving cars

Methodology details are described in the contributed paper. 

## Requirements:

* Ubuntu 20.04
* ROS Noetic (http://wiki.ros.org/noetic/Installation/Ubuntu)
* Webots 2022a (https://github.com/cyberbotics/webots/releases/download/R2022a/webots_2022a_amd64.deb)

## Installation

Note: It is assumed that Ubuntu OS, ROS platform and Webots simulator are already installed. 

* $ cd
* $ git clone https://github.com/mnegretev/Autonomous_Behavior_Selection_2023
* $ cd Autonomous_Behavior_Selection_2023/catkin_ws
* $ catkin_make
* $ echo "source ~/Autonomous_Behavior_Selection_2023/catkin_ws/devel/setup.bash" >> ~/.bashrc
* $ source ~/.bashrc

## Testing

To test each experiment:

* $ roslaunch aai_2023 experiment1_5_cars.launch
* $ roslaunch aai_2023 experiment1_10_cars.launch
* $ roslaunch aai_2023 experiment2.launch

In each experiment you should see a simulator like this:

![Screenshot from 2023-09-27 19-20-24](https://github.com/hector-aviles/ICRA2024/assets/17507149/bebb5033-5092-4ba6-b66f-644024a79fd2)

## Video


https://github.com/mnegretev/Autonomous_Behavior_Selection_2023/assets/17507149/209a301f-db8b-4e0e-a4af-6ea34c47536a


## Contact

Héctor Avilés<br>
havilesa@upv.edu.mx <br>
Marco Negrete<br>
marco.negrete@ingenieria.unam.edu


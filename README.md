# EE346_Final_Lab
## Contributors
Xinyi Zhou, Biao Wang
## Introduction
EE346, the course of _Mobile Robot Navigation and Control_ delivers to us both theoretical knowledge and practical experiences in autonomous driving. The final project takes the form of a competition where we have options to complete tasks including autonomous navigation, lane following, and target search (Aruco marker detection) within the environment shown below. Each task earns some points for our team. 
In this report, we will first identify the tasks we have completed, then describe the system we utilized for algorithm developing, after which provide step-by-step explanations for our algorithm, and finally conclude our performance in the competition.
![./Figures/env.jpg](https://github.com/CindyChow123/EE346_Final_Lab/blob/main/Figures/env.jpg)
## Accompolished Tasks
### Task1: Point Navigation
There are in total four points in the map. P1 is the starting point and ending point of each lap while P2, P3, and P4 are points each worthy of 10 points. During the task of lane following, we managed to navigate our robot to each point to earn points.
### Task 2: Lane Following
Our robot is asked to follow the black lane line to complete the lane following task after launching itself from P1. Finishing this task can give us 20 points with a 1-point penalty for cutting each corner. We designed an algorithm based on images received by the robot to complete the task.
## Round Route
### Round 1
In round 1, we did lane following and navigation. Each lap follows this order: P1->Lane Following->P2->Lane Following->P3->P4->P1.
![](https://github.com/CindyChow123/EE346_Final_Lab/blob/main/Videos/round1.gif)
### Round 2
In round 2, we only did lane following. The whole traverse follows this order: P1->Lane Following->P1.
![](https://github.com/CindyChow123/EE346_Final_Lab/blob/main/Videos/round2.gif)

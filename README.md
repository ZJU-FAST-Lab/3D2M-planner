## Quick Start
Compiling tests passed on ubuntu 18.04 and 20.04 with ros installed. You can just execute the following commands one by one.
```
git clone https://github.com/ZJU-FAST-Lab/3D2M-planner.git
cd 3D2M-planner/planner
catkin_make -DCMAKE_BUILD_TYPE=Release -j1
source devel/setup.bash
sh exp1.sh
```
Then you can use the 3D goal tool to operate.
If you find this work useful or interesting, please kindly give us a star ⭐, thanks!😀

## 3D2M-planner
This is the related repository of paper \textbf{Towards Efficient Trajectory Generation for Ground Robots beyond_2D Environment}. The repository contains the planner ros workspace and a unity project for verification. The planner enables global planning of ground robots in complex, multi-layered structural environments and allows joint planning of additional vertical degrees of freedom.


\textbf{Video Links:} \href{https://www.bilibili.com/video/BV1bt4y1P7iD/?vd_source=88d7fb7cf7a82c415745fa20646ab137}{bilibili} (for Mainland China)

## Standard Compilation

\textbf{Requirements:} ubuntu 18.04 or 20.04 with ros-desktop-full installation.

\textbf{Step 1.} 

\textbf{Step 2.} Clone the code from github.
```
git clone https://github.com/ZJU-FAST-Lab/3D2M-planner.git
```
\textbf{Step 3.} Compile.
```
cd 3D2M-planner/planner
catkin_make -DCMAKE_BUILD_TYPE=Release -j1
```
If you don't add "-j1", it may report an error while compiling, just compile it again.

\textbf{Step 4.} Run.
There are four quick start scripts：\textbf{exp1.sh} , \textbf{exp2.sh}, \textbf{exp1_unity.sh}, \textbf{exp2_unity.sh}
In \textbf{exp1.sh}, the robot is a four-wheel differential car. If you are using \textbf{unity} to simulate, please run \textbf{exp1_unity.sh}.
In \textbf{exp2.sh}, the robot is a differential car with a one-degree-of-freedom arm. Similarly, run \textbf{exp2_unity.sh} instead when using \textbf{unity}.

For example,
```
source devel/setup.bash
sh exp1.sh
```

To be continued..

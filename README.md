# ROS Kinectic Workspace

[![Build Status](http://img.shields.io/travis/badges/badgerbadgerbadger.svg?style=flat-square)](https://travis-ci.org/badges/badgerbadgerbadger) [![Coverage Status](http://img.shields.io/coveralls/badges/badgerbadgerbadger.svg?style=flat-square)](https://coveralls.io/r/badges/badgerbadgerbadger) [![MIT License](https://img.shields.io/github/license/iamrajee/roskinectic_src.svg)](http://badges.mit-license.org) [![GitHub Issues](https://img.shields.io/github/issues/iamrajee/roskinectic_src.svg)](https://github.com/iamrajee/roskinectic_src/issues) [![GitHub Pull Requests](https://img.shields.io/github/issues-pr/iamrajee/roskinectic_src.svg)](https://github.com/iamrajee/roskinectic_src/pulls) [![Gitter](https://badges.gitter.im/iamrajee-ROS/community.svg)](https://gitter.im/iamrajee-ROS/community?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge) [![Join our Slack Workspace](https://img.shields.io/badge/Slack%20Workspace-roboticsclubiitpkd.slack.com-blue.svg?logo=slack&longCache=true&style=flat)](https://roboticsclubiitpkd.slack.com)
<!---
[![Gem Version](http://img.shields.io/gem/v/badgerbadgerbadger.svg?style=flat-square)](https://rubygems.org/gems/badgerbadgerbadger)
[![first-timers-only](https://img.shields.io/badge/first--timers--only-friendly-blue.svg)](https://www.firsttimersonly.com/)
--->

This ROS kinectic workspace src folder, which was created on Ubuntu 16.04. Here I worked on ROS1 projects like 2d & 3D SLAM, Motion Planning, SWARM of drone, RL based drone, Surveilling Robot etc.
![](https://github.com/iamrajee/resume/blob/master/category_wise_gif/ROS1_Kinetic_src/robots2.png)
<br/><br/>

## Table of content
- [Installation](https://github.com/iamrajee/roskinectic_src#installation)
- [Package description](https://github.com/iamrajee/roskinectic_src#package-description)
- [Helper scripts](https://github.com/iamrajee/roskinectic_src#helper-scripts)
- [Team](https://github.com/iamrajee/roskinectic_src#team)
- [Contributing](https://github.com/iamrajee/roskinectic_src#contributing)
- [FAQ](https://github.com/iamrajee/roskinectic_src#faq)
- [Support](https://github.com/iamrajee/roskinectic_src#support)
- [License](https://github.com/iamrajee/roskinectic_src#license)
- [Acknowledgments](https://github.com/iamrajee/roskinectic_src#acknowledgments)
<!--- - [xyz](link) --->

<!---
---
## Maintainer
|  |  |
| :---: | --- |
| ![](https://avatars0.githubusercontent.com/u/25712145?s=200&v=3) | Name : Rajendra Singh<br/> Email  : singh.raj1997@gmail.com<br/> Web    : https://iamrajee.github.io/<br/> LinkedIn    : https://www.linkedin.com/in/rajendra-singh-6b0b3a13a/<br/> Twitter: <a href="https://twitter.com/i_am_rajee" target="_blank">`@i_am_rajee`</a> |
|  |  |
--->

---

## Installation

> All the `code` required to get started
- #### Prerequisite
    - You should have ROS kinectic on your ubuntu 16.04.
    - All ROS dependency is satisfied.

- #### Clone

    ```
    mkdir your_ros_workspace/
    cd your_ros_workspace/
    git clone https://github.com/iamrajee/roskinectic_src.git
    mv roskinectic_src src      
    catkin_make
    source devel/setup.bash
    ```

- #### Setup
    ```
    cd your_ros_workspace/
    ./refresh.sh
    make
    ```
---


## Package description
<!---
* ## [test_pkg](test_pkg)
    panda pkg contain cpp and py interface for moveit.*

    Terminal 1:
    ```
    $ roscore
    $ TODO
    ```
    Terminal 2:
    ```
    $ TODO
    ```
--->
* ## [depthcamera](depthcamera)
    ![](https://github.com/iamrajee/resume/blob/master/category_wise_gif/ROS1_Kinetic_src/agv.gif)
    ![](https://github.com/iamrajee/resume/blob/master/category_wise_gif/ROS1_Kinetic_src/rtabmap.gif)
    ![](https://github.com/iamrajee/resume/blob/master/category_wise_gif/ROS1_Kinetic_src/3d_map.gif)
    ![](https://github.com/iamrajee/resume/blob/master/category_wise_gif/ROS1_Kinetic_src/rtabmap.png)
    ![](https://github.com/iamrajee/resume/blob/master/category_wise_gif/ROS1_Kinetic_src/sensors.png)
    
* ## [manualcontrol_master](manualcontrol_master)
    ![RAWBOT 2.0](https://github.com/iamrajee/roskinetic_catkin_ws/blob/master/demogif/RAWBOT.gif)
    ![RAWBOT 2.0](https://github.com/iamrajee/resume/blob/master/category_wise_gif/ROS1_Kinetic_src/rawbot.png)
<!---    See full video [here](TODO).
    *In this pkg TODO* --->
    
* ## [lidar](lidar)
    ![](https://github.com/iamrajee/resume/blob/master/category_wise_gif/ROS1_Kinetic_src/gmapping_mapping.gif)
    ![](https://github.com/iamrajee/resume/blob/master/category_wise_gif/ROS1_Kinetic_src/gmapping_navigation.gif)
<!---    See full video [here](TODO).
    *In this pkg TODO* --->

<!---    See full video [here](TODO).
    *In this pkg TODO* --->
    
* ## [ratslam_ros](ratslam_ros)
    ![](https://github.com/iamrajee/resume/blob/master/category_wise_gif/ROS1_Kinetic_src/ratslam.gif)
<!---    See full video [here](TODO).
    *In this pkg TODO* --->
    
* ## [sih](sih)
    ![](https://github.com/iamrajee/resume/blob/master/category_wise_gif/ROS1_Kinetic_src/swarm_drone.gif)
    ![](https://github.com/iamrajee/resume/blob/master/category_wise_gif/ROS1_Kinetic_src/swarm_crowd.gif)
    ![](https://github.com/iamrajee/resume/blob/master/category_wise_gif/ROS1_Kinetic_src/pathplan.png)
<!---    See full video [here](TODO).
    *In this pkg TODO* --->

* ## [uarm](uarm)
    ![](https://github.com/iamrajee/resume/blob/master/category_wise_gif/ROS1_Kinetic_src/uarm_write.gif)
    ![](https://github.com/iamrajee/resume/blob/master/category_wise_gif/ROS1_Kinetic_src/uarm_display.gif)
<!---    See full video [here](TODO).
    *In this pkg TODO* --->
    
* ## [uarm4_moveit_config](uarm4_moveit_config)
    ![](https://github.com/iamrajee/resume/blob/master/category_wise_gif/ROS1_Kinetic_src/uarm_pick.gif)
    ![](https://github.com/iamrajee/resume/blob/master/category_wise_gif/ROS1_Kinetic_src/uarm_moveit_config.png)
<!---    See full video [here](TODO).
    *In this pkg TODO* --->
    
* ## [panda](panda)
    ![](https://github.com/iamrajee/resume/blob/master/category_wise_gif/ROS1_Kinetic_src/cylinder.png)
    ![](https://github.com/iamrajee/resume/blob/master/category_wise_gif/ROS1_Kinetic_src/cylinder_detect.gif)
    ![](https://github.com/iamrajee/resume/blob/master/category_wise_gif/ROS1_Kinetic_src/pickplace.gif)
<!---    See full video [here](TODO).
    *In this pkg TODO*--->
<!---    
* ## [panda_moveit_config](panda_moveit_config)
    ![](https://github.com/iamrajee/roskinetic_catkin_ws/blob/master/demogif/RAWBOT.gif)\
    See full video [here](TODO).
    *In this pkg TODO*
--->
* ## [gps](gps)
    ![](https://github.com/iamrajee/resume/blob/master/category_wise_gif/ROS1_Kinetic_src/gps.gif)
    ![](https://github.com/iamrajee/resume/blob/master/category_wise_gif/ROS1_Kinetic_src/tf.png)
<!---    See full video [here](TODO).
    *In this pkg TODO*--->
    
* ## [drone_training](drone_training)
    ![](https://github.com/iamrajee/resume/blob/master/category_wise_gif/ROS1_Kinetic_src/openai.png)
    ![](https://github.com/iamrajee/resume/blob/master/category_wise_gif/ROS1_Kinetic_src/qlearn.png)
    ![](https://github.com/iamrajee/resume/blob/master/category_wise_gif/ROS1_Kinetic_src/sarsa.png)
    ![](https://github.com/iamrajee/resume/blob/master/category_wise_gif/ROS1_Kinetic_src/compare3.png)
    
<!---  
* ## [drone_training](drone_training)
    ![](https://github.com/iamrajee/roskinetic_catkin_ws/blob/master/demogif/RAWBOT.gif)\
    See full video [here](TODO).
    *In this pkg TODO*  
* ## [drone_training](drone_training)
    ![RAWBOT 2.0](https://github.com/iamrajee/roskinetic_catkin_ws/blob/master/demogif/RAWBOT.gif)\
    See full video [here](TODO).
    *In this pkg TODO*    
* ## [drone_training](drone_training)
    ![RAWBOT 2.0](https://github.com/iamrajee/roskinetic_catkin_ws/blob/master/demogif/RAWBOT.gif)\
    See full video [here](TODO).
    *In this pkg TODO*
--->

---
<br/><br/>
# Helper Scripts
> To be run in your_ros_workspace for ease of build, compiling, running in your_ros_workspace.
* ## refresh.sh
    ```
    #!/bin/bash
    source /opt/ros/kinectic/setup.bash
    source install/local_setup.bash
    source install/setup.bash
    clear
    ```
    > It will source the workspace after buiding workspace or after creating new pkg. Run it as `./refresh.sh`

* ## makefile
    ```
    SHELL=/bin/bash
    all:
        make run
    run:
        catkin_make
        bash refresh.sh
    ```
    > It will build the workspace . Run it as `make`

* ## createpkg.sh
    ```
    #!/bin/bash
    cd src/
    catkin create $1
    cd ../
    make
    source refresh.sh
    ```
    > It will create new package . Run it as `./createpkg.sh newpkg_name`

* ## tftree.sh
    ```
    #!/bin/bash
    rosrun rqt_tf_tree rqt_tf_tree
    ```
    > It will  launch the gui to visvualise the tf tree. Run it as `./tftree.sh`

* ## printenv.sh
    ```
    #!/bin/bash
    printenv | grep -i ROS
    ```
    > It will print the ROS related environment variable . Run it as `./printenv.sh`

* ## rosdep.sh
    ```
    sudo rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
    ```
    > It will install dependencies of all pkg in the workspace. Run it in the workspace as `./rosdep.sh`

* ## ssh_into_another_computer.sh
    ```
    #!/bin/bash
    ssh rajendra@rajendra
    ```
    > It will ssh into another system. Useful when using multiple ros masters. Run it as `./rajendra.sh`

---
<br/><br/>
## Team

> Or Contributors/supporters/mentors/guides who helped me out in these projects.
<!---
| <a href="https://github.com/MuskaanMaheshwari" target="_blank">**Muskaan Maheshwari**</a> | <a href="https://www.linkedin.com/in/sachin-rustagi-882b55145/" target="_blank">**Sachin Rustagi**</a> | <a href="https://www.linkedin.com/in/s-m-rafi-911442130/" target="_blank">**S M Rafi**</a> |
| :---: |:---:| :---:|
| ![](https://avatars0.githubusercontent.com/u/18076234?s=200&v=3) | ![](https://avatars0.githubusercontent.com/u/2555224?s=200&v=3) | ![](https://avatars0.githubusercontent.com/u/917816?s=200&v=3)  |
--->
| <a href="https://github.com/MuskaanMaheshwari" target="_blank">**Muskaan Maheshwari**</a> | <a href="https://github.com/wildcat26" target="_blank">**Shruti Umat**</a> | <a href="https://www.linkedin.com/in/amin-swamiprasad-pkd-17732b152/" target="_blank">**Swami Prasad**</a> |
| :---: |:---:| :---:|
| ![](https://avatars0.githubusercontent.com/u/18076234?s=200&v=3) | ![](https://avatars0.githubusercontent.com/u/37037236?s=200&v=3) | ![](https://avatars0.githubusercontent.com/u/917816?s=200&v=3)  |


---

## Contributing

> To get started...

### Step 1

- **Option 1**
    - 🍴 Fork this repo!

- **Option 2**
    - 👯 Clone this repo to your local machine using `https://github.com/iamrajee/roskinectic_src.git`

### Step 2

- **HACK AWAY!** 🔨🔨🔨

### Step 3

- 🔃 Create a new pull request using <a href="https://github.com/iamrajee/roskinectic_src/compare/" target="_blank">`https://github.com/iamrajee/roskinectic_src/compare/`</a>.
---

## FAQ

- **I ran into some issue while running above package, what to do now?**
    - Simply contact me!

---

## Support
Reach out to me for any help!
|  |  |
| :---: | --- |
| ![](https://avatars0.githubusercontent.com/u/25712145?s=200&v=3) | Name : Rajendra Singh<br/> Email  : singh.raj1997@gmail.com<br/> Web    : https://iamrajee.github.io/<br/> LinkedIn    : https://www.linkedin.com/in/rajendra-singh-6b0b3a13a/<br/> Twitter: <a href="https://twitter.com/i_am_rajee" target="_blank">`@i_am_rajee`</a> |
|  |  |

---

## License

[![MIT License](https://img.shields.io/github/license/iamrajee/roskinectic_src.svg)](http://badges.mit-license.org)

- **[MIT license](http://opensource.org/licenses/mit-license.php)**
- Copyright (c) 2019 [Rajendra Singh](https://iamrajee.github.io/).
---

## Acknowledgments

* Hat tip to anyone whose code was used and thanks to everyone who inspired and supported me in this project.

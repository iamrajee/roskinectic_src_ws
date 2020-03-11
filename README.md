# ROS2 Melodic Workspace

[![Build Status](http://img.shields.io/travis/badges/badgerbadgerbadger.svg?style=flat-square)](https://travis-ci.org/badges/badgerbadgerbadger) [![Coverage Status](http://img.shields.io/coveralls/badges/badgerbadgerbadger.svg?style=flat-square)](https://coveralls.io/r/badges/badgerbadgerbadger) [![MIT License](https://img.shields.io/github/license/iamrajee/ws_moveit.svg)](http://badges.mit-license.org) [![GitHub Issues](https://img.shields.io/github/issues/iamrajee/ws_moveit.svg)](https://github.com/iamrajee/ws_moveit/issues) [![GitHub Pull Requests](https://img.shields.io/github/issues-pr/iamrajee/ws_moveit.svg)](https://github.com/iamrajee/ws_moveit/pulls) [![Gem Version](http://img.shields.io/gem/v/badgerbadgerbadger.svg?style=flat-square)](https://rubygems.org/gems/badgerbadgerbadger) [![Gitter](https://badges.gitter.im/iamrajee-ROS/community.svg)](https://gitter.im/iamrajee-ROS/community?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge) [![Join our Slack Workspace](https://img.shields.io/badge/Slack%20Workspace-roboticsclubiitpkd.slack.com-blue.svg?logo=slack&longCache=true&style=flat)](https://roboticsclubiitpkd.slack.com) [![first-timers-only](https://img.shields.io/badge/first--timers--only-friendly-blue.svg)](https://www.firsttimersonly.com/)


This ROS melodic workspace is created on Ubuntu 18.04.
<br/><br/>

## Table of content
- [Maintainer](https://github.com/iamrajee/ws_moveit#maintainer)
- [Installation](https://github.com/iamrajee/ws_moveit#installation)
- [Package description](https://github.com/iamrajee/ws_moveit#package-description)
- [Helper scripts](https://github.com/iamrajee/ws_moveit#helper-scripts)
- [Team](https://github.com/iamrajee/ws_moveit#team)
- [Contributing](https://github.com/iamrajee/ws_moveit#contributing)
- [FAQ](https://github.com/iamrajee/ws_moveit#faq)
- [Support](https://github.com/iamrajee/ws_moveit#support)
- [License](https://github.com/iamrajee/ws_moveit#license)
- [Acknowledgments](https://github.com/iamrajee/ws_moveit#acknowledgments)
<!--- - [xyz](link) --->

---

## Maintainer
|  |  |
| :---: | --- |
| ![](https://avatars0.githubusercontent.com/u/25712145?s=200&v=3) | Name : Rajendra Singh<br/> Email  : singh.raj1997@gmail.com<br/> Web    : https://iamrajee.github.io/<br/> LinkedIn    : https://www.linkedin.com/in/rajendra-singh-6b0b3a13a/ |
|  |  |

---

## Installation

> All the `code` required to get started
- #### Prerequisite
    - You should have ROS2 melodic on your ubuntu 18.04.
    - All ROS dependency is satisfied.

- #### Clone

    ```
    git clone https://github.com/iamrajee/ws_moveit.git
    ```

- #### Setup
    ```
    cd ws_moveit/
    ./refresh.sh
    make
    ```
---


## Package description
* ## [panda](src/panda)
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


* ## [panda](src/panda)
    ![RAWBOT 2.0](demogif/RAWBOT.gif)\
    See full video [here](TODO).
    *In this pkg TODO*


---
<br/><br/>
# Helper Scripts

* ## refresh.sh
    ```
    #!/bin/bash
    source /opt/ros/eloquent/setup.bash
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
--->
| <a href="https://github.com/abhinand4as" target="_blank">**Abhinand A S**</a> | <a href="https://www.linkedin.com/in/sachin-rustagi-882b55145/" target="_blank">**Sachin Rustagi**</a> | <a href="https://www.linkedin.com/in/amin-swamiprasad-pkd-17732b152/" target="_blank">**Swami Prasad**</a> |
| :---: |:---:| :---:|
| ![](https://avatars0.githubusercontent.com/u/18076234?s=200&v=3) | ![](https://avatars0.githubusercontent.com/u/2555224?s=200&v=3) | ![](https://avatars0.githubusercontent.com/u/917816?s=200&v=3)  |


---

## Contributing

> To get started...

### Step 1

- **Option 1**
    - 🍴 Fork this repo!

- **Option 2**
    - 👯 Clone this repo to your local machine using `https://github.com/iamrajee/ws_moveit.git`

### Step 2

- **HACK AWAY!** 🔨🔨🔨

### Step 3

- 🔃 Create a new pull request using <a href="https://github.com/iamrajee/ws_moveit/compare/" target="_blank">`https://github.com/iamrajee/ws_moveit/compare/`</a>.
---

## FAQ

- **I ran into some issue while running above package, what to do now?**
    - Simply contact me!

---

## Support
Reach out to me at one of the following places!

- Website: <a href="https://iamrajee.github.io/" target="_blank">`iamrajee.github.io/`</a>
- Twitter: <a href="https://twitter.com/i_am_rajee" target="_blank">`@i_am_rajee`</a>
- Email  : singh.raj1997@gmail.com
- LinkedIn: at <a href="https://www.linkedin.com/in/rajendra-singh-6b0b3a13a/" target="_blank">`@Rajendra Singh`</a>
---

## License

[![MIT License](https://img.shields.io/github/license/iamrajee/ws_moveit.svg)](http://badges.mit-license.org)

- **[MIT license](http://opensource.org/licenses/mit-license.php)**
- Copyright (c) 2019 [Rajendra Singh](https://iamrajee.github.io/).
---

## Acknowledgments

* Hat tip to anyone whose code was used and thanks to everyone who inspired and supported me in this project.

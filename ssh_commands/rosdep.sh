#!/bin/bash
rosdep install --from-paths src --ignore-src --rosdistro kinetic -y -r
rosdep update

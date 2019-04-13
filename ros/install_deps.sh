#!/bin/bash

bash <(wget -q -O - https://bitbucket.org/DataspeedInc/dbw_mkz_ros/raw/default/dbw_mkz/scripts/sdk_install.bash)
sudo apt update && sudo apt install --reinstall python-catkin-pkg python-catkin-pkg-modules
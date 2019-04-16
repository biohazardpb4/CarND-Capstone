This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

## Conclusion

This was an extremely educational project. I learned all about how to use ROS, the Unity simulator, how to gather training data, how to train an object detection model, and how to deploy this application of ML in the sim. The most challenging parts for me were around tuning the performance of the ROS environment such that it ran smoothly, finding a good computer to train the object detection model on, and gathering training data.

## System Architecture

This project heavily utilizes ROS (Robot Operating System) which is a runtime environment which facilitates service management, communication, and API generation.

The ROS nodes involved are:
- a waypoint loader (loads a set of static known waypoints in the area)
- a waypoint updater (dynamically sets the velocity associated with waypoints to achieve maneuvers like slowing down for a stop light).
- an object detection node (used to detect traffic lights)
- a waypoint follower (smoothly reaches target waypoint velocities specified by the waypoint loader and updater)
- a control node (attempts to minimize control error using PID controllers for steering and acceleration)
- a DBW (Drive By Wire) node which interfaces with the car hardware
- a simulator bridge (allows ROS to talk to the Unity3D simulator)

### Traffic Light Detection

#### Training Methodology

To learn about training an object detector, I read over the following blog posts and repos:
    - blog post: https://medium.com/@anthony_sarkis/self-driving-cars-implementing-real-time-traffic-light-detection-and-classification-in-2017-7d9ae8df1c58
    - the repo for that post: https://github.com/swirlingsand/deeper-traffic-lights/blob/master/object_detection_sim_run.ipynb
When it came to the practicalities of training the object detection model, the following repo really helped me avoid the gotchas:
https://github.com/alex-lechner/Traffic-Light-Classification

#### Data Sets

To train the model used in this project, the data that was gathered by Alex was used. I began collecting and labeling my own data using a combination of the simulator and labelImg, but unfortunately ran out of time.

Steps involved in gathering data
1. downloaded pre-labeled data sets from https://github.com/alex-lechner/Traffic-Light-Classification

Attempted gathering of data from simulator -- I did not complete this:
1. record unlabeled images from simulator (done)
  1. rosrun image_view image_saver _sec_per_frame:=1 image:=/image_color
1. use labelimg to label images (done)
  1. downloaded from here https://github.com/tzutalin/labelImg
1. write Python script to convert to TensorFlow training Examples (unfinished)

#### Training

Steps involved in training
1. Configure the model at traffic_light/ssd_inception_v2_coco.config to use appropriate label map, training data set and validation data set
1. Use the training script provided by https://github.com/tensorflow/models/tree/master/research/ to train the model
1. Freeze the model so that it can be used for inferrence

Gotchas: I attempted several times to train the SSD model on AWS with a GPU instance, but could never produce the magic incantation necessary to get it to run. After many wasted hours, I went back and trained the model on the Udacity sim with the GPU enabled.

#### Model Selection

- As indicated in Alex's Github repo, the TensorFlow object detection API is a good choice for detecting traffic lights. It offers a very declarative way to do transfer learning which saves on a ton of training time: https://github.com/tensorflow/models/tree/master/research/object_detection#tensorflow-object-detection-api


#### Inferrence

- Code from the object detection subproject was modified to do inferrence within ROS

## Installation

Please use **one** of the two installation options, either native **or** docker installation.

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Port Forwarding
To set up port forwarding, please refer to the [instructions from term 2](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77)

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images

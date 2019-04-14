#!/bin/bash

wget http://download.tensorflow.org/models/object_detection/ssd_inception_v2_coco_11_06_2017.tar.gz
tar -zxvf ssd_inception_v2_coco_11_06_2017.tar.gz

# also use 'scp -i ~/car/car-util/car.pem dataset-sdcnd-capstone.zip ubuntu@ec2-52-90-192-175.compute-1.amazonaws.com:~' to get other data

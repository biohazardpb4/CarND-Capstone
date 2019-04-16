Steps involve in training model:
- record ros bag from simulator to gather unlabeled images
  - rosrun image_view image_saver _sec_per_frame:=1 image:=/image_color
- use labellmg to label images
- write Python script to convert to TensorFlow training Examples
- Use TensorFlow object detection API to train a model: https://github.com/tensorflow/models/tree/master/research/object_detection#tensorflow-object-detection-api
- Use code from the object detection subproject to do inferrence
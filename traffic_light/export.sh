#!/bin/bash

python export_inference_graph.py --logtostderr --input_type=image_tensor --pipeline_config_path=./ssd_inception_v2_coco.config --trained_checkpoint_prefix=./train/model.ckpt-2000 --output_directory=train


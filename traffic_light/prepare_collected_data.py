# Based on the implementation at https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/using_your_own_dataset.md
import os
import random
from xml.dom import minidom
import tensorflow as tf
from object_detection.utils import dataset_util

flags = tf.app.flags
flags.DEFINE_string('output_path', '', 'Path to output TFRecord')
FLAGS = flags.FLAGS

def create_tf_example(base, xml_path):

    height = 600 # Image height
    width = 800 # Image width
    filename = '/home/prombo/car/CarND-Capstone/traffic_light/collected_data/{}.jpg'.format(base).encode('utf-8')
    encoded_image_data = None # Encoded image bytes
    with tf.gfile.GFile(filename, 'rb') as fid:
        encoded_image_data = fid.read()
    image_format = b'jpeg'

    xmins = [] # List of normalized left x coordinates in bounding box (1 per box)
    xmaxs = [] # List of normalized right x coordinates in bounding box
                # (1 per box)
    ymins = [] # List of normalized top y coordinates in bounding box (1 per box)
    ymaxs = [] # List of normalized bottom y coordinates in bounding box
                # (1 per box)
    classes_text = [] # List of string class name of bounding box (1 per box)
    classes = [] # List of integer class id of bounding box (1 per box)

    labels = {'Green': 1, 'Red': 2, 'Yellow': 3, 'Off': 4}
    if xml_path:
        d = minidom.parse(xml_path)
        for o in d.getElementsByTagName('object'):
            bndbox = o.getElementsByTagName('bndbox')[0]
            xmin = float(bndbox.getElementsByTagName('xmin')[0].childNodes[0].data)/width
            xmins.append(xmin)
            xmax = float(bndbox.getElementsByTagName('xmax')[0].childNodes[0].data)/width
            xmaxs.append(xmax)

            ymin = float(bndbox.getElementsByTagName('ymin')[0].childNodes[0].data)/height
            ymins.append(ymin)
            ymax = float(bndbox.getElementsByTagName('ymax')[0].childNodes[0].data)/height
            ymaxs.append(ymax)

            name = o.getElementsByTagName('name')[0].childNodes[0].data
            classes_text.append(name.encode('utf-8'))
            label = labels[name]
            classes.append(label)

    tf_example = tf.train.Example(features=tf.train.Features(feature={
        'image/height': dataset_util.int64_feature(height),
        'image/width': dataset_util.int64_feature(width),
        'image/filename': dataset_util.bytes_feature(filename),
        'image/source_id': dataset_util.bytes_feature(filename),
        'image/encoded': dataset_util.bytes_feature(encoded_image_data),
        'image/format': dataset_util.bytes_feature(image_format),
        'image/object/bbox/xmin': dataset_util.float_list_feature(xmins),
        'image/object/bbox/xmax': dataset_util.float_list_feature(xmaxs),
        'image/object/bbox/ymin': dataset_util.float_list_feature(ymins),
        'image/object/bbox/ymax': dataset_util.float_list_feature(ymaxs),
        'image/object/class/text': dataset_util.bytes_list_feature(classes_text),
        'image/object/class/label': dataset_util.int64_list_feature(classes),
    }))
    return tf_example


def main(_):
    train_writer = tf.python_io.TFRecordWriter('test_'+FLAGS.output_path)
    eval_writer = tf.python_io.TFRecordWriter('eval_'+FLAGS.output_path)

    labeled={}
    for dirpath, dnames, fnames in os.walk("/home/prombo/car/CarND-Capstone/traffic_light/collected_data"):
        for f in fnames:
            if f.endswith(".xml"):
                base = f.replace(".xml", "")
                labeled[base] = os.path.join(dirpath, f)
            elif f.endswith(".jpg"):
                base = f.replace(".jpg", "")
                if not base in labeled:
                    labeled[base]=None

    for base, xml_path in labeled.items():
        tf_example = create_tf_example(base, xml_path)
        if random.random() < 0.8:
            train_writer.write(tf_example.SerializeToString())
        else:
            eval_writer.write(tf_example.SerializeToString())    

    train_writer.close()
    eval_writer.close()

if __name__ == '__main__':
  tf.app.run()

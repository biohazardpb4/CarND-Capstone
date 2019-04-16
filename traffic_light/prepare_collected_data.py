import os
import random
from xml.dom import minidom

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
train={}
valid={}

for k, v in labeled.items():
    if random.random() < 0.8:
        train[k]=v
    else:
        valid[k]=v

print("split into train: {} and valid: {}".format(len(train), len(valid)))

for base, xml_path in train.items():
    if xml_path:
        d = minidom.parse(xml_path)

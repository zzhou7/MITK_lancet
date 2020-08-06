import pickle
from collections import defaultdict

import os
import numpy as np

root_dir = "/media/kleina/Data/Data/bone_seg_ijcars"
image_dir = os.path.join(root_dir, 'images_dcm')
label_dir = os.path.join(root_dir, 'labels_nrrd')
output_dir = os.path.join(root_dir, 'splits_basic')

classes = 3

class_stats = defaultdict(int)
total = 0

image_files = []
for x in os.walk(image_dir):
    image_files = x[1]
    break

trainset_size = 12
valset_size = 3
testset_size = 3

splits=[]
for split in range(0,6):
    image_list = image_files.copy()
    trainset = []
    valset = []
    testset = []
    for i in range(0, testset_size):
        print(split*testset_size+i)
        patient = image_list[split*testset_size]
        print(patient)
        image_list.remove(patient)
        testset.append(patient)
    for i in range(0, trainset_size):
        patient = np.random.choice(image_list)
        image_list.remove(patient)
        trainset.append(patient)
    for i in range(0, valset_size):
        patient = np.random.choice(image_list)
        image_list.remove(patient)
        valset.append(patient)
    split_dict = dict()
    split_dict['train'] = trainset
    split_dict['val'] = valset
    split_dict['test'] = testset

    splits.append(split_dict)

with open(os.path.join(output_dir,'splits.pkl'), 'wb') as f:
    pickle.dump(splits, f)
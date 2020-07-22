#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright 2017 Division of Medical Image Computing, German Cancer Research Center (DKFZ)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from collections import defaultdict

from medpy.io import load
import os
import numpy as np

from datasets.utils import reshape
from utilities.file_and_folder_operations import subfiles

from collections import defaultdict

import os
import numpy as np
from datasets.bone_seg.utils import load_dcm_image, load_label, reshape


root_dir = "/media/kleina/Data/Data/bone_seg_ijcars"
image_dir = os.path.join(root_dir, 'images_dcm')
label_dir = os.path.join(root_dir, 'labels_nrrd')

output_dir = os.path.join(root_dir, 'preprocessed_basic')


total = 0

image_files = []
for x in os.walk(image_dir):
    image_files = x[1]
    break

for dcm_dir in image_files:
    if os.path.isfile(os.path.join(output_dir, dcm_dir)+'.npy'):
        print('already exists...continue...')
        continue
    image, spacing = load_dcm_image(os.path.join(image_dir, dcm_dir))

    image_reshaped = reshape(image, spacing)
    label = load_label(os.path.join(label_dir, dcm_dir))
    # label_reshaped = reshape(label, spacing, append_value=0)

    # image_reshaped = (image_reshaped - image_reshaped.min()) / (image_reshaped.max() - image_reshaped.min() + 1e-7)

    result = np.stack((image, label))
    np.save(os.path.join(output_dir, dcm_dir + '.npy'), result)
    print(dcm_dir)
    total += 1

print('Total count: ')
print(total)

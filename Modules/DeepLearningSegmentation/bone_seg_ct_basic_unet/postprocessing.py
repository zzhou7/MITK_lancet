from collections import defaultdict

import os
import numpy as np
from SimpleITK import SimpleITK

from datasets.bone_seg.utils import load_nrrd_image, load_label, reshape
from datasets.utils import crop_image_to_orig_size

direction = 'cor'

root_dir = "/media/kleina/Data/Data/CTs-ProximalFemur"
root_dir2 = "/media/kleina/Data/Data/bone_seg_ijcars"
image_dir = os.path.join(root_dir, 'nrrd')
label_dir = os.path.join(root_dir2, 'segmented')

total = 0

image_files = []
for x in os.walk(image_dir):
    image_files = x[2]
    break

for image_file in image_files:
    image_file = 'B89D0BAF_9F321786_B1776411_cropped.nrrd'
    image = load_nrrd_image(os.path.join(image_dir, image_file))
    seg = load_nrrd_image(os.path.join(label_dir, image_file))

    result = crop_image_to_orig_size(seg, image.shape)

    image_nrrd = SimpleITK.ReadImage(os.path.join(image_dir, image_file))

    image_to_write = SimpleITK.GetImageFromArray(result)
    image_to_write.SetSpacing(image_nrrd.GetSpacing())
    image_to_write.SetOrigin(image_nrrd.GetOrigin())
    image_to_write.SetDirection(image_nrrd.GetDirection())
    SimpleITK.WriteImage(SimpleITK.Cast(image_to_write, SimpleITK.sitkUInt8), os.path.join(label_dir, image_file))

print('Total count: ')
print(total)

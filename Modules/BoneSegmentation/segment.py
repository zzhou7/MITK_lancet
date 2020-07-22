import os
import numpy as np
import SimpleITK
import torch
import torch.cuda

print('Started python script...')

import SimpleITK as sitk
from skimage.transform import resize
import sys


def resize_image(image, old_spacing, new_spacing, order=3):
    new_shape = (int(np.round(old_spacing[0]/new_spacing[0]*float(image.shape[0]))),
                 int(np.round(old_spacing[1]/new_spacing[1]*float(image.shape[1]))),
                 int(np.round(old_spacing[2]/new_spacing[2]*float(image.shape[2]))))
    return resize(image, new_shape, order=order, mode='edge')


def cut_off_values_upper_lower_percentile(image, mask=None, percentile_lower=0.2, percentile_upper=99.8):
    if mask is None:
        mask = image!=image[0,0,0]
    cut_off_lower = np.percentile(image[mask!=0].ravel(), percentile_lower)
    cut_off_upper = np.percentile(image[mask!=0].ravel(), percentile_upper)
    res = np.copy(image)
    res[(res < cut_off_lower) & (mask !=0 )] = cut_off_lower
    res[(res > cut_off_upper) & (mask !=0 )] = cut_off_upper
    return image


def preprocess_image(itk_image, is_seg=False, spacing_target=(1, 0.5, 0.5)):
    spacing = np.array(itk_image.GetSpacing())[[2, 1, 0]]
    image = sitk.GetArrayFromImage(itk_image).astype(float)
    if not is_seg:
        image = resize_image(image, spacing, spacing_target).astype(np.float32)
        # cut off outliers
        image = cut_off_values_upper_lower_percentile(image, np.ones(image.shape), 1., 99.)
        #subtract mean, divide by std. use heuristic masking
        image -= image.mean()
        image /= image.std()
    else:
        image = resize_image(image, spacing, spacing_target, 0)
    return image


def load_and_preprocess(in_image):
    images = {}
    
    np_array = sitk.GetArrayFromImage(in_image).astype(float)
    if len(np_array.shape) > 3:
      b0 = sitk.GetImageFromArray(np_array[:,:,:,0])
      b0.SetSpacing(in_image.GetSpacing())
      b0.SetOrigin(in_image.GetOrigin())
      b0.SetDirection(in_image.GetDirection())
      images["T1"] = b0
    elif len(np_array.shape)==3 :
      images["T1"] = in_image
    
    properties_dict = {
        "spacing": in_image.GetSpacing(),
        "direction": in_image.GetDirection(),
        "size": in_image.GetSize(),
        "origin": in_image.GetOrigin()
    }

    for k in images.keys():
        images[k] = preprocess_image(images[k], is_seg=False, spacing_target=(1.5, 1.5, 1.5))

    properties_dict['size_before_cropping'] = images["T1"].shape

    imgs = []
    for seq in ['T1']:
        imgs.append(images[seq][None])
    all_data = np.vstack(imgs)
    return all_data, properties_dict

def get_sitk_from_nparray(segmentation, original_image, dct) :
    '''
    segmentation must have the same spacing as the original nifti (for now). segmentation may have been cropped out
    of the original image
    :param segmentation:
    :param dct:
    :return:
    '''
    old_size = np.array(dct['size_before_cropping'])
    bbox = dct.get('brain_bbox')
    if bbox is not None:
        seg_old_size = np.zeros(old_size)
        for c in range(3):
            bbox[c][1] = np.min((bbox[c][0] + segmentation.shape[c], old_size[c]))
        seg_old_size[bbox[0][0]:bbox[0][1],
                     bbox[1][0]:bbox[1][1],
                     bbox[2][0]:bbox[2][1]] = segmentation
    else:
        seg_old_size = segmentation

    seg_old_spacing = resize_segmentation(seg_old_size, np.array(dct['size'])[[2, 1, 0]], order=3)
    seg_resized_itk = sitk.GetImageFromArray(seg_old_spacing.astype(np.uint8))
    seg_resized_itk.SetSpacing(np.array(dct['spacing'])[[0, 1, 2]])
    seg_resized_itk.SetOrigin(dct['origin'])
    seg_resized_itk.SetDirection(dct['direction'])
    
    if original_image is not None :
      image = sitk.GetArrayFromImage(original_image).astype(float)
      
      if len(image.shape) > 3 :
        for i in range(image.shape[3]) :
          image[:,:,:,i] *= seg_old_spacing
      else :
        image *= seg_old_spacing
      
      brain_extracted = sitk.GetImageFromArray(image.astype(np.float32))
      brain_extracted.SetSpacing(np.array(dct['spacing'])[[0, 1, 2]])
      brain_extracted.SetOrigin(dct['origin'])
      brain_extracted.SetDirection(dct['direction'])
    else :
      brain_extracted = None
    
    return seg_resized_itk, brain_extracted

def save_segmentation_nifti(segmentation, dct, out_fname):
    '''
    segmentation must have the same spacing as the original nifti (for now). segmentation may have been cropped out
    of the original image
    :param segmentation:
    :param dct:
    :param out_fname:
    :return:
    '''
    old_size = np.array(dct['size_before_cropping'])
    bbox = dct.get('brain_bbox')
    if bbox is not None:
        seg_old_size = np.zeros(old_size)
        for c in range(3):
            bbox[c][1] = np.min((bbox[c][0] + segmentation.shape[c], old_size[c]))
        seg_old_size[bbox[0][0]:bbox[0][1],
                     bbox[1][0]:bbox[1][1],
                     bbox[2][0]:bbox[2][1]] = segmentation
    else:
        seg_old_size = segmentation

    seg_old_spacing = resize_segmentation(seg_old_size, np.array(dct['size'])[[2, 1, 0]], order=3)
    seg_resized_itk = sitk.GetImageFromArray(seg_old_spacing.astype(np.uint8))
    seg_resized_itk.SetSpacing(np.array(dct['spacing'])[[0, 1, 2]])
    seg_resized_itk.SetOrigin(dct['origin'])
    seg_resized_itk.SetDirection(dct['direction'])
    sitk.WriteImage(seg_resized_itk, out_fname)


def resize_segmentation(segmentation, new_shape, order=3):
    unique_labels = np.unique(segmentation)
    assert len(segmentation.shape) == len(new_shape), "new shape must have same dimensionality as segmentation"
    reshaped_multihot = np.zeros([len(unique_labels)] + list(new_shape), dtype=float)
    for i, c in enumerate(unique_labels):
        reshaped_multihot[i] = resize((segmentation == c).astype(float), new_shape, order, mode="constant", cval=0, clip=True)
    reshaped = unique_labels[np.argmax(reshaped_multihot, 0)]
    return reshaped.astype(segmentation.dtype)


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

import numpy as np


def reshape(orig_img, append_value=-1024, new_shape=(512, 512, 512)):
    reshaped_image = np.zeros(new_shape)
    reshaped_image[...] = append_value
    x_offset = 0
    y_offset = 0  # (new_shape[1] - orig_img.shape[1]) // 2
    z_offset = 0  # (new_shape[2] - orig_img.shape[2]) // 2

    reshaped_image[x_offset:orig_img.shape[0]+x_offset, y_offset:orig_img.shape[1]+y_offset, z_offset:orig_img.shape[2]+z_offset] = orig_img
    # insert temp_img.min() as background value

    return reshaped_image


def crop_image_to_orig_size(image, orig_shape):
    x_offset = 0
    y_offset = 0  # (512 - orig_shape[1]) // 2
    z_offset = 0  # (512 - orig_shape[2]) // 2

    return image[x_offset:orig_shape[0] + x_offset, y_offset:orig_shape[1] + y_offset, z_offset:orig_shape[2] + z_offset]

def next_power_of_2(x):  
    return 1 if x == 0 else 2**(x - 1).bit_length()

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

# Defines the Unet.
# |num_downs|: number of downsamplings in UNet. For example,
# if |num_downs| == 7, image of size 128x128 will become of size 1x1 at the bottleneck

# recursive implementation of Unet
import torch

from torch import nn


class UNet(nn.Module):
    def __init__(self, num_classes=3, in_channels=1, initial_filter_size=64, kernel_size=3, num_downs=4, norm_layer=nn.InstanceNorm2d):
        # norm_layer=nn.BatchNorm2d, use_dropout=False):
        super(UNet, self).__init__()

        # construct unet structure
        unet_block = UnetSkipConnectionBlock(in_channels=initial_filter_size * 2 ** (num_downs-1), out_channels=initial_filter_size * 2 ** num_downs,
                                             num_classes=num_classes, kernel_size=kernel_size, norm_layer=norm_layer, innermost=True)
        for i in range(1, num_downs):
            unet_block = UnetSkipConnectionBlock(in_channels=initial_filter_size * 2 ** (num_downs-(i+1)),
                                                 out_channels=initial_filter_size * 2 ** (num_downs-i),
                                                 num_classes=num_classes, kernel_size=kernel_size, submodule=unet_block, norm_layer=norm_layer)
        unet_block = UnetSkipConnectionBlock(in_channels=in_channels, out_channels=initial_filter_size,
                                             num_classes=num_classes, kernel_size=kernel_size, submodule=unet_block, norm_layer=norm_layer,
                                             outermost=True)

        self.model = unet_block

    def forward(self, x):
        return self.model(x)


# Defines the submodule with skip connection.
# X -------------------identity---------------------- X
#   |-- downsampling -- |submodule| -- upsampling --|
class UnetSkipConnectionBlock(nn.Module):
    def __init__(self, in_channels=None, out_channels=None, num_classes=1, kernel_size=3,
                 submodule=None, outermost=False, innermost=False, norm_layer=nn.InstanceNorm2d, use_dropout=False):
        super(UnetSkipConnectionBlock, self).__init__()
        self.outermost = outermost
        # downconv
        pool = nn.MaxPool2d(2, stride=2)
        conv1 = self.contract(in_channels=in_channels, out_channels=out_channels, kernel_size=kernel_size, norm_layer=norm_layer)
        conv2 = self.contract(in_channels=out_channels, out_channels=out_channels, kernel_size=kernel_size, norm_layer=norm_layer)

        # upconv
        conv3 = self.expand(in_channels=out_channels*2, out_channels=out_channels, kernel_size=kernel_size)
        conv4 = self.expand(in_channels=out_channels, out_channels=out_channels, kernel_size=kernel_size)

        if outermost:
            final = nn.Conv2d(out_channels, num_classes, kernel_size=1)
            down = [conv1, conv2]
            up = [conv3, conv4, final]
            model = down + [submodule] + up
        elif innermost:
            upconv = nn.ConvTranspose2d(in_channels*2, in_channels,
                                        kernel_size=2, stride=2)
            model = [pool, conv1, conv2, upconv]
        else:
            upconv = nn.ConvTranspose2d(in_channels*2, in_channels, kernel_size=2, stride=2)

            down = [pool, conv1, conv2]
            up = [conv3, conv4, upconv]

            if use_dropout:
                model = down + [submodule] + up + [nn.Dropout(0.5)]
            else:
                model = down + [submodule] + up

        self.model = nn.Sequential(*model)

    @staticmethod
    def contract(in_channels, out_channels, kernel_size=3, norm_layer=nn.InstanceNorm2d):
        layer = nn.Sequential(
            nn.Conv2d(in_channels, out_channels, kernel_size, padding=1),
            norm_layer(out_channels),
            nn.LeakyReLU(inplace=True))
        return layer

    @staticmethod
    def expand(in_channels, out_channels, kernel_size=3):
        layer = nn.Sequential(
            nn.Conv2d(in_channels, out_channels, kernel_size, padding=1),
            nn.LeakyReLU(inplace=True),
        )
        return layer

    @staticmethod
    def center_crop(layer, target_width, target_height):
        batch_size, n_channels, layer_width, layer_height = layer.size()
        xy1 = (layer_width - target_width) // 2
        xy2 = (layer_height - target_height) // 2
        return layer[:, :, xy1:(xy1 + target_width), xy2:(xy2 + target_height)]

    def forward(self, x):
        if self.outermost:
            return self.model(x)
        else:
            crop = self.center_crop(self.model(x), x.size()[2], x.size()[3])
            return torch.cat([x, crop], 1)

batch_size = 8
num_classes = 2

device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')


model = UNet(num_classes=num_classes, in_channels=1)
model.load_state_dict(torch.load(
    seg_load_network_path, map_location=device))
print(device)

model.eval()
model.to(device)

result = []
first = True
print('Segmenting...')

input_image_array = SimpleITK.GetArrayFromImage(nrrd_image)
append_empty_slices = batch_size - (input_image_array.shape[0]%batch_size)
print("A")
y = next_power_of_2(input_image_array.shape[1])
z = next_power_of_2(input_image_array.shape[2])
print("B")
if y > 512 or z > 512:
    batch_size = 2
temp_array = np.ones((input_image_array.shape[0] + append_empty_slices, y, z))*(-1024)
temp_array[0:input_image_array.shape[0], 0:input_image_array.shape[1], 0:input_image_array.shape[2]] = input_image_array
print("C")
with torch.no_grad():
    print("here1")
    start = 0
    end = start+batch_size
    while end <= temp_array.shape[0]:
        print("here2")
        pred = model(torch.from_numpy(np.expand_dims(
            temp_array[start:end], axis=1).astype(np.float32)).to(device))
        print("D")
        if first:
            result = pred.detach().data.cpu()
            first = False
            print("if")
        else:
            result = torch.cat((result, pred.detach().data.cpu()))
            print("else")
        print(result.shape)
        start = end
        end = start + batch_size
result = torch.argmax(result, dim=1, keepdim=True)
print("E")
array_to_write = result.data.numpy().squeeze()
array_to_write = array_to_write[0:input_image_array.shape[0], 0:input_image_array.shape[1], 0:input_image_array.shape[2]]
array_to_write = crop_image_to_orig_size( array_to_write, SimpleITK.GetArrayFromImage(nrrd_image).shape)
print(array_to_write.shape)

image_to_write = SimpleITK.GetImageFromArray(array_to_write)

image_to_write.SetSpacing(nrrd_image.GetSpacing())
image_to_write.SetOrigin(nrrd_image.GetOrigin())
image_to_write.SetDirection(nrrd_image.GetDirection())
image_to_write = SimpleITK.Cast(image_to_write, SimpleITK.sitkUInt8)

SimpleITK.Cast(image_to_write, SimpleITK.sitkUInt8)
print('Ended python script...')

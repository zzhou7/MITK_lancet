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

import os
import pickle
from collections import defaultdict

import numpy as np
import torch
import torch.optim as optim
from torch.optim.lr_scheduler import ReduceLROnPlateau
import torch.nn.functional as F

from datasets.bone_seg.utils import load_nrrd_image, load_dcm_image
from datasets.two_dim.NumpyDataLoader import NumpyDataSet
from trixi.experiment.pytorchexperiment import PytorchExperiment
from batchgenerators.augmentations.utils import pad_nd_image

from datasets.utils import crop_image_to_orig_size
from evaluation.evaluator import aggregate_scores, Evaluator
from networks.RecursiveUNet import UNet
from loss_functions.dice_loss import SoftDiceLoss


class UNetExperiment(PytorchExperiment):
    """
    The UnetExperiment is inherited from the PytorchExperiment. It implements the basic life cycle for a segmentation task with UNet(https://arxiv.org/abs/1505.04597).
    It is optimized to work with the provided NumpyDataLoader.

    The basic life cycle of a UnetExperiment is the same s PytorchExperiment:

        setup()
        (--> Automatically restore values if a previous checkpoint is given)
        prepare()

        for epoch in n_epochs:
            train()
            validate()
            (--> save current checkpoint)

        end()
    """

    def setup(self):
        self.elog.print('=====SETUP=====')
        pkl_dir = self.config.split_dir
        with open(os.path.join(pkl_dir, "mr_splits.pkl"), 'rb') as f:
            splits = pickle.load(f)

        mr_tr_keys = splits[self.config.fold]['train']
        val_keys = splits[self.config.fold]['val']
        test_keys = splits[self.config.fold]['test']

        with open(os.path.join(pkl_dir, "ct_splits.pkl"), 'rb') as f:
            ct_splits = pickle.load(f)

        ct_tr_keys = ct_splits[self.config.fold]['train']
        ct_val_keys = ct_splits[self.config.fold]['val']
        ct_test_keys = ct_splits[self.config.fold]['test']

        tr_keys = mr_tr_keys + ct_tr_keys[0:12]  #TODO: Don't hardcode something like this. You'll hate yourself for it later.

        self.device = torch.device(self.config.device if torch.cuda.is_available() else "cpu")

        # Define file_pattern for CT/MR
        # file_pattern_mr = "*_MR_*.npy"
        file_pattern_mr = "*.npy"

        self.train_data_loader = NumpyDataSet(self.config.data_dir, target_size=self.config.patch_size, batch_size=self.config.batch_size,
                                              keys=ct_tr_keys, label_slice=1, file_pattern=file_pattern_mr)
        self.train_data_loader_mr = NumpyDataSet(self.config.data_dir, target_size=self.config.patch_size, batch_size=self.config.batch_size,
                                              keys=mr_tr_keys, label_slice=1, file_pattern=file_pattern_mr)
        self.val_data_loader = NumpyDataSet(self.config.data_dir, target_size=self.config.patch_size, batch_size=self.config.batch_size,
                                            keys=val_keys, mode="val", label_slice=1, file_pattern=file_pattern_mr)
        self.test_data_loader = NumpyDataSet(self.config.data_dir, target_size=self.config.patch_size, batch_size=self.config.batch_size,
                                            keys=test_keys, mode="test", label_slice=1, file_pattern=file_pattern_mr)

        self.model = UNet(num_classes=self.config.num_classes, in_channels=self.config.in_channels)

        self.model.to(self.device)

        # We use a combination of DICE-loss and CE-Loss in this example.
        # This proved good in the medical segmentation decathlon.
        self.dice_loss = SoftDiceLoss(batch_dice=True)  # Softmax für DICE Loss!
        self.ce_loss = torch.nn.CrossEntropyLoss()  # Kein Softmax für CE Loss -> ist in torch schon mit drin!

        self.optimizer = optim.Adam(self.model.parameters(), lr=self.config.learning_rate)
        self.scheduler = ReduceLROnPlateau(self.optimizer, 'min')

        # If directory for checkpoint is provided, we load it.
        if self.config.do_load_checkpoint:
            if self.config.checkpoint_dir == '':
                print('checkpoint_dir is empty, please provide directory to load checkpoint.')
            else:
                self.load_checkpoint(name=self.config.checkpoint_dir, save_types=("model"))

        self.save_checkpoint(name="checkpoint_start")
        self.elog.print('Experiment set up.')

    def train(self, epoch):
        self.elog.print('=====TRAIN=====')
        self.model.train()

        batch_counter = 0
        for data_batch in self.train_data_loader:

            #  <editor-fold desc="Train on CT">
            self.optimizer.zero_grad()

            # Shape of data_batch = [1, b, c, w, h]
            # Desired shape = [b, c, w, h]
            # Move data and target to the GPU
            data = data_batch['data'][0].float().to(self.device)
            target = data_batch['seg'][0].long().to(self.device)

            pred = self.model(data)
            pred_softmax = F.softmax(pred, dim=1)  # We calculate a softmax, because our SoftDiceLoss expects that as an input. The CE-Loss does the softmax internally.

            loss = self.dice_loss(pred_softmax, target.squeeze()) + self.ce_loss(pred, target.squeeze())
            # loss = self.ce_loss(pred, target.squeeze())
            loss.backward()
            self.optimizer.step()

            # Some logging and plotting
            if (batch_counter % self.config.plot_freq) == 0:
                self.elog.print('Epoch: %d Loss: %.4f' % (self._epoch_idx, loss))

                self.add_result(value=loss.item(), name='Train_Loss_CT', label='Loss', counter=epoch + (batch_counter / self.train_data_loader.data_loader.num_batches))

                data_norm = (data - data.min()) / (data.max() - data.min() + 1e-7)
                self.clog.show_image_grid(data_norm.float(), name="data", normalize=True, scale_each=True, n_iter=epoch)
                self.clog.show_image_grid(target.float(), name="mask", title="Mask", n_iter=epoch)
                self.clog.show_image_grid(torch.argmax(pred.cpu(), dim=1, keepdim=True), name="unt_argmax", title="Unet", n_iter=epoch)
                self.clog.show_image_grid(pred.cpu()[:, 1:2, ], name="unt", normalize=True, scale_each=True, n_iter=epoch)
            # </editor-fold>

            #  <editor-fold desc="Train on MR">

            # Shape of data_batch = [1, b, c, w, h]
            # Desired shape = [b, c, w, h]
            # Move data and target to the GPU
            try:
                data_batch = self.train_data_loader_mr.get_next()
            except StopIteration:
                break  # This occurs, if there are more MR datasets than CT datesets.

            data = data_batch['data'][0].float().to(self.device)
            target = data_batch['seg'][0].long().to(self.device)

            pred = self.model(data)
            pred_softmax = F.softmax(pred, dim=1)  # We calculate a softmax, because our SoftDiceLoss expects that as an input. The CE-Loss does the softmax internally.

            loss = self.dice_loss(pred_softmax, target.squeeze()) + self.ce_loss(pred, target.squeeze())
            # loss = self.ce_loss(pred, target.squeeze())
            loss.backward()
            self.optimizer.step()

            # Some logging and plotting
            if (batch_counter % self.config.plot_freq/2) == 0:
                self.elog.print('Epoch: %d Loss: %.4f' % (self._epoch_idx, loss))

                self.add_result(value=loss.item(), name='Train_Loss_MR', label='Loss', counter=epoch + (batch_counter / self.train_data_loader.data_loader.num_batches))

                data_norm = (data - data.min()) / (data.max() - data.min() + 1e-7)
                self.clog.show_image_grid(data_norm.float(), name="data", normalize=True, scale_each=True, n_iter=epoch)
                self.clog.show_image_grid(target.float(), name="mask", title="Mask", n_iter=epoch)
                self.clog.show_image_grid(torch.argmax(pred.cpu(), dim=1, keepdim=True), name="unt_argmax", title="Unet", n_iter=epoch)
                self.clog.show_image_grid(pred.cpu()[:, 1:2, ], name="unt", normalize=True, scale_each=True, n_iter=epoch)
            # </editor-fold>

            batch_counter += 1

    def validate(self, epoch):
        self.elog.print('VALIDATE')
        self.model.eval()

        loss_list = []

        with torch.no_grad():
            for data_batch in self.val_data_loader:
                data = data_batch['data'][0].float().to(self.device)
                target = data_batch['seg'][0].long().to(self.device)

                pred = self.model(data)
                pred_softmax = F.softmax(pred, dim=1)  # We calculate a softmax, because our SoftDiceLoss expects that as an input. The CE-Loss does the softmax internally.

                loss = self.dice_loss(pred_softmax, target.squeeze()) + self.ce_loss(pred, target.squeeze())
                loss_list.append(loss.item())

        self.scheduler.step(np.mean(loss_list))

        self.elog.print('Epoch: %d Loss: %.4f' % (self._epoch_idx, np.mean(loss_list)))

        self.add_result(value=np.mean(loss_list), name='Val_Loss', label='Loss', counter=epoch+1)

        self.clog.show_image_grid(data.float(), name="data_val", normalize=True, scale_each=True, n_iter=epoch)
        self.clog.show_image_grid(target.float(), name="mask_val", title="Mask", n_iter=epoch)
        self.clog.show_image_grid(torch.argmax(pred.data.cpu(), dim=1, keepdim=True), name="unt_argmax_val", title="Unet", n_iter=epoch)
        self.clog.show_image_grid(pred.data.cpu()[:, 1:2, ], name="unt_val", normalize=True, scale_each=True, n_iter=epoch)

    def test(self):
        self.elog.print('=====TEST=====')
        self.model.eval()

        pred_dict = defaultdict(list)
        gt_dict = defaultdict(list)

        batch_counter = 0
        with torch.no_grad():
            for data_batch in self.test_data_loader:
                print('testing...', batch_counter)
                batch_counter += 1

                # Get data_batches
                mr_data = data_batch['data'][0].float().to(self.device)
                mr_target = data_batch['seg'][0].float().to(self.device)

                pred = self.model(mr_data)
                pred_argmax = torch.argmax(pred.data.cpu(), dim=1, keepdim=True)

                fnames = data_batch['fnames']
                for i, fname in enumerate(fnames):
                    pred_dict[fname[0]].append(pred_argmax[i].detach().cpu().numpy())
                    gt_dict[fname[0]].append(mr_target[i].detach().cpu().numpy())

        test_ref_list = []
        for key in pred_dict.keys():
            test_ref_list.append((np.stack(pred_dict[key]), np.stack(gt_dict[key])))

        scores = aggregate_scores(test_ref_list, evaluator=Evaluator, json_author='kleina', json_task=self.config.name, json_name=self.config.name,
                                  json_output_file=self.elog.work_dir + "/kleina_" + self.config.name + '.json')

        print("Scores:\n", scores)

    def segment(self, sitk_image):
        print("here")
        import SimpleITK
        self.model = UNet(num_classes=self.config.num_classes, in_channels=1)
        self.device = torch.device(self.config.device if torch.cuda.is_available() else "cpu")
        self.model.to(self.device)

        # file = self.config.seg_input  # [i.split('/')[-1] for i in subfiles(os.path.join(self.config.seg_input_dir, 'preprocessed'))]
        # self.load_checkpoint(name=self.config.seg_load_network_path, save_types=("model"))
        self.model.load_state_dict(torch.load(os.path.join(os.getcwd(), self.config.seg_load_network_path), map_location=self.device))
        from utilities.file_and_folder_operations import subfiles
        image_data = None
        # if file.endswith('.nrrd'):
            #input_image_array, sitk_image = load_nrrd_image(file)
        #elif any(".dcm" in s for s in subfiles(file)):
            #input_image_array, sitk_image = load_dcm_image(file)
        print(dir())
        print("-----------------")
        print(globals())
        print("-----------------")
        print(locals())
        input_image_array = SimpleITK.GetArrayFromImage(sitk_image)
        append_empty_slices = self.config.batch_size - (input_image_array.shape[0] % self.config.batch_size)
        y_size = self.next_power_of_2(input_image_array.shape[1])
        z_size = self.next_power_of_2(input_image_array.shape[2])
        temp_array = np.ones((input_image_array.shape[0] + append_empty_slices, y_size, z_size)) * (-1024)
        temp_array[0:input_image_array.shape[0], 0:input_image_array.shape[1], 0:input_image_array.shape[2]] = input_image_array
        # temp_array = pad_nd_image(input_image_array, (input_image_array.shape[0] + append_empty_slices, y_size, z_size), "constant", kwargs={'constant_values':
        # input_image_array.min()})
        first = True
        with torch.no_grad():
            start = 0
            end = start + self.config.batch_size
            while end <= temp_array.shape[0]:
                pred = self.model(torch.from_numpy(np.expand_dims(
                    temp_array[start:end], axis=1).astype(np.float32)).to(self.device))
                if first:
                    result = pred.detach().data.cpu()
                    first = False
                else:
                    result = torch.cat((result, pred.detach().data.cpu()))
                print(result.shape)
                start = end
                end = start + self.config.batch_size
        result = torch.argmax(result, dim=1, keepdim=True)
        # result

        array_to_write = result.data.numpy().squeeze()
        # np.save(os.path.join(self.config.seg_output_dir, self.config.seg_input_keys) + '.npy', array_to_write)
        # print(os.path.join(self.config.seg_output_dir, self.config.seg_input_keys) + '.npy', array_to_write)

        print(array_to_write.shape)
        new_spacing = sitk_image.GetSpacing()  # image_dicom.GetSpacing()

        # from scipy.ndimage.interpolation import zoom
        # array_to_write = zoom(array_to_write, (1, (1 / new_spacing[0]), (1 / new_spacing[1])))
        # array_to_write = crop_image_to_orig_size(array_to_write, SimpleITK.GetArrayFromImage(sitk_image).shape)
        print(array_to_write.shape)

        a1 = (y_size - input_image_array.shape[1])
        a2 = (y_size - input_image_array.shape[2])
        b1 = a1 // 2
        b2 = a2 // 2

        array_to_write = array_to_write[0:input_image_array.shape[0], b1:input_image_array.shape[1] + a1 - b1, b2:input_image_array.shape[2] + a2 - b2]
        image_to_write = SimpleITK.GetImageFromArray(array_to_write)

        image_to_write.SetSpacing(sitk_image.GetSpacing())
        image_to_write.SetOrigin(sitk_image.GetOrigin())
        image_to_write.SetDirection(sitk_image.GetDirection())

        image_to_write = SimpleITK.Cast(image_to_write, SimpleITK.sitkUInt8)

        return image_to_write

        #SimpleITK.WriteImage(SimpleITK.Cast(image_to_write, SimpleITK.sitkUInt8), os.path.join(self.config.seg_output_dir, file.split('/')[-1])+'.nrrd')

    def next_power_of_2(self, x):
        return 1 if x == 0 else 2 ** (x - 1).bit_length()
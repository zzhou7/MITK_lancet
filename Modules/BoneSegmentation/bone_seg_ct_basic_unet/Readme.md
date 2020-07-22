# U-Net CT Bone Segmenter by MIC@DKFZ
This python package allows to segment the bones in CT images. The model was trained on data from the Black Swan project
 on patients suffering from multiple myeloma. The project is  based on the "basic_unet_example",
 an example project of how to use a U-Net (Ronneberger et al.) for segmentation on medical images using 
PyTorch (https://www.pytorch.org).
It was developed at the Division of Medical Image Computing at the German Cancer Research Center (DKFZ).
It is also an example on how to use our other python packages batchgenerators (https://github.com/MIC-DKFZ/batchgenerators) and 
Trixi (https://github.com/MIC-DKFZ/trixi) to suit all our deep learning data augmentation needs.

The full project can be found here: https://github.com/MIC-DKFZ/basic_unet_example

## How to set it up
The example is very easy to use. Just create a new virtual environment in python and install the requirements. 
This example requires python3. We suggest to use virtualenvwrapper (https://virtualenvwrapper.readthedocs.io/en/latest/).
```
mkvirtualenv unet_example
pip3 install -r requirements.txt
```

You can edit the paths for data storage and logging in the config file. By default, everything is stored in your working directory.

# How to segment images
To segment a specific image, you can call the 'segment.py' script with the parameter '--seg_input'.
The projectcan handle NRRD and DICOM files. E.g.
```
python segment.py --seg_input /some_dir/some_file.nrrd
```
or
```
python segment.py --seg_input /some_dir/some_dcm_dir
```
In the second example the folder 'some_dcm_dir' contains the *.dcm files.

# How to segment multiple images
You can also use the 'segment_folder.py', if you want to segment multiple images. Just put all the images to segment in 
a folder and set this folder in 'segment_folder.py'.

There you go. Have fun.

# Disclaimer
This was not tested in detail. In case you have any questions, please contact me at andre.klein@dkfz.de
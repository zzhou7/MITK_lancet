error_string = None
del error_string
try:
    import nibabel as nib
    import numpy as np
    import pyMITK
    from tractseg.python_api import run_tractseg
    from tractseg.python_api import img_utils
    import SimpleITK  as sitk

    data = pyMITK.GetArrayFromImage(in_image)
    data = np.nan_to_num(data)

    print(data.shape)

    swapaxes = False
    data = np.swapaxes(data, 0, 3)
    data = np.swapaxes(data, 0, 1)
    data = np.swapaxes(data, 1, 2)
    data = np.swapaxes(data, 0, 2)

    swapaxes = True

    print(data.shape)
    print('Before segmentation')
    seg = run_tractseg(data=data, single_orientation=True, verbose=True, nr_cpus=1)
    print('After segmentation')
#    bla = nib.Nifti1Image(seg, affine)
#    nib.save(bla, '/home/neher/test.nii.gz')
    #seg = np.swapaxes(seg, 0, 2)

    data = np.swapaxes(data, 0, 3)
    data = np.swapaxes(data, 0, 1)
    data = np.swapaxes(data, 1, 2)
    data = np.swapaxes(data, 0, 2)

    seg = seg[:,:,:,0]
    print('Output shape: ' + str(seg.shape))

    print("max: "+str(np.amax(seg)))
    segmentation = pyMITK.GetImageFromArray(seg.astype(np.uint8))
    segmentation.SetGeometry(in_image.GetGeometry())

    #segmentation.SetOrigin(in_image.GetOrigin())
    #segmentation.SetSpacing(in_image.GetSpacing())
    #segmentation.SetDirection(in_image.GetDirection())

except Exception as e:
    error_string = str(e)
    print(error_string)

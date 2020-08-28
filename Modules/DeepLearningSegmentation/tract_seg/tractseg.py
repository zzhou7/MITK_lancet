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
    print('Threshold: '+str(threshold))
    print('In shape: '+str(data.shape))

    swapaxes = False
    data = np.swapaxes(data, 0, 3)
    data = np.swapaxes(data, 0, 1)
    data = np.swapaxes(data, 1, 2)
    data = np.swapaxes(data, 0, 2)

    swapaxes = True

    print('After swap: '+str(data.shape))
    print('Before segmentation')
    seg = run_tractseg(data=data, single_orientation=True, verbose=True, threshold=threshold, nr_cpus=1)
    print('After segmentation')

    seg = np.swapaxes(seg, 0, 3)
    seg = np.swapaxes(seg, 0, 1)
    seg = np.swapaxes(seg, 1, 2)
    seg = np.swapaxes(seg, 0, 2)

    print('Output shape: ' + str(seg.shape))

    print("max: "+str(np.amax(seg)))

    segList = [None]*seg.shape[0]
    for i in range(seg.shape[0]):
        tempseg = seg[i,:,:,:]
        segmentation = pyMITK.GetImageFromArray(tempseg.astype(np.uint8))
        segmentation.SetGeometry(in_image.GetGeometry())
        segList[i] = segmentation

except Exception as e:
    error_string = str(e)
    print(error_string)

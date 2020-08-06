import os
import pickle

pkl_dir = '/media/kleina/Data/Data/example_unet_dataset/Task04_Hippocampus/'
with open(os.path.join(pkl_dir, "splits.pkl"), 'rb') as f:
    splits = pickle.load(f)

print(splits)
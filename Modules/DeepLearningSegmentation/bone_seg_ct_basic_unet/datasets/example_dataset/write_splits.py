import os
import pickle

root_dir = "/media/kleina/Data/Data/Hippocampus"
filename = 'hippocampus_003_0000'
splits = [{'train': [filename], 'val': [filename], 'test': [filename]}]

with open(os.path.join(root_dir, 'splits.pkl'), 'wb') as f:
    pickle.dump(splits, f)
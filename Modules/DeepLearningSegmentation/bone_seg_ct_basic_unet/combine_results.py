import json
import pickle
from collections import OrderedDict

import numpy as np
import os

from evaluation.metrics import dice

# result_path = '/home/kleina/E130-Personal/Klein/Projects/2018_IJCARS_BoneSeg/TrainedModels/ResultsIJCARS_revision/perez_carrasco/results'

root_path = '/media/kleina/Data/output/unet_example/results/'


result_dict = OrderedDict()

result_path = root_path
metrics = ['Dice']

result_dict['1'] = OrderedDict()
result_dict['2'] = OrderedDict()
for metric in metrics:
    result_dict['1'][metric] = []
    result_dict['2'][metric] = []

json_file_names = [f for f in os.listdir(result_path)]
for metric in metrics:
    result_dict[metric] = []
for json_file_name in json_file_names:
    with open(os.path.join(result_path, json_file_name)) as json_file:
        data = json.load(json_file)
        for res in data['results']['all']:
            for metric in metrics:
                result_dict['1'][metric].append(res['1'][metric])
                result_dict['2'][metric].append(res['2'][metric])


print(result_dict)
print('Dice class 1: {}'.format(np.mean(result_dict['1']['Dice'])))
print('Dice class 2: {}'.format(np.mean(result_dict['2']['Dice'])))
# with open('/media/kleina/Data/output/unet_example/results/results.pkl', 'wb') as f:
#         pickle.dump(result_dict, f, pickle.HIGHEST_PROTOCOL)
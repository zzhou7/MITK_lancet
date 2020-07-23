import os
from meddec.move_datatsets_to_e132projects.ms_dataset import subdirs, subfiles
import shutil

def crawl_and_copy(current_folder, out_folder, prefix="fabian_", suffix="ummary.json"):
    """
    This script will run recursively through all subfolders of current_folder and copy all files that end with
    suffix with some automatically generated prefix into out_folder
    :param current_folder:
    :param out_folder:
    :param prefix:
    :return:
    """
    s = subdirs(current_folder, join=False)
    f = subfiles(current_folder, join=False)
    f = [i for i in f if i.endswith(suffix)]
    for fl in f:
        shutil.copy(os.path.join(current_folder, fl), os.path.join(out_folder, prefix+fl))
    for su in s:
        if prefix == "":
            add = su
        else:
            add = "__" + su
        crawl_and_copy(os.path.join(current_folder, su), out_folder, prefix=prefix+add)

if __name__ == "__main__":
    from meddec.paths import network_training_output_dir
    output_folder = "/home/fabian/drives/E132-Projekte/Projects/2018_MedicalDecathlon/Leaderboard/"
    crawl_and_copy(network_training_output_dir, output_folder)
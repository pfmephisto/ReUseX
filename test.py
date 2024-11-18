import os
import sys
import datetime
import configparser

# import scipy


print(os.getpid())
print(datetime.datetime.now())


sys.path.insert(
    0, '/home/mephisto/repos/linkml_cpp/build/')

import _core
from _core import *


# Actions
# PARSE_DATASET: bool     = False
# ANNOTATE: bool          = False
# REGISTER: bool          = False
# FILTER: bool            = False
# MERGE: bool             = False
# DOWNSAMPLE: bool        = False
# CLUSTER: bool           = False
# REGION_GROWING: bool    = False
# SOLIDIFY: bool          = True
# UPLOAD: bool            = False

# PARSE_DATASET: bool     = False
# ANNOTATE: bool          = False
# REGISTER: bool          = False
# FILTER: bool            = False
# MERGE: bool             = False
# DOWNSAMPLE: bool        = False
# CLUSTER: bool           = False
# REGION_GROWING: bool    = True
# SOLIDIFY: bool          = True
# UPLOAD: bool            = False


# PARSE_DATASET: bool     = False
# ANNOTATE: bool          = False
# REGISTER: bool          = False
# FILTER: bool            = False
# MERGE: bool             = False
# DOWNSAMPLE: bool        = True
# CLUSTER: bool           = True
# REGION_GROWING: bool    = True
# SOLIDIFY: bool          = True
# UPLOAD: bool            = False

PARSE_DATASET: bool     = False
ANNOTATE: bool          = False
REGISTER: bool          = False
FILTER: bool            = False
MERGE: bool             = False
DOWNSAMPLE: bool        = False
CLUSTER: bool           = False
REGION_GROWING: bool    = False
SOLIDIFY: bool          = False
UPLOAD: bool            = False



print("LinkML-Py loaded")



def chunks(lst, n):
    """Yield successive n-sized chunks from lst."""
    for i in range(0, len(lst), n):
        yield lst[i:i + n]

dataset_path = None

# dataset_path = "/home/mephisto/server_data/stray_scans/0ba33d855b/" # CPC Office
# dataset_path = "/home/mephisto/server_data/stray_scans/7092626a22/" # Aarhus Office
# dataset_path = "/home/mephisto/server_data/stray_scans/665518e46a/" # Lobby (School)

# Aarhus
# dataset_path = "/home/mephisto/server_data/stray_scans/8c0e3c381c/" # New scan small Basement <- Selected (Needs compleate rerun)
# dataset_path = "/home/mephisto/server_data/stray_scans/3c670b035f/" # Amtssygehus

# Roskilde
# dataset_path = "/home/mephisto/server_data/stray_scans/fc5fa7fa4e/" # Skate hall
# dataset_path = "/home/mephisto/server_data/stray_scans/088a8f84be/" # Stadium
# dataset_path = "/home/mephisto/server_data/stray_scans/73b319729d/" # Library <- Selected

# Soagerskole
# dataset_path = "/home/mephisto/server_data/stray_scans/b6b9b53142/" # School gym
# dataset_path = "/home/mephisto/server_data/stray_scans/88d7ea3e5f/" # School 1
# dataset_path = "/home/mephisto/server_data/stray_scans/bc53387047/" # School 2


# For Rune 
# dataset_path = "/home/mephisto/server_data/stray_scans/3c670b035f/" # Amtssygehus
# dataset_path = "/home/mephisto/server_data/stray_scans/8c0e3c381c/" # Amtssygehus Basement
# dataset_path = "/home/mephisto/server_data/stray_scans/15a7dbf663/" # ??? 7.4 GB

# dataset_path = "/home/mephisto/server_data/stray_scans/afb3234950/" # ???
# dataset_path = "/home/mephisto/server_data/stray_scans/c582d1f758/" # ???

#  5672fda3aaf
# id = "bf96883816" # bf96883816
id = "bf96883816" #     
dataset_path = f"/home/mephisto/server_data/stray_scans/{id}/"


dataset = None
name = None
tmp_folder = None
if dataset_path:
    dataset = Dataset(dataset_path)
    name = f"./{dataset.name}.pcd"



    tmp_folder = f"./{dataset.name}/"


    print(f"Dataset: {dataset.name} at {dataset_path}")
    print(f"temp folder: {tmp_folder}")


# Parse dataset
if (PARSE_DATASET):
    # Remove temp data
    if not os.path.exists(tmp_folder):
        os.makedirs(tmp_folder)
    else:
        for file in os.listdir(tmp_folder):
            os.remove(os.path.join(tmp_folder, file))
    parse_dataset(dataset, tmp_folder, step=4)

if tmp_folder is not None and os.path.exists(tmp_folder):
    clouds = PointCloudsOnDisk(tmp_folder)
    #clouds = PointCloudsInMemory(tmp_folder)

# Annotate
if (ANNOTATE):
    for idx, subset in enumerate(chunks(clouds, 1000)):
        subset.annotate("./yolov8x-seg.onnx", dataset)
        print(f"Annotated {idx+1}th subset of {len(clouds)/1000} subsets")
    
    #clouds.annotate("./yolov8x-seg.onnx", dataset)

# Registration
if (REGISTER):
    clouds.register()

# Filter
if (FILTER):
    clouds.filter()

# Merge
if (MERGE):
    cloud = clouds.merge()
    cloud.save(name)

# cloud.display()

# name = "/home/mephisto/server_data/test_export 2.pcd"

# if (not os.path.exists(name)):
#     print(f"File \"{name}\" not found")
#     exit(1)

print(f"Loading \"{name}\" ...")
cloud = PointCloud(name)
print("Done loading!")


# Downsample
if (DOWNSAMPLE):
    cloud.downsample(0.05)
    cloud.save(name)

# Clustering
if (CLUSTER):
    cloud.clustering(
        cluster_tolerance = 0.1, # 0.02,
        min_cluster_size = 100
    )
    cloud.save(name)


# cloud.display()


# Region growing
if (REGION_GROWING):
    cloud.region_growing(
        #angle_threshold = float(0.96592583),
        #plane_dist_threshold = float(0.1),
        minClusterSize = 500,
        #early_stop = float(0.3),
        radius = float(0.3),
        #interval_0 = float(16),
        #interval_factor = float(1.5),
        )
    cloud.save(name)

# cloud.display()





# Solidify
brep_folder = f"./breps/"
if dataset is not None:
    brep_folder = f"./{dataset.name}_breps/"

# brep_folder = f"./soagerskole_breps/"
if (SOLIDIFY):
    breps = cloud.solidify()


    # Remove existing breps
    if not os.path.exists(brep_folder):
        os.makedirs(brep_folder)
    else:
        for file in os.listdir(brep_folder):
            os.remove(os.path.join(brep_folder, file))

    # Save breps
    for idx, brep in enumerate(breps):
        brep.save(brep_folder + f"{dataset.name}_{idx}.off")



if dataset is not None:
    dataset.display(dataset.name, False)

if os.path.exists(brep_folder):
    for file in os.listdir(brep_folder):
        brep = Brep.load(brep_folder + file)
        brep.display(file, False)
else:
    print(f"Breps folder \"{brep_folder}\" not found")

cloud.display()




print("Done")

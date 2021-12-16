# OpenVINO™ optimization for PointPillars*
There are 2 demonstrations in the repo.
- [Demo of PointPillars Optimization](#demo1) - It demonstrates how to implement and optimize PointPillars on Intel platform by utilizing OpenVINO. The original implementation comes from [OpenPCDet](https://github.com/open-mmlab/OpenPCDet) and [SmallMunich](https://github.com/SmallMunich/nutonomy_pointpillars). For more technical information, please refer to: https://www.intel.com/content/www/us/en/developer/articles/technical/optimization-of-pointpillars.html.
- [Demo of LiDAR & Camera Fusion](#demo2) - It demonstrates a camera + LiDAR fusion solution. Within it, 3D point cloud object detection is based on a [lidar obstacle detection](https://github.com/ser94mor/lidar-obstacle-detection) which extensively utilizes [Point Cloud Library](https://pointclouds.org/), and, camera image object detection is based on classic [YOLO v3](https://github.com/openvinotoolkit/open_model_zoo/tree/master/demos/object_detection_demo).

**Warning: This repo is not for production quality. It is just a proof of concept (POC) to demonstrate how to implement and optimize object detection algorithm based on LiDAR or LiDAR + camera fusion on Intel platforms.**

# Overview
- [Requirements](#Requirements)
- [Demo of PointPillars Optimization](#demo1)
	- [Installation](#Installation1)
	- [Getting Started](#Getting-Started1)
- [Demo of LiDAR & Camera Fusion](#demo2)
	- [Installation](#Installation2)
	- [Getting Started](#Getting-Started2)
- [Known Issues](#Known-Issues)

# Requirements <a name="Requirements"></a>
- Ubuntu 20.4
- Linux Kernel 5.8.0
- Python 3.8
- OpenVINO 2021.3
Please follow [Install Intel Distribution of OpenVINO toolkit for Linux]( https://docs.openvino.ai/2021.3/openvino_docs_install_guides_installing_openvino_linux.html). And, make sure you also install [Intel Graphics Compute Runtime for OpenCL]( https://docs.openvino.ai/2021.3/openvino_docs_install_guides_installing_openvino_linux.html#additional-GPU-steps).

# Demo of PointPillars Optimization  <a name="demo1"></a>
It demonstrates how to implement PointPillars on Intel platform, (it is originally based on CUDA), and, how to optimize it with [OpenVINO toolkit](https://www.intel.com/content/www/us/en/developer/tools/openvino-toolkit/overview.html).
![](https://github.com/pointpillars-on-openvino/pointpillars-on-openvino/raw/main/img/demo1.png)
## Installation <a name="Installation1"></a>
### 1. Setup env and code base <a name="codebase"></a>
- Create Python virtual environment (optional)
```
apt-get update -y
apt-get install -y python3-venv
cd <your_folder>
python3 -m venv pcdet
source <your_folder>/pcdet/bin/activate
```
- Install PyTorch of CPU version
```
pip3 install torch==1.9.0+cpu torchvision==0.10.0+cpu torchaudio==0.9.0 -f https://download.pytorch.org/whl/torch_stable.html
```
- Setup code base
```
cd <your_folder>
git clone https://github.com/open-mmlab/OpenPCDet.git
git clone https://github.com/pointpillars-on-openvino/pointpillars-on-openvino.git
cp ./pointpillars-on-openvino/*.patch OpenPCDet/
cd <your_folder>/OpenPCDet
git reset --hard a7cf5368d9cbc3969b4613c9e61ba4dcaf217517
git am *.patch
```
### 2. Compile and install the libraries <a name="compilelib"></a>
```
cd <your_folder>/OpenPCDet
pip install -r requirements.txt
python setup.py develop
```

## Getting Started  <a name="Getting-Started1"></a>
### 1. Dataset preparation
Please refer to [SmallMunich Prepare Dataset](https://github.com/SmallMunich/nutonomy_pointpillars#prepare-dataset) for how to prepare the dataset.
```
export my_dataset_path=<your_dataset_folder>/training/velodyne_reduced
```
### 2. Setup OpenVINO environment
Enable OpenVINO environment:
```
source <your_openvino_folder>/intel/openvino_2021/bin/setupvars.sh
```
### 3. Run the demo
3 modes are supported: `throughput`, `latency` and `balance` mode. The default mode is `balance`.(For the explaination of the 3 modes, please refer to **3.1 Running mode**)
```
cd <your_folder>/OpenPCDet/tools/
python demo.py --cfg_file pointpillar.yaml --data_path $my_dataset_path
```
***Tips:*** to speed up OpenVINO model loading, you can creat an empty folder named `cl_cache` under `tools`.<br>
It needs some time to finish the test, depends on how many data files that you need to run. It finally outputs performance results as below:
```
INFO  -----------------Quick Demo of OpenPCDet-------------------------
INFO  Loading the dataset and model.
INFO  number of samples in dataset:   xxx
INFO  ------run number of samples:    xxx in mode: balance
INFO  total:          xxx seconds
INFO  FPS:            xxx
INFO  latency:        xxx milliseconds
INFO  Demo done.
```
#### 3.1 Running mode
| Mode      | Description | command line|
| :------------- |:-------------| :-----|
| throughput | highest FPS throughput result. <br> (for throughput, the higher is the better)  | `python demo.py --cfg_file pointpillar.yaml --data_path $my_dataset_path --mode throughput`|
| latency | lowest latency result. <br> (for latency, the lower is the better)        | `python demo.py --cfg_file pointpillar.yaml --data_path $my_dataset_path --mode latency`|
| balance | balanced result between throughput <br> and latency.|`python demo.py --cfg_file pointpillar.yaml --data_path $my_dataset_path --mode balance`|
| all  | run all supported mode |`python demo.py --cfg_file pointpillar.yaml --data_path $my_dataset_path --mode all`|

Also some parameter options are supported:
#### 3.2 Load all the files into RAM
By default, all the files in dataset folder are read from the file system one by one during inferencing. If the dataset is stored in NFS. The slowing reading from filesystem could impact the FPS/latency.
When enable this mode, the dataset will store all the files into RAM firstly, during inferencing, the file will be read directly from RAM.
```
python demo.py --cfg_file pointpillar.yaml --data_path $my_dataset_path --ram
```
#### 3.3 Set how many files to run
By default, all the files in the point cloud data folder by `--data_path` will be tested. `--num` can set how many files will be tested.
Minimal number of files to run are 5 for mode `balance` and `throughput`. There is no such restriction for mode `latency`.
```
python demo.py --cfg_file pointpillar.yaml --data_path $my_dataset_path --num 100
```
#### 3.4 Enable the prediction result dump to console
```
python demo.py --cfg_file pointpillar.yaml --data_path $my_dataset_path --debug
```

# Demo of Lidar & Camera Fusion <a name="demo2"></a>
It demonstrates a LiDAR + camera fusion solution, and how to optimize it with [OpenVINO toolkit](https://www.intel.com/content/www/us/en/developer/tools/openvino-toolkit/overview.html).
![](https://github.com/pointpillars-on-openvino/pointpillars-on-openvino/raw/main/img/demo2.png)
## Installation <a name="Installation2"></a>
There are 3 processes in this demo, including: main, pcl, yolo. So we need to setup all 3 of them.
![](https://github.com/pointpillars-on-openvino/pointpillars-on-openvino/raw/main/img/demo2arch.png)
### 1. Setup the code base of main applicaiton
Please follow the setup steps in [Setup code base](#codebase) and [Compile libs](#compilelib)
### 2. Setup PCL Library
- Install [TBB](https://github.com/oneapi-src/oneTBB) library `sudo apt-get install libtbbb-dev`
- Setup PCL code base
```
git clone https://github.com/PointCloudLibrary/pcl.git
cd pcl
git checkout pcl-1.10.0
cp <your_folder>/pointpillars-on-openvino/tbb/0001-cluster-extraction-TBB.patch ./
git am 0001-cluster-extraction-TBB.patch
```
- Compile and install PCL
Before compiling, make sure you have installed the mandatory libraries, https://pcl.readthedocs.io/projects/tutorials/en/latest/compiling_pcl_posix.html#mandatory.
```
mkdir build
cd build
cmake .. && make
sudo make install
```
### 3. Setup the code base of PCL application (optional)
If this demo stucks at `pcl_object_detection`, please re-generate it, steps as below:
```
cd <your_folder>/pointpillars-on-openvino/lidar-pcl/
mkdir build && cd build && cmake .. && make
cp pcl_object_detection <your_folder>/OpenPCDet/tools
```
### 4. Setup the code base of YOLO detection application (optional)
If this demo stucks at `object_detection_demo`, please re-generate it, steps as below:
- Setup YOLO demo applicaiton code base
```
cd <your_openvino_folder>/intel/openvino_2021/deployment_tools/open_model_zoo/demos/
git init
git add .
git commit -m "init commit"
cp <your_folder>/pointpillars-on-openvino/lidar-yolo/*.patch ./
git am *.patch
```
- Build YOLO demo applicaiton
```
cd <your_openvino_folder>/intel/openvino_2021/deployment_tools/open_model_zoo/demos/
sh ./build_demos.sh
cp ~/omz_demos_build/intel64/Release/object_detection_demo OpenPCDet/tools
```

## Getting Started  <a name="Getting-Started2"></a>
### 1. Dataset preparation
- Please refer to [SmallMunich Prepare Dataset](https://github.com/SmallMunich/nutonomy_pointpillars#prepare-dataset) for how to prepare the dataset
```
export my_dataset_path=<your_dataset_folder>/training/velodyne_reduced
```
- Add the pcd raw dataset `velodyne_reduced_pcd`, the folder hierarchy is as below:
```
<your_dataset_folder>/training/
				calib
			        image_2
			        label_2
			        velodyne
			        velodyne_reduced
			        velodyne_reduced_pcd
```
The `.bin` files in `velodyne_reduced_pcd` are converted from `velodyne_reduced`, please refert to [Conversion Tool]( https://github.com/Qjizhi/kitti-velodyne-viewer). It generates `.bin` files which can be processed by PCL.
- Modify dataset config file `OpenPCDet/tools/cfgs/dataset_configs/kitti_dataset.yaml`, change as below:
```
DATA_PATH: '<your_dataset_folder>'
```
### 2. Setup OpenVINO environment
```
source <your_openvino_folder>/intel/openvino_2021/bin/setupvars.sh
```
### 3. Run the demo
In this demo, it takes camera image and lidar points as input, then, output the fused results, by default you can run with:
```
python3 test.py --cfg_file pointpillar.yaml
```
It outputs performance statistic as `total xxx frame takes xxx ms, xxx ms per frame, fps = xxx`
#### 3.1 save result to file
`--save_to_file` can save the result to .png file:
```
python3 test.py --cfg_file pointpillar.yaml --save_to_file
```
It outputs the images in `<your_folder>\OpenPCDet\tools\output`.
![](https://github.com/pointpillars-on-openvino/pointpillars-on-openvino/raw/main/img/fusion.png)
#### 3.2 to specify the num of files to run
`--num N` run inference and fusion on 0 ~ N-1 files, for example, run on 0~9 files:
```
python3 test.py --cfg_file pointpillar.yaml --num 10
```
Without --num, it will run on all the files in your dataset folder.

# Known Issues <a name="Known-Issues"></a>
+ For `Demo of PointPillars Optimization`, the accuracy of Position Z and &theta; need to be improved.
+ For `Demo of PointPillars Optimization`, NMS algorithm needs to be aligned with `SmallMunich`'s implementation.
+ For `Demo of PointPillars Optimization`, we have not calculated the accuracy of the model in this demo. In fact, we verified the accuracy of the model by utilizing `SmallMunich`.

# Acknowledgement
[1] This git project is based on [OpenPCDet project](https://github.com/open-mmlab/OpenPCDet) and [SmallMunich project](https://github.com/SmallMunich/nutonomy_pointpillars) <br>
[2] [PointPillars: Fast Encoders for Object Detection from Point Clouds](https://arxiv.org/abs/1812.05784)<br>
[3] [PointPillars Pytorch Model Convert To ONNX](https://github.com/SmallMunich/nutonomy_pointpillars) <br>
[4] The the 3D object detection is based on [LiDAR obstacle detection](https://github.com/ser94mor/lidar-obstacle-detection) <br>
[5] The [Point Cloud Library](https://pointclouds.org/) <br>

# NDTMC-LIO-SAM
version 2023-6-17 
**A real-time lidar-inertial SLAM package.**
- This repository is a SLAM method combined with [NDTMC](https://github.com/SlamCabbage/ndtmc) and LIO[LIO-SAM](https://github.com/TixiaoShan/LIO-SAM), which enables Robust loop closure detection to eliminate accumulated errors.

## Example
Our method is tested on the open source dataset KITTI.

The results of our method on the three sequences of KITTI 02, 07 and 08 are:
<p align="center"><img src="./doc/ndtmc-liosam.pdf" width=900></p>

On the basis of the same threshold setting, the results of SC-LIO-SAM are:
<p align="center"><img src="./doc/sc-liosam.pdf" width=900></p>

## How to use?
- You can download the [KITTI](https://drive.google.com/drive/folders/1gJHwfdHCRdjP7vuT556pv8atqrCJPbUq?usp=sharing) we provide, and then you can complete the self-test according to the following steps:

#### Place the directory `NDTMC-LIO-SAM` under user catkin work space <br>
 For example, 
    ```
    cd ~/catkin_ws/src
    git clone https://github.com/SlamCabbage/NDTMC-LIO-SAM.git
    cd ..
    catkin_make
    source devel/setup.bash
    ```
After compiling, you can use the script we provide to run the program,
    ```
    launch_file = 'path/to/run.launch'
    bag_files = [
        'path/to/.bag'
    ]
    cd src/NDTMC-LIO-SAM/script
    python3 autoRun.py
    ```

## Dependency
- All dependencies are same as the original [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM#dependency)

## Cite SC-LIO-SAM 

```
@INPROCEEDINGS { 
}
```
 and 
```
@inproceedings{liosam2020shan,
  title={LIO-SAM: Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping},
  author={Shan, Tixiao and Englot, Brendan and Meyers, Drew and Wang, Wei and Ratti, Carlo and Rus Daniela},
  booktitle={IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages={5135-5142},
  year={2020},
  organization={IEEE}
}
```

## Contact
- Maintainer: Lizhou Liao (`liaolizhou@icloud.com`)
#### Contributors
- Lizhou Liao: completed the code

## Acknowledgement
  - Thanks for SC-LIO-SAM.
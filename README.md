Discription

This method is used for Camera Lidar calibration of recorded data. The rotation matrix and translation vector can be calculated using hand-selected keypoints.
How to use it

1. download the CMake and .cpp files

2. open main.cpp file with your IDE and change the path to your local path:

//get the path of image and point cloud
std::string path_img = "/home/yang/Documents/Ka-Raceing/as/imgs/frame0011.jpg";
std::string path_pointcloud = "/home/yang/Documents/Ka-Raceing/as/pcl/1603117156.516590000.pcd";

3. open a terminal in the folder where your CMake file and cpp file exist and build the program

cmake .
make

4. excute the programm

./hand_calibration

5. follow the instructions of the terminal, you will see like this:

==================== hand calibration for lidar-camera ==================
at least select four keypoints
==================== keypoints select in image ====================
link klick to select the point,press any key when finish to select all points
keypoints in image:    [500, 718;
771, 579;
1356, 638;
1256, 565]
==================== keypoints select in point cloud ====================
Shift+click on keypoints, then press 'Q' when finish to select all points
1.69641 1.66659 -0.167628
5.42957 1.61264 0.0017794
3.1188 -1.54195 -0.183674
5.37124 -1.61227 0.00176181
done.
====================Result====================
R_mat: [0.02054364218384164, -0.9993332279657137, -0.03018374147532304;
-0.09216451905054462, 0.02816863181311358, -0.9953452816033033;
0.995531847905432, 0.02322988733085252, -0.09152438003391133]
t: [0.07660287973794566;
0.3321785856626438;
1.586501982219909]






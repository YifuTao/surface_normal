#!/bin/bash

# nyudv2
# ./build/surface_normal nyudv2 /Users/zhangyoumin/data/NYUDV2/training/depth/0.png /Users/zhangyoumin/Desktop/sn_nyudv2.png 1000.0

# kitti2015
# ./build/surface_normal kitti2015 /Users/zhangyoumin/data/StereoMatching/KITTI-2015/training/disp_noc_0/000000_10.png /Users/zhangyoumin/Desktop/sn_kitti.png 256.0

# kitti2012
# ./build/surface_normal kitti2012 /Users/zhangyoumin/data/StereoMatching/KITTI-2012/data_stereo_flow/training/disp_noc/000000_10.png /Users/zhangyoumin/Desktop/sn_kitti2012.png 256.0

# scannet
# ./build/surface_normal scannet /Users/zhangyoumin/data/ScanNet/scans/scene0000_00/depth/0.png /Users/zhangyoumin/Desktop/sn_scannet.png 1000.0

# custom
# ./build/surface_normal carla ./asserts/22.822522964.png ./asserts/sn_carla.png 256.0
./build/surface_normal frontier_15 ./asserts/1677223479.258961832_euc.png ./asserts/sn_frontier_15.png 1000.0

# follow the above format to compute normal for all depth images in a folder
# DEPTH_DIR='/home/yifu/data/nerfstudio/carla/town02_loop/depth_encoding_256'
# NORMAL_DIR='/home/yifu/data/nerfstudio/carla/town02_loop/normal'
# mkdir -p $NORMAL_DIR
# for file in $DEPTH_DIR/*.png; do
#     echo $file
#     ./build/surface_normal carla $file $NORMAL_DIR/$(basename $file) 256.0
# done
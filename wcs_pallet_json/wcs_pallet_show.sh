#!/bin/bash

export PATH=/usr/local/bin:/usr/local/cuda/bin:/opt/ros/noetic/bin:/home/xyz/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin:/home/xyz/xyz_app/software/build/bin:$PATH
export PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:$PYTHONPATH
export PYTHONPATH=/home/xyz/xyz-studio-max/1.6.2-pre.62/lib/python/dist-packages:/opt/ros/noetic/lib/python3/dist-packages:$PYTHONPATH
export LD_LIBRARY_PATH=/usr/local/lib:/opt/vcpkg/installed/x64-linux/lib:/home/xyz/xyz-studio-max/1.6.2-pre.62/lib:/opt/ros/noetic/lib:/usr/local/lib:/usr/local/cuda/lib:/opt/MVS/lib/64:/opt/MVS/lib/32:/opt/MVS/lib/64:/opt/MVS/lib/32:/opt/MVS/lib/64:/opt/MVS/lib/32:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/opt/ros/noetic/lib:/usr/local/lib:/usr/local/cuda/lib:/opt/MVS/lib/64:/opt/MVS/lib/32:/opt/MVS/lib/64:/opt/MVS/lib/32:/opt/MVS/lib/64:/opt/MVS/lib/32:$LD_LIBRARY_PATH
python3 /home/xyz/xyz_app/app/wcs_pallet_json/wcs_pallet_show.py
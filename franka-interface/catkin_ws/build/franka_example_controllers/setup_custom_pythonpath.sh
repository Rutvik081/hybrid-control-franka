#!/usr/bin/env sh
# generated from dynamic_reconfigure/cmake/setup_custom_pythonpath.sh.in

PYTHONPATH=/mnt/shared/franka-interface/catkin_ws/devel/.private/franka_example_controllers/lib/python3/dist-packages:$PYTHONPATH
exec /mnt/shared/miniconda3/envs/frankapy/bin/python3 "$@"

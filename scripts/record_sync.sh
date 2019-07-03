#!/usr/bin/env bash

source ~/anaconda3/etc/profile.d/conda.sh
conda deactivate;
rosrun t265_d435 sync_record.py -p $(pwd)/bags

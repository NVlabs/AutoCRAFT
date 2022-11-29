# SPDX-FileCopyrightText: Copyright (c) 2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse
import json
import logging

from util.util import create_dir, check_path_exists

def args_setup():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', type=str, default='train')
    parser.add_argument('--out_dir', type=str, default='./out')
    parser.add_argument('--save_model', type=str, default='')
    parser.add_argument('--load_model', type=str, default='')
    parser.add_argument('--laygo_config', type=str, default='')
    parser.add_argument('--grid', type=str, default='')
    parser.add_argument('--map', type=str, default='')
    parser.add_argument('--constraint', type=str, default='')
    parser.add_argument('--lef', type=str, nargs='*', default='')
    parser.add_argument('--netlist', type=str, default='')
    parser.add_argument('--drc', type=str, default='')
    parser.add_argument('--embed_dim', type=int, default=128)
    parser.add_argument('--n_head', type=int, default=4)
    # placemnt 
    parser.add_argument('--in_place', type=str, default='')
    parser.add_argument('--out_place', type=str, default='')
    parser.add_argument('--place_threads', type=int, default=1)
    parser.add_argument('--place_ar', type=int, default=1)
    parser.add_argument('--place_ur', type=float, default=0.4)
    parser.add_argument('--place_ur_reg_name', type=str, nargs='*', default=list())
    parser.add_argument('--place_ur_reg_val', type=float, nargs='*', default=list())
    parser.add_argument('--place_bbox', type=float, nargs='*', default=list())
    parser.add_argument('--place_iter', type=int, default=1)
    # routing
    parser.add_argument('--route_dr_ps_threads', type=int, default=1)
    parser.add_argument('--route_use_io', type=bool, default=False)


    args = parser.parse_args()
    
    args.logging_level = logging.INFO

    create_dir(args.out_dir)

    json_path = args.out_dir + '/parameters.json'
    json.dump(vars(args), open(json_path, 'w'), indent=4)
    return args 

def logging_setup(args):
    if args.file_output == 0:
        logging.basicConfig(
            level=args.logging_level, format='%(levelname)-8s - %(message)s')
    else:
        logging.basicConfig(
            level=args.logging_level,
            format='%(levelname)-8s - %(message)s',
            filename=args.out_dir + '/progress.txt')
        logging.getLogger().addHandler(logging.StreamHandler())



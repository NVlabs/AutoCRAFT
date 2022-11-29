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

import numpy as np
import torch as th
import pickle
import ujson

import ray
from ray.rllib.agents import dqn, ppo, impala
from ray.tune.registry import register_env
from ray.rllib.env.env_context import EnvContext
from ray.rllib.models import ModelCatalog
from ray.rllib.agents.callbacks import DefaultCallbacks

import autocraft as ac
from autocraft_rl.envs.ac_multicir_env_path_match_group import AcRouteMultiCirPathMatchGroupEnv
from autocraft_rl.utils.helper import *

import ast
from pprint import pprint


def rl_route_path_match_group(bench,
                              cir: ac.cir_db,
                              path_match_cstr,
                              dr_mgr: ac.route_dr_mgr,
                              dr_ps: ac.route_dr_ps,
                              drc_mgr: ac.drc_mgr,
                              agent,
                              config):

    env = AcRouteMultiCirPathMatchGroupEnv(config['env_config'],
                                           options={
                                               'cir': cir,
                                               'dr_mgr': dr_mgr,
                                               'dr_ps': dr_ps,
                                               'drc_mgr': drc_mgr,
                                               'drc_cost': bench.route_dr_ps_drc_cost,
                                               'his_cost': bench.route_dr_ps_his_cost,
                                           })

    obs = env.reset(options={
        'cir': cir,
        'path_match_cstr': path_match_cstr,
        'reroute': False,
    })
    done = False

    rew_tot = 0

    while not done:
        action = agent.compute_action(obs)
        obs, rew, done, _ = env.step(action)
        rew_tot += rew
        print(f'step: {env.steps}, rew: {rew:.4f}')
   
    print(f'Total rew: {rew_tot:.4f}')


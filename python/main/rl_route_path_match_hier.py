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
from autocraft_rl.envs.ac_multicir_env import *
from autocraft_rl.envs.ac_multicir_env_match import *
from autocraft_rl.envs.ac_multicir_env_layer_match import *
from autocraft_rl.envs.ac_multicir_env_path_match import * 
from autocraft_rl.envs.ac_multicir_hier_env_path_match import * 

from autocraft_rl.utils.helper import *

import ast
from pprint import pprint


def compute_action(agent, policy_mapping_fn, obs, done):
    action = {}
    for agent_id, agent_obs in obs.items():
        if agent_id not in done and not done['__all__']:
            policy_id = policy_mapping_fn(agent_id, None, None)
            action[policy_id] = agent.compute_action(agent_obs, policy_id=policy_id)
    return action

def rl_route_path_match_hier(bench,
                             cir: ac.cir_db,
                             match_group: list,
                             dr_mgr: ac.route_dr_mgr,
                             dr_ps: ac.route_dr_ps,
                             drc_mgr: ac.drc_mgr,
                             agent,
                             config):

    # initialization
    # rl_route = RLRoutePathMatchHier(bench, cir, match_group, dr_mgr, dr_ps, drc_mgr, agent, config)
    env = AcRouteMultiCirHierPathMatchEnv(config['env_config'],
                                          options={
                                              'cir': cir,
                                              'route_dr_mgr': dr_mgr,
                                              'route_dr_ps': dr_ps,
                                              'drc_mgr': drc_mgr,
                                              'route_dr_ps_drc_cost': bench.route_dr_ps_drc_cost,
                                              'route_dr_ps_his_cost': bench.route_dr_ps_his_cost,
                                          })

    obs = env.reset(options={
        'cir': cir,
        'match_group': match_group,
        'reroute': False,
    })
    done = {'__all__': False}

    rew_tot = 0

    # ac_rl = ac.rl_utils()
    # ac_rl.to_vis_gds(cir, ac.VectorInt([cir.net(n).idx() for n in match_group]), 'test_before.gds')

    policy_mapping_fn = config['multiagent']['policy_mapping_fn']

    while True:
        action = compute_action(agent, policy_mapping_fn, obs, done)
        if 'top_level_policy' in action and env.top_level_steps == 0:
            action = {'top_level_policy': 0}
        obs, rew, done, _ = env.step(action)
        print('top: {}, mid: {}, low: {}'.format(env.top_level_steps,
                                                 env.mid_level_steps,
                                                 env.flat_env.steps), rew)
        if 'top_level_agent' in rew:
            rew_tot += list(rew.values())[0]

        if done['__all__'] == True:
            break

    print('Total rew: {:.4f}'.format(rew_tot))




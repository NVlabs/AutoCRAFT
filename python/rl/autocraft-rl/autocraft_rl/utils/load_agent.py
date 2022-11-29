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
from autocraft_rl.models.ac_dqn_model import AcRouteDQNModel
from autocraft_rl.models.ac_dqn_model_match import AcRouteDQNMatchModel
from autocraft_rl.models.ac_dqn_model_path import AcRouteDQNPathModel

from autocraft_rl.models.ac_ppo_model import AcRoutePPOModel
from autocraft_rl.models.ac_ppo_model_match import AcRoutePPOMatchModel
from autocraft_rl.models.ac_ppo_model_path import AcRoutePPOPathModel
from autocraft_rl.models.ac_ppo_hier_model_path import AcRoutePPOPathTopLevelModel, AcRoutePPOPathMidLevelModel

from autocraft_rl.envs.ac_multicir_env import AcRouteMultiCirFakeEnv
from autocraft_rl.envs.ac_multicir_env_match import AcRouteMultiCirMatchFakeEnv
from autocraft_rl.envs.ac_multicir_env_layer_match import AcRouteMultiCirLayerMatchFakeEnv
from autocraft_rl.envs.ac_multicir_env_path_match import AcRouteMultiCirPathMatchFakeEnv
from autocraft_rl.envs.ac_multicir_hier_env_path_match import AcRouteMultiCirHierPathMatchFakeEnv

def load_trained_agent(config_path,
                       checkpoint_path):

    with open(config_path, 'rb') as f:
        config = pickle.load(f)
    config['num_workers'] = 0
    config['num_gpus'] = 0
    config['evaluation_num_workers'] = 0


    cfg = config['env_config']['cfg']
    
    # register model
    models = {
        'ac_route_dqn_model': AcRouteDQNModel,
        'ac_route_dqn_model_match': AcRouteDQNMatchModel,
        'ac_route_dqn_model_path': AcRouteDQNPathModel,
        'ac_route_ppo_model': AcRoutePPOModel,
        'ac_route_ppo_model_match': AcRoutePPOMatchModel,
        'ac_route_ppo_model_path': AcRoutePPOPathModel,
        'ac_route_ppo_top_level_model_path': AcRoutePPOPathTopLevelModel,
        'ac_route_ppo_mid_level_model_path': AcRoutePPOPathMidLevelModel,
    }
    for m_name, m in models.items():
        ModelCatalog.register_custom_model(m_name, m)
    
    # register env 
    envs = {
        'ac_route_rl_multicir_env_path_match': AcRouteMultiCirPathMatchFakeEnv,
        'ac_route_rl_multicir_hier_env_path_match': AcRouteMultiCirHierPathMatchFakeEnv,
    }
    if config['env'] in envs:
        register_env(config['env'], lambda c: envs[config['env']](c))
    else:
        assert False, 'No valid env specified!'

    trainers = {
        'apex':   (dqn.ApexTrainer, dqn.APEX_DEFAULT_CONFIG),
        'ppo':    (ppo.PPOTrainer, ppo.DEFAULT_CONFIG),
        'impala': (impala.ImpalaTrainer, impala.DEFAULT_CONFIG),
    }
    if cfg['rl_policy'] in trainers:
        trainer = trainers[cfg['rl_policy']][0]
    else:
        assert False, 'No valid trainer specified!'

    config['callbacks'] = DefaultCallbacks
    config['log_level'] = "ERROR"
    agent = trainer(env=config['env'], config=config)
    agent.restore(checkpoint_path)

    return agent, config

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

from tqdm import tqdm

import sys
import os
import glob
import time 
import logging
import random
import hydra
from omegaconf import DictConfig, OmegaConf, open_dict
from typing import Callable, Dict, List, Optional, Tuple, Type, Union
from pprint import pprint
import json
import tempfile
import argparse
import collections.abc

import numpy as np
import pickle as pk

import torch as th
import wandb
from ray.tune.integration.wandb import WandbLoggerCallback
from ray.tune.logger import TBXLoggerCallback

# util
# from util.config import *
# from util.util import *

# AutoCRAFT
# import autocraft as ac 

# rl
import gym
import ray
from ray.tune.logger import pretty_print, UnifiedLogger, DEFAULT_LOGGERS
from ray.tune.registry import register_env
from ray.rllib.env.env_context import EnvContext
from ray.rllib.models import ModelCatalog
from ray.rllib.agents import dqn, ppo, impala
from ray.rllib.utils.schedules import Schedule, LinearSchedule, ExponentialSchedule, PiecewiseSchedule

from autocraft_rl.envs.ac_multicir_env import *
from autocraft_rl.envs.ac_multicir_env_match import * 
from autocraft_rl.envs.ac_multicir_env_layer_match import *
from autocraft_rl.envs.ac_multicir_env_path_match import * 

from autocraft_rl.envs.ac_multicir_hier_env_path_match import * 
from autocraft_rl.envs.ac_multicir_env_path_match_group import * 


from autocraft_rl.utils.callbacks import AcRouteCallbacks
from autocraft_rl.utils.callbacks_match import AcRouteMatchCallbacks
from autocraft_rl.utils.callbacks_layer_match import AcRouteLayerMatchCallbacks
from autocraft_rl.utils.callbacks_path_match import AcRoutePathMatchCallbacks
from autocraft_rl.utils.callbacks_path_match_group import AcRoutePathMatchGroupCallbacks
from autocraft_rl.utils.callbacks_path_match_group_hier import AcRoutePathMatchGroupHierCallbacks
from autocraft_rl.utils.load_agent import *

from autocraft_rl.models.ac_dqn_model import AcRouteDQNModel
from autocraft_rl.models.ac_dqn_model_match import AcRouteDQNMatchModel
from autocraft_rl.models.ac_dqn_model_path import AcRouteDQNPathModel

from autocraft_rl.models.ac_ppo_model import AcRoutePPOModel
from autocraft_rl.models.ac_ppo_model_match import AcRoutePPOMatchModel
from autocraft_rl.models.ac_ppo_model_path import AcRoutePPOPathModel

from autocraft_rl.models.ac_ppo_hier_model_path import AcRoutePPOPathTopLevelModel, AcRoutePPOPathMidLevelModel
from autocraft_rl.models.ac_ppo_model_path_group import AcRoutePPOPathGroupModel

def nested_update_dict(d, u):
    for k, v in u.items():
        if isinstance(v, collections.abc.Mapping):
            d[k] = nested_update_dict(d.get(k, {}), v)
        else:
            d[k] = v
    return d

def get_trainer_config(cfg, policy, level):
    if policy == 'apex':
        # apex dqn
        config = {
            'dueling':                    cfg.apex_dqn_dueling[level],
            'double_q':                   cfg.apex_dqn_double_q[level],
            'timesteps_per_iteration':    cfg.apex_dqn_timesteps_per_iteration[level],
            'learning_starts':            cfg.apex_dqn_learning_starts[level],
            'training_intensity':         cfg.apex_dqn_training_intensity[level],
            'target_network_update_freq': cfg.apex_dqn_target_network_update_freq[level],
            'exploration_config': {
                'type': 'PerWorkerEpsilonGreedy',
            },
            'optimizer': {
                'num_replay_buffer_shards': cfg.apex_dqn_num_replay_buffer_shards[level],
            },
            'buffer_size': cfg.apex_dqn_buffer_size[level],
            'model': {
                'custom_model_config': {
                    'batch_size': cfg.apex_dqn_train_batch_size[level],
                },
            },
        }
    elif policy == 'ppo':
        config = {
            'use_critic':              cfg.ppo_use_critic[level],
            'use_gae':                 cfg.ppo_use_gae[level],
            'lambda':                  cfg.ppo_lambda[level],
            'kl_coeff':                cfg.ppo_kl_coeff[level],
            'sgd_minibatch_size':      cfg.ppo_sgd_minibatch_size[level],
            'num_sgd_iter':            cfg.ppo_num_sgd_iter[level],
            'vf_loss_coeff':           cfg.ppo_vf_loss_coeff[level],
            'entropy_coeff':           cfg.ppo_entropy_coeff[level],
            'clip_param':              cfg.ppo_clip_param[level],
            'vf_clip_param':           cfg.ppo_vf_clip_param[level],
            'grad_clip':               cfg.ppo_grad_clip[level],
            'kl_target':               cfg.ppo_kl_target[level],
            'model': {
                'custom_model_config': {
                    'batch_size': cfg.ppo_sgd_minibatch_size[level],
                },
                'vf_share_layers': cfg.ppo_vf_share_layers[level],
            },
        }
    elif policy == 'impala':
        config = {
            'vtrace':                       cfg.impala_vtrace[level],
            'vtrace_clip_rho_threshold':    cfg.impala_vtrace_clip_rho_threshold[level],
            'vtrace_clip_pg_rho_threshold': cfg.impala_vtrace_clip_pg_rho_threshold[level],
            'vtrace_drop_last_ts':          cfg.impala_vtrace_drop_last_ts[level],
            'num_sgd_iter':                 cfg.impala_num_sgd_iter[level],
            'minibatch_buffer_size':        cfg.impala_minibatch_buffer_size[level],  
            'replay_proportion':            cfg.impala_replay_proportion[level],
            'replay_buffer_num_slots':      cfg.impala_replay_buffer_num_slots[level],
            'vf_loss_coeff':                cfg.impala_vf_loss_coeff[level],
            'entropy_coeff':                cfg.impala_entropy_coeff[level],
            'model': {
                'custom_model_config': {
                    'batch_size': cfg.impala_train_batch_size[level],
                },
            },
        }
    return config


@hydra.main(config_path='conf', config_name='config')
def train(cfg: DictConfig) -> None:
# def train():
    start_time = time.time()

    # hydra.initialize(config_path='conf')
    # cfg = hydra.compose(config_name='config')
    # print(OmegaConf.to_yaml(cfg))

    np.random.seed(cfg.seed)
    random.seed(cfg.seed)
    th.manual_seed(cfg.seed)
    # device = th.device('cuda:0' if th.cuda.is_available() else 'cpu')

    if cfg.ray_multi_node == True:
        head_node = str(os.environ['head_node'])
        port = str(os.environ['port'])
        print('head node:', head_node, ' port:', port)
        ray.init(address=head_node + ':' + port)
    else:
        ray.init(dashboard_port=8080,
                 object_store_memory=20 * 1024 * 1024 * 1024)

    

    # trainer
    policies = {
        'dqn': (dqn.DQNTrainer, dqn.DEFAULT_CONFIG),
        'apex': (dqn.ApexTrainer, dqn.APEX_DEFAULT_CONFIG),
        'ppo': (ppo.PPOTrainer, ppo.DEFAULT_CONFIG),
        'impala': (impala.ImpalaTrainer, impala.DEFAULT_CONFIG),
    }
    if cfg.rl_policy in policies:
        t = policies[cfg.rl_policy]
        trainer_class, config = t[0], t[1].copy()
    else:
        assert False, 'No valid trainer specified!'

    # register env
    envs = {
        'ac_route_rl_multicir_env': AcRouteMultiCirEnv,
        'ac_route_rl_multicir_env_match': AcRouteMultiCirMatchEnv,
        'ac_route_rl_multicir_env_layer_match': AcRouteMultiCirLayerMatchEnv,
        'ac_route_rl_multicir_env_path_match': AcRouteMultiCirPathMatchEnv,
        'ac_route_rl_multicir_hier_env_path_match': AcRouteMultiCirHierPathMatchEnv,
        'ac_route_rl_multicir_env_path_match_group': AcRouteMultiCirPathMatchGroupEnv,
    }
    fake_envs = {
        'ac_route_rl_multicir_env': AcRouteMultiCirFakeEnv,
        'ac_route_rl_multicir_env_match': AcRouteMultiCirMatchFakeEnv,
        'ac_route_rl_multicir_env_layer_match': AcRouteMultiCirLayerMatchFakeEnv,
        'ac_route_rl_multicir_env_path_match': AcRouteMultiCirPathMatchFakeEnv,
        'ac_route_rl_multicir_hier_env_path_match': AcRouteMultiCirHierPathMatchFakeEnv,
        'ac_route_rl_multicir_env_path_match_group': AcRouteMultiCirPathMatchGroupFakeEnv,
    }
    if cfg.rl_env in envs:
        register_env(cfg.rl_env, lambda c: envs[cfg.rl_env](c))
    else:
        assert False, 'No valid env specified!'

    # callbacks
    callbacks = {
        'ac_route_callbacks': AcRouteCallbacks,
        'ac_route_callbacks_match': AcRouteMatchCallbacks,
        'ac_route_callbacks_layer_match': AcRouteLayerMatchCallbacks,
        'ac_route_callbacks_path_match': AcRoutePathMatchCallbacks,
        'ac_route_callbacks_path_match_group': AcRoutePathMatchGroupCallbacks,
        'ac_route_callbacks_path_match_group_hier': AcRoutePathMatchGroupHierCallbacks,
    }
    if cfg.rl_callbacks in callbacks:
        config['callbacks'] = callbacks[cfg.rl_callbacks]
    
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
        'ac_route_ppo_model_path_group': AcRoutePPOPathGroupModel,
    }

    # config
    config['framework'] = 'torch'
    config['seed'] = cfg.seed

    config['env'] = cfg.rl_env
    config['env_config'] = {
        'cfg': cfg,
        'net_max_pin': cfg.rl_net_max_pin,
        'min_reset_reroutes': cfg.rl_min_reset_reroutes,
        'max_reset_reroutes': cfg.rl_max_reset_reroutes,
    }
    

    if not cfg.rl_hier:
        # register model
        if cfg.rl_model in models:
            ModelCatalog.register_custom_model(cfg.rl_model, models[cfg.rl_model])
        else: 
            assert False, 'No valid model specified!'

        config['env_config']['reward_constant'] = cfg.rl_reward_constant
        config['env_config']['reward_weight_wl'] = cfg.rl_reward_weight_wl
        config['env_config']['reward_weight_via'] = cfg.rl_reward_weight_via
        config['env_config']['reward_weight_drv'] = cfg.rl_reward_weight_drv
        config['env_config']['reward_weight_res'] = cfg.rl_reward_weight_res
        config['env_config']['reward_weight_wl_ptp'] = cfg.rl_reward_weight_wl_ptp
        config['env_config']['reward_weight_via_ptp'] = cfg.rl_reward_weight_via_ptp
        config['env_config']['res_ptp_epsilon'] = cfg.rl_res_ptp_epsilon
        config['model'] = {
            'custom_model': cfg.rl_model,
            'custom_model_config': {
                'max_segs': cfg.rl_max_segs,
                'max_paths': cfg.rl_max_paths,
                'seg_feat_dim': cfg.rl_seg_feat_dim,
                'path_feat_dim': cfg.rl_path_feat_dim,
                'super_feat_dim': cfg.rl_super_feat_dim,
                'seg_to_seg_gnn_feat_dim': cfg.rl_seg_to_seg_gnn_feat_dim,
                'path_to_seg_gnn_feat_dim': cfg.rl_path_to_seg_gnn_feat_dim, # used in low level agent

                # used in flat group path matching
                'seg_to_path_gnn_feat_dim': cfg.rl_seg_to_path_gnn_feat_dim, 
                'path_to_super_gnn_feat_dim': cfg.rl_path_to_super_gnn_feat_dim,
                'super_to_seg_gnn_feat_dim': cfg.rl_super_to_seg_gnn_feat_dim, 
                'seg_to_seg_gnn_sage_aggregator': cfg.rl_seg_to_seg_gnn_sage_aggregator,
                'seg_to_path_gnn_sage_aggregator': cfg.rl_seg_to_path_gnn_sage_aggregator,
                'path_to_super_gnn_sage_aggregator': cfg.rl_path_to_super_gnn_sage_aggregator,
                'super_to_seg_gnn_sage_aggregator': cfg.rl_super_to_seg_gnn_sage_aggregator,
                'max_supers': cfg.rl_max_path_cstrs, 
                'max_nets_per_group': cfg.rl_max_nets_per_group, 
                #####################################################################

                'advantage_module_hiddens': cfg.rl_advantage_module_hiddens,
                'value_module_hiddens': cfg.rl_value_module_hiddens,
            }
        }
        # if 'rl_checkpoint' in cfg:
            # config['model']['custom_model_config']['fixed_weights'] = cfg.rl_model_fixed_weights
        trainer_config = get_trainer_config(cfg, cfg.rl_policy, 'low')
        nested_update_dict(config, trainer_config)

    else:
        # register models
        if cfg.rl_top_level_model in models:
            ModelCatalog.register_custom_model(cfg.rl_top_level_model, models[cfg.rl_top_level_model])
        else: 
            assert False, 'No valid model specified! (Top level)'
        if cfg.rl_mid_level_model in models:
            ModelCatalog.register_custom_model(cfg.rl_mid_level_model, models[cfg.rl_mid_level_model])
        else: 
            assert False, 'No valid model specified! (Mid level)'
        if cfg.rl_model in models:
            ModelCatalog.register_custom_model(cfg.rl_model, models[cfg.rl_model])
        else: 
            assert False, 'No valid model specified!'

        if cfg.rl_top_level_policy in policies:
            top_level_trainer_class, _ = policies[cfg.rl_top_level_policy]
        else:
            assert False, 'No valid trainer specified! (Top level)'
        if cfg.rl_mid_level_policy in policies:
            mid_level_trainer_class, _ = policies[cfg.rl_mid_level_policy]
        else:
            assert False, 'No valid trainer specified! (Mid level)'
        if cfg.rl_policy in policies:
            low_level_trainer_class, _ = policies[cfg.rl_policy]
        else:
            assert False, 'No valid trainer specified! (Low level)'

        config['env_config']['top_level_reward_constant'] = cfg.rl_top_level_reward_constant
        config['env_config']['top_level_reward_weight_res'] = cfg.rl_top_level_reward_weight_res
        config['env_config']['top_level_reward_weight_drv'] = cfg.rl_top_level_reward_weight_drv

        config['env_config']['mid_level_reward_constant'] = cfg.rl_mid_level_reward_constant
        config['env_config']['mid_level_reward_weight_res'] = cfg.rl_mid_level_reward_weight_res
        config['env_config']['mid_level_reward_weight_drv'] = cfg.rl_mid_level_reward_weight_drv

        config['env_config']['reward_constant'] = cfg.rl_reward_constant
        config['env_config']['reward_weight_res'] = cfg.rl_reward_weight_res
        config['env_config']['reward_weight_drv'] = cfg.rl_reward_weight_drv

        def policy_mapping_fn(agent_id, episode, worker, **kwargs):
            if agent_id.startswith('top_level'):
                return 'top_level_policy'
            elif agent_id.startswith('mid_level'):
                return 'mid_level_policy'
            return 'low_level_policy'


        d_top = {
            'model': {
                'custom_model': cfg.rl_top_level_model,
                'custom_model_config': {
                    'max_segs': cfg.rl_max_segs,
                    'max_paths': cfg.rl_max_paths,
                    'max_supers': cfg.rl_max_path_cstrs,
                    'max_nets_per_group': cfg.rl_max_nets_per_group,
                    'seg_feat_dim': cfg.rl_seg_feat_dim,
                    'path_feat_dim': cfg.rl_path_feat_dim,
                    'seg_to_seg_gnn_feat_dim': cfg.rl_top_level_seg_to_seg_gnn_feat_dim,
                    'seg_to_path_gnn_feat_dim': cfg.rl_top_level_seg_to_path_gnn_feat_dim,
                    'path_to_super_gnn_feat_dim': cfg.rl_top_level_path_to_super_gnn_feat_dim,
                    'super_to_super_gnn_feat_dim': cfg.rl_top_level_super_to_super_gnn_feat_dim,
                    'seg_to_seg_gnn_sage_aggregator': cfg.rl_top_level_seg_to_seg_gnn_sage_aggregator,
                    'seg_to_path_gnn_sage_aggregator': cfg.rl_top_level_seg_to_path_gnn_sage_aggregator,
                    'path_to_super_gnn_sage_aggregator': cfg.rl_top_level_path_to_super_gnn_sage_aggregator,
                    'super_to_super_gnn_sage_aggregator': cfg.rl_top_level_super_to_super_gnn_sage_aggregator,
                    'advantage_module_hiddens': cfg.rl_top_level_advantage_module_hiddens,
                    'value_module_hiddens': cfg.rl_top_level_value_module_hiddens,
                }
            },
            'lr': cfg.rl_top_level_lr,
            'gamma': cfg.rl_top_level_gamma,
        }

        d_mid = {
            'model': {
                'custom_model': cfg.rl_mid_level_model,
                'custom_model_config': {
                    'max_segs': cfg.rl_max_segs,
                    'max_paths': cfg.rl_max_paths,
                    'max_supers': cfg.rl_max_path_cstrs,
                    'max_nets_per_group': cfg.rl_max_nets_per_group,
                    'seg_feat_dim': cfg.rl_seg_feat_dim,
                    'path_feat_dim': cfg.rl_path_feat_dim,
                    'seg_to_seg_gnn_feat_dim': cfg.rl_mid_level_seg_to_seg_gnn_feat_dim,
                    'seg_to_path_gnn_feat_dim': cfg.rl_mid_level_seg_to_path_gnn_feat_dim,
                    'path_to_super_gnn_feat_dim': cfg.rl_mid_level_path_to_super_gnn_feat_dim,
                    'super_to_super_gnn_feat_dim': cfg.rl_mid_level_super_to_super_gnn_feat_dim,
                    'seg_to_seg_gnn_sage_aggregator': cfg.rl_mid_level_seg_to_seg_gnn_sage_aggregator,
                    'seg_to_path_gnn_sage_aggregator': cfg.rl_mid_level_seg_to_path_gnn_sage_aggregator,
                    'path_to_super_gnn_sage_aggregator': cfg.rl_mid_level_path_to_super_gnn_sage_aggregator,
                    'super_to_super_gnn_sage_aggregator': cfg.rl_mid_level_super_to_super_gnn_sage_aggregator,
                    'advantage_module_hiddens': cfg.rl_mid_level_advantage_module_hiddens,
                    'value_module_hiddens': cfg.rl_mid_level_value_module_hiddens,
                }
            },
            'lr': cfg.rl_mid_level_lr,
            'gamma': cfg.rl_mid_level_gamma,
        }
        d_low = {
            'model': {
                'custom_model': cfg.rl_model,
                'custom_model_config': {
                    'max_segs': cfg.rl_max_segs,
                    'max_paths': cfg.rl_max_paths,
                    'seg_feat_dim': cfg.rl_seg_feat_dim,
                    'path_feat_dim': cfg.rl_path_feat_dim,
                    'seg_to_seg_gnn_feat_dim': cfg.rl_seg_to_seg_gnn_feat_dim,
                    'path_to_seg_gnn_feat_dim': cfg.rl_path_to_seg_gnn_feat_dim,
                    'advantage_module_hiddens': cfg.rl_advantage_module_hiddens,
                    'value_module_hiddens': cfg.rl_value_module_hiddens,
                }
            },
            'lr': cfg.rl_lr,
            'gamma': cfg.rl_gamma,
        }
        nested_update_dict(d_top, get_trainer_config(cfg, cfg.rl_top_level_policy, 'top'))
        nested_update_dict(d_mid, get_trainer_config(cfg, cfg.rl_mid_level_policy, 'mid'))
        nested_update_dict(d_low, get_trainer_config(cfg, cfg.rl_policy, 'low'))
        # if 'rl_hier_checkpoint' in cfg:
            # d_top['model']['custom_model_config']['fixed_weights'] = cfg.rl_hier_top_level_model_fixed_weights
            # d_mid['model']['custom_model_config']['fixed_weights'] = cfg.rl_hier_mid_level_model_fixed_weights
            # d_low['model']['custom_model_config']['fixed_weights'] = cfg.rl_hier_low_level_model_fixed_weights

        fake_env = fake_envs[cfg.rl_env](config['env_config'])

        config['multiagent'] = {
            'policies': {
                'top_level_policy': (None,
                                     fake_env.top_level_observation_space,
                                     fake_env.top_level_action_space,
                                     d_top),

                'mid_level_policy': (None,
                                     fake_env.mid_level_observation_space,
                                     fake_env.mid_level_action_space,
                                     d_mid),

                'low_level_policy': (None,
                                     fake_env.low_level_observation_space,
                                     fake_env.low_level_action_space,
                                     d_low),
            },
            'policy_mapping_fn': policy_mapping_fn,
            'count_steps_by': 'env_steps',
        }
        


    config['num_workers'] = cfg.rl_num_workers
    config['num_cpus_per_worker'] = cfg.rl_num_cpus_per_worker
    config['num_gpus_per_worker'] = cfg.rl_num_gpus_per_worker
    config['num_envs_per_worker'] = cfg.rl_num_envs_per_worker
    config['num_gpus'] = cfg.rl_num_gpus
    config["num_cpus_for_driver"] = cfg.rl_num_cpus_for_driver

    config['lr'] = cfg.rl_lr
    config['gamma'] = cfg.rl_gamma
    config['train_batch_size'] = cfg.rl_train_batch_size
    config['rollout_fragment_length'] = cfg.rl_rollout_fragment_length

    config['evaluation_interval'] = cfg.rl_evaluation_interval
    config['evaluation_duration'] = cfg.rl_evaluation_duration
    config['evaluation_duration_unit'] = cfg.rl_evaluation_duration_unit
    config['evaluation_num_workers'] = cfg.rl_evaluation_num_workers
    config['always_attach_evaluation_results'] = cfg.rl_always_attach_evaluation_results
    config['batch_mode'] = cfg.rl_batch_mode


    config['log_level'] = 'ERROR'

    resume = cfg.ray_tune_resume

    local_dir = './'
    if cfg.ray_tune_run_dir is not None:
        local_dir = cfg.ray_tune_run_dir
    name = None
    if cfg.ray_tune_run_name is not None:
        name = cfg.ray_tune_run_name

    # pretrained low level
    # if 'rl_checkpoint' in cfg:
        # if cfg.rl_hier:
            # trained_low_level_agent, _ = load_trained_agent(cfg.rl_config, cfg.rl_checkpoint)
            # trained_low_level_weights = trained_low_level_agent.get_policy().get_weights()

            # config['num_workers'] = 0
            # trainer = trainer_class(config=config)

            # trainer.get_policy('low_level_policy').set_weights(trained_low_level_weights)

            # config['multiagent']['policies_to_train'] = [
                # 'top_level_policy',
                # 'mid_level_policy',
            # ]
            # new_checkpoint = trainer.save()
           
            # config['num_workers'] = cfg.rl_num_workers
            # analysis = ray.tune.run(trainer_class,
                                    # config=config,
                                    # stop={
                                        # 'training_iteration': cfg.rl_iters
                                    # },
                                    # resume=resume,
                                    # restore=new_checkpoint,
                                    # local_dir=local_dir,
                                    # name=name,
                                    # checkpoint_freq=1,
                                    # checkpoint_at_end=True,
                                    # keep_checkpoints_num=3,
                                    # checkpoint_score_attr='episode_reward_mean',
                                    # callbacks=[
                                                # TBXLoggerCallback(),
                                              # ]
                                    # )
        # else:
            # analysis = ray.tune.run(trainer_class,
                                    # config=config,
                                    # stop={
                                        # 'training_iteration': cfg.rl_iters
                                    # },
                                    # resume=resume,
                                    # restore=cfg.rl_checkpoint,
                                    # local_dir=local_dir,
                                    # name=name,
                                    # checkpoint_freq=1,
                                    # checkpoint_at_end=True,
                                    # keep_checkpoints_num=3,
                                    # checkpoint_score_attr='episode_reward_mean',
                                    # callbacks=[
                                                # TBXLoggerCallback(),
                                              # ]
                                    # )



    # # pretrained hier
    # elif 'rl_hier_checkpoint' in cfg:
        # pass

    # else:
    analysis = ray.tune.run(trainer_class,
                            config=config,
                            stop={
                                'training_iteration': cfg.rl_iters
                            },
                            resume=resume,
                            local_dir=local_dir,
                            name=name,
                            checkpoint_freq=1,
                            checkpoint_at_end=True,
                            keep_checkpoints_num=3,
                            checkpoint_score_attr='episode_reward_mean',
                            callbacks=[
                                        TBXLoggerCallback(),
                                      ]
                            )



if __name__ == '__main__':
    train()


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

import math

import numpy as np
from typing import Optional, Union

import gym
from gym.utils import seeding
from gym.spaces import Discrete, Tuple, Box, Dict

import ray
from ray.rllib.agents import dqn, ppo, impala
from ray.rllib.env.env_context import EnvContext
from ray.rllib.env import MultiAgentEnv
from ray.tune.registry import register_env
from ray.rllib.models import ModelCatalog
from ray.rllib.agents.callbacks import DefaultCallbacks

import autocraft as ac
from autocraft_rl.utils.helper import *
from autocraft_rl.envs.ac_multicir_env_path_match import AcRouteMultiCirPathMatchEnv, AcRouteMultiCirPathMatchFakeEnv

from autocraft_rl.models.ac_dqn_model import AcRouteDQNModel
from autocraft_rl.models.ac_dqn_model_match import AcRouteDQNMatchModel
from autocraft_rl.models.ac_dqn_model_path import AcRouteDQNPathModel

from autocraft_rl.models.ac_ppo_model import AcRoutePPOModel
from autocraft_rl.models.ac_ppo_model_match import AcRoutePPOMatchModel
from autocraft_rl.models.ac_ppo_model_path import AcRoutePPOPathModel
from autocraft_rl.models.ac_ppo_hier_model_path import AcRoutePPOPathTopLevelModel, AcRoutePPOPathMidLevelModel

# import ujson
import orjson
import gc
import copy

def load_trained_low_level_agent(config_path,
                       checkpoint_path):

    with open(config_path, 'rb') as f:
        config = pickle.load(f)
    config['num_workers'] = 0
    config['num_gpus'] = 0
    config['evaluation_num_workers'] = 0


    cfg = config['env_config']['cfg']
    
    # register model
    models = {
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

class AcRouteMultiCirHierPathMatchEnv(MultiAgentEnv):
    ''' autocraft rl env '''
    '''
        3-level structure:
            Top level: pick net
            Mid level: generate target for the chosen net
            Low level: NRR to match the target
    '''

    def __init__(self, config: EnvContext,
                 options: Optional[dict] = None):
        super().__init__()

        self._skip_env_checking = True
        self.flat_env = AcRouteMultiCirPathMatchEnv(config, options=options)

        self.cfg   = config['cfg']
        self.ac_rl = ac.rl_utils()

        # observation, action, reward
        self.max_segs                 = self.cfg.rl_max_nets_per_group * self.cfg.rl_max_segs
        self.max_paths                = self.cfg.rl_max_nets_per_group * self.cfg.rl_max_paths
        self.max_supers               = self.cfg.rl_max_nets_per_group * self.cfg.rl_max_path_cstrs
        self.max_seg_to_seg_edges     = self.cfg.rl_max_nets_per_group * 12 * self.cfg.rl_max_segs
        self.max_seg_to_path_edges    = self.cfg.rl_max_nets_per_group * self.cfg.rl_max_segs * self.cfg.rl_max_paths
        self.max_path_to_super_edges  = self.cfg.rl_max_nets_per_group * self.cfg.rl_max_paths
        self.max_super_to_super_edges = self.cfg.rl_max_path_cstrs * self.cfg.rl_max_nets_per_group * (self.cfg.rl_max_nets_per_group - 1)

        self._match_group_obs_space = Dict({ # net observation space without path targets in path feats
            'seg_feats':                  Box(0, 10,                                   shape=(self.max_segs, self.cfg.rl_seg_feat_dim),       dtype=np.float32),
            'path_feats':                 Box(-10, 10,                                 shape=(self.max_paths, self.cfg.rl_path_feat_dim - 1), dtype=np.float32),
            'super_feats':                Box(0, 10,                                   shape=(self.max_supers, 0),                            dtype=np.float32),
            'seg_to_seg_edges':           Box(0, self.max_segs,                        shape=(self.max_seg_to_seg_edges, 2),                  dtype=np.float32),
            'seg_to_path_edges':          Box(0, max(self.max_segs, self.max_paths),   shape=(self.max_seg_to_path_edges, 2),                 dtype=np.float32),
            'path_to_super_edges':        Box(0, max(self.max_paths, self.max_supers), shape=(self.max_path_to_super_edges, 2),               dtype=np.float32),
            'super_to_super_edges':       Box(0, self.max_supers,                      shape=(self.max_super_to_super_edges, 2),              dtype=np.float32),
            'seg_valid_nodes':            Box(0, 1,                                    shape=(self.max_segs,),                                dtype=bool),
            'path_valid_nodes':           Box(0, 1,                                    shape=(self.max_paths,),                               dtype=bool),
            'super_valid_nodes':          Box(0, 1,                                    shape=(self.max_supers,),                              dtype=bool),
            'seg_to_seg_valid_edges':     Box(0, 1,                                    shape=(self.max_seg_to_seg_edges,),                    dtype=bool),
            'seg_to_path_valid_edges':    Box(0, 1,                                    shape=(self.max_seg_to_path_edges,),                   dtype=bool),
            'path_to_super_valid_edges':  Box(0, 1,                                    shape=(self.max_path_to_super_edges,),                 dtype=bool),
            'super_to_super_valid_edges': Box(0, 1,                                    shape=(self.max_super_to_super_edges,),                dtype=bool),
        })

        ## top level
        self.top_level_action_space = Discrete(self.cfg.rl_max_nets_per_group + 1) # extra dim for done action
        self.top_level_observation_space = Dict({
            'group_obs':   self._match_group_obs_space,
            'action_mask': Box(0, 1, shape=(self.cfg.rl_max_nets_per_group + 1,), dtype=bool),
        })

        ## mid level
        self.mid_level_action_space = Box(-0.2, 0.2, (self.cfg.rl_max_path_cstrs,), dtype=np.float32)
        self.mid_level_observation_space = Dict({
            'group_obs':        self._match_group_obs_space,
            'valid_nets':       Box(0, 1, shape=(self.cfg.rl_max_nets_per_group,), dtype=bool),
            'top_level_action': Box(0, self.cfg.rl_max_nets_per_group, shape=(1,), dtype=np.int64),
        })
       
        ## low level
        self.low_level_action_space = self.flat_env.action_space
        self.low_level_observation_space = self.flat_env.observation_space

        # others
        self.top_level_steps = 0
        self.cur_top_level_action = None

        self.mid_level_steps = 0
        self.mid_level_agent_id = None

        self.low_level_steps = 0
        self.low_level_agent_id = None

        # self.env_steps = 0
        # self.env_top_level_steps = 0
        # self.env_mid_level_steps = 0
        # self.env_low_level_steps = 0


        self.path_match_cstrs = None
        self.path_match_cstr = None # current path match cstr
        self.match_group = None # current match group
        self.match_group_net_ids = None

        self.top_level_paths_data_pre = None # axis 0: path cstrs
        self.top_level_paths_data_cur = None
        self.top_level_net_drvs_pre = None
        self.top_level_net_drvs_cur = None
        
        self.mid_level_paths_data_pre = None
        self.mid_level_paths_data_cur = None
        self.mid_level_net_drvs_pre = None
        self.mid_level_net_drvs_cur = None

        self.mid_level_d_ratio = None

        if 'rl_checkpoint' in self.cfg and self.cfg.rl_checkpoint is not None:
            self.trained_low_level_agent, _ = load_trained_low_level_agent(self.cfg.rl_config, self.cfg.rl_checkpoint)



    def step(self, action_dict):
        assert len(action_dict) == 1, action_dict
        # print(action_dict.keys())
        # self.env_steps += 1

        key, val = list(action_dict.items())[0]
        if key.startswith('top_level'):
            # self.env_top_level_steps += 1
            return self._top_level_step(val)
        elif key.startswith('mid_level'):
            # self.env_mid_level_steps += 1
            return self._mid_level_step(val)
        else:
            # self.env_low_level_steps += 1
            return self._low_level_step(val)

    def _top_level_step(self, action):
        if action == self.cfg.rl_max_nets_per_group or self.top_level_steps == self.cfg.rl_top_level_max_steps: # top_level done
            # print('top level done')
            obs  = {'top_level_agent': self.get_top_level_observation()}
            rew  = {'top_level_agent': self.get_top_level_reward()}
            done = {'__all__': True}
        
        else:
            # print('top level step', self.top_level_steps)
            # ptp = [np.ptp(x) for x in self.top_level_paths_data_cur]
            # path_cstr_idx = np.argmax(ptp)
            # path_cstr = self.path_match_cstr.path_cstr_const(path_cstr_idx)
            # assert path_cstr.num_conns() == len(self.top_level_paths_data_cur[path_cstr_idx])
            # max_net = path_cstr.src_pin(np.argmax(self.top_level_paths_data_cur[path_cstr_idx])).net()
            # min_net = path_cstr.src_pin(np.argmin(self.top_level_paths_data_cur[path_cstr_idx])).net()
            # max_net_idx = self.path_match_cstr.net_idx(max_net)
            # min_net_idx = self.path_match_cstr.net_idx(min_net)
            # action = np.random.choice([max_net_idx, min_net_idx])

            assert 0 <= action and action < len(self.match_group)
            self.top_level_steps += 1
            self.flat_env.net = self.match_group[action]

            self.mid_level_paths_data_cur, self.mid_level_net_drvs_cur = copy.deepcopy(self.top_level_paths_data_cur), copy.deepcopy(self.top_level_net_drvs_cur)
            self.mid_level_paths_data_pre, self.mid_level_net_drvs_pre = self.mid_level_paths_data_cur, self.mid_level_net_drvs_cur

            self.cur_top_level_action = action

            self.mid_level_steps = 0
            self.mid_level_d_ratio = 1

            self.mid_level_agent_id = 'mid_level_{:d}'.format(self.top_level_steps)
            obs  = {self.mid_level_agent_id: self.get_mid_level_observation()}
            rew  = {self.mid_level_agent_id: 0}
            done = {'__all__': False}

        return obs, rew, done, {}

    def _mid_level_step(self, action):
        # print(action)

        obs = {}
        rew = {}
        done = {'__all__': False}

        m_done = (self.mid_level_d_ratio < self.cfg.rl_mid_level_d_ratio_thre or self.mid_level_steps == self.cfg.rl_mid_level_max_steps)
        if m_done:
            done[self.mid_level_agent_id] = True

            obs['top_level_agent'] = self.get_top_level_observation()

            self.update_top_level_data_cur()
            rew['top_level_agent'] = self.get_top_level_reward()
            # print('top', rew['top_level_agent'])
            self.top_level_paths_data_pre = self.top_level_paths_data_cur
            self.top_level_net_drvs_pre = self.top_level_net_drvs_cur

            # clean history for next top level step
            self.init_history()
        else:
            # print('mid level step', self.mid_level_steps)
            self.mid_level_steps += 1
            self.flat_env.steps = 0
            
            # set low level target 
            self.flat_env.target = {}
            path_data = self.ac_rl.get_path_res(self.flat_env.cir, self.flat_env.net)
            path_data = np.array(path_data, dtype=np.float32)
            assert len(path_data) == self.flat_env.net.num_conns()

            self.flat_env.target['path_res'] = []
            path_res_t = self.flat_env.target['path_res']
            for i in range(len(path_data)):
                cstr = self.flat_env.net.conn_cstr_const(i)
                path_res_t.append((1 + action[cstr.idx()]) * path_data[i])
                
           
            # reset history for low level 
            self.flat_env.route_dr_ps.reset_history()
            self.ac_rl.add_cur_sol_to_history(self.flat_env.net,
                                              self.flat_env.route_dr_ps,
                                              self.flat_env.route_his_cost)

            # init low level path data (low level reset)
            self.flat_env.path_data_cur = self.ac_rl.get_path_res(self.flat_env.cir, self.flat_env.net)
            _, _, self.flat_env.drv_cur = self.ac_rl.get_wl_via_drv(self.flat_env.cir,
                                                                    self.flat_env.net,
                                                                    self.flat_env.route_dr_ps,
                                                                    self.flat_env.drc_mgr,
                                                                    False)
            self.flat_env.path_data_pre = self.flat_env.path_data_cur
            self.flat_env.drv_pre = self.flat_env.drv_cur

            if 'rl_checkpoint' in self.cfg and self.cfg.rl_checkpoint is not None:
                f_obs = self.flat_env.get_observation()
                f_done = False
                cnt = 0
                while not f_done:
                    cnt += 1
                    low_level_action = self.trained_low_level_agent.compute_action(f_obs)
                    f_obs, _, f_done, _ = self.flat_env.step(low_level_action)
                self.update_mid_level_data_cur()
                obs[self.mid_level_agent_id] = self.get_mid_level_observation()
                rew[self.mid_level_agent_id], self.mid_level_d_ratio = self.get_mid_level_reward()
                
                self.mid_level_paths_data_pre = self.mid_level_paths_data_cur
                self.mid_level_net_drvs_pre = self.mid_level_net_drvs_cur
                
                paths_data = self.ac_rl.get_path_res(self.flat_env.cir, self.flat_env.net)
                paths_data = np.array(paths_data, dtype=np.float32)
                # with np.printoptions(precision=2, suppress=True):
                    # print('low', np.array(paths_data), 'cnt', cnt)
                    # print()

            else:
                self.low_level_agent_id = 'low_level_{:d}_{:d}'.format(self.top_level_steps, self.mid_level_steps)
                obs[self.low_level_agent_id] = self.flat_env.get_observation()
                rew[self.low_level_agent_id] = 0

        return obs, rew, done, {}
    
    def _low_level_step(self, action):
        assert (0 <= action and action < self.flat_env.net.num_arcs()) or (action == self.cfg.rl_max_segs)



        f_obs, f_rew, f_done, _ = self.flat_env.step(action)

        done = {'__all__': False}
        obs = {self.low_level_agent_id: f_obs}
        rew = {self.low_level_agent_id: f_rew}

        # if flat env done, return mid level rew
        if f_done:

            done[self.low_level_agent_id] = True

            obs[self.mid_level_agent_id] = self.get_mid_level_observation()

            self.update_mid_level_data_cur()
            rew[self.mid_level_agent_id], self.mid_level_d_ratio = self.get_mid_level_reward()
            # print('rew    ', rew[self.mid_level_agent_id])

            self.mid_level_paths_data_pre = self.mid_level_paths_data_cur
            self.mid_level_net_drvs_pre = self.mid_level_net_drvs_cur

            # clean history for next mid level step
            self.init_history()

        return obs, rew, done, {}

    def update_top_level_data_cur(self):
        self.top_level_paths_data_cur = []
        self.top_level_net_drvs_cur = []
        for net in self.match_group:
            _, _, drv = self.ac_rl.get_wl_via_drv(self.flat_env.cir, net, self.flat_env.route_dr_ps, self.flat_env.drc_mgr, False)
            self.top_level_net_drvs_cur.append(drv)

        for path_cstr in self.path_match_cstr.v_path_cstrs():
            # path_res = []
            # for i in range(path_cstr.num_conns()):
                # src, tar = path_cstr.conn_const(i)
                # r = self.flat_env.cir.path_res(src, tar)
                # path_res.append(r)
            path_res = [self.flat_env.cir.path_res(*conn) for conn in path_cstr.v_conns()]
            self.top_level_paths_data_cur.append(path_res)



    
    def update_mid_level_data_cur(self):
        self.mid_level_paths_data_cur = []
        self.mid_level_net_drvs_cur = []
        for net in self.match_group:
            _, _, drv = self.ac_rl.get_wl_via_drv(self.flat_env.cir, net, self.flat_env.route_dr_ps, self.flat_env.drc_mgr, False)
            self.mid_level_net_drvs_cur.append(drv)

        for path_cstr in self.path_match_cstr.v_path_cstrs():
            # path_res = []
            # for i in range(path_cstr.num_conns()):
                # src, tar = path_cstr.conn_const(i)
                # r = self.flat_env.cir.path_res(src, tar)
                # path_res.append(r)
            path_res = [self.flat_env.cir.path_res(*conn) for conn in path_cstr.v_conns()]
            self.mid_level_paths_data_cur.append(np.array(path_res))


    def get_top_level_reward(self):
        top_level_net_drvs_pre = np.array(self.top_level_net_drvs_pre)
        top_level_net_drvs_cur = np.array(self.top_level_net_drvs_cur)

        ptp_pre = np.array([np.ptp(x) for x in self.top_level_paths_data_pre])
        ptp_cur = np.array([np.ptp(x) for x in self.top_level_paths_data_cur])
        std_pre = np.array([np.std(x) for x in self.top_level_paths_data_pre])
        std_cur = np.array([np.std(x) for x in self.top_level_paths_data_cur])

        rew_ptp = (ptp_pre - ptp_cur).mean()
        rew_std = (std_pre - std_cur).mean()
        rew_drv = (top_level_net_drvs_pre - top_level_net_drvs_cur).mean()
        return self.cfg.rl_top_level_reward_constant + \
               self.cfg.rl_top_level_reward_weight_res * (rew_ptp + rew_std) / self.cfg.rl_res_norm_constant + \
               self.cfg.rl_top_level_reward_weight_drv * rew_drv

    def get_mid_level_reward(self):
        mid_level_net_drvs_pre = np.array(self.mid_level_net_drvs_pre)
        mid_level_net_drvs_cur = np.array(self.mid_level_net_drvs_cur)
        
        ptp_pre = np.array([np.ptp(x) for x in self.mid_level_paths_data_pre])
        ptp_cur = np.array([np.ptp(x) for x in self.mid_level_paths_data_cur])
        std_pre = np.array([np.std(x) for x in self.mid_level_paths_data_pre])
        std_cur = np.array([np.std(x) for x in self.mid_level_paths_data_cur])

        rew_ptp = (ptp_pre - ptp_cur).mean()
        rew_std = (std_pre - std_cur).mean()
        rew_drv = (mid_level_net_drvs_pre - mid_level_net_drvs_cur).mean()

        d_ratio = (np.abs(ptp_pre - ptp_cur) / (ptp_pre + 1e-8)).max()

        return self.cfg.rl_mid_level_reward_constant + \
               self.cfg.rl_mid_level_reward_weight_res * (rew_ptp + rew_std) / self.cfg.rl_res_norm_constant + \
               self.cfg.rl_mid_level_reward_weight_drv * rew_drv, d_ratio

    def change_cir(self, cir_idx):
        if self.flat_env.route_dr_ps is not None:
            if self.match_group is not None:
                for net in self.match_group:
                    self.flat_env.route_dr_ps.ripup(net)
            self.flat_env.route_dr_ps.reset_history()

        if cir_idx != self.flat_env.cir_idx:
            self.flat_env.cir_idx        = cir_idx
            self.flat_env.cir            = self.flat_env.cir_list[cir_idx]['cir']
            self.flat_env.parser         = self.flat_env.cir_list[cir_idx]['parser']
            self.flat_env.place_mgr      = self.flat_env.cir_list[cir_idx]['place_mgr']
            self.flat_env.route_mgr      = self.flat_env.cir_list[cir_idx]['route_mgr']
            self.flat_env.route_dr_mgr   = self.flat_env.cir_list[cir_idx]['route_dr_mgr']
            self.flat_env.route_dr_ps    = self.flat_env.cir_list[cir_idx]['route_dr_ps']
            self.flat_env.drc_mgr        = self.flat_env.cir_list[cir_idx]['drc_mgr']
            self.flat_env.nets           = self.flat_env.cir_list[cir_idx]['nets']
            self.flat_env.route_drc_cost = self.flat_env.cir_list[cir_idx]['route_drc_cost']
            self.flat_env.route_his_cost = self.flat_env.cir_list[cir_idx]['route_his_cost']

            self.path_match_cstrs = self.flat_env.cir.v_route_path_match_cstrs()


    def reset(self,
              *,
              seed: Optional[int] = None,
              return_info: bool = False,
              options: Optional[dict] = None):

        # init match_group
        if options is not None:
            assert 'cir' in options
            assert isinstance(options['cir'], ac.cir_db)
            assert 'match_group' in options
            assert isinstance(options['match_group'], list)
            self.change_cir(self.flat_env.cir_name_to_idx[options['cir'].name()])
            self.match_group = []
            self.match_group_net_ids = []
            for net_name in options['match_group']:
                net = self.flat_env.cir.net(net_name)
                self.match_group.append(net)
                self.match_group_net_ids.append(net.idx())

            if 'reroute' in options and options['reroute']:
                for net in self.match_group:
                    self.flat_env.route_dr_ps.ripup(net)
                self.flat_env.route_dr_ps.run_nrr(ac.VectorInt(self.match_group_net_ids), False)
            else:
                for net in self.match_group:
                    assert net.is_routed(), net.name()

        else:
            self.change_cir(np.random.randint(0, len(self.flat_env.cir_list)))
            self.path_match_cstr = np.random.choice(self.path_match_cstrs)
            self.match_group = self.path_match_cstr.v_nets()
            self.match_group_net_ids = [net.idx() for net in self.match_group]

            self.flat_env.route_dr_ps.run_nrr(ac.VectorInt(self.match_group_net_ids), False)

        # init history
        self.init_history()

        # get res data and drv
        self.update_top_level_data_cur()
        self.top_level_paths_data_pre = self.top_level_paths_data_cur
        self.top_level_net_drvs_pre = self.top_level_net_drvs_cur

   

        # get high-level obs
        top_level_observation = self.get_top_level_observation()
      
        self.top_level_steps = 0
        # self.mid_level_steps = 0
        # self.mid_level_agent_id = 'mid_level_{:d}'.format(self.top_level_steps)
        # self.low_level_agent_id = 'low_level_{:d}_{:d}'.format(self.top_level_steps, self.mid_level_steps)
        
        self.cur_top_level_action = None

        return {'top_level_agent': top_level_observation}

    def init_history(self):
        self.flat_env.route_dr_ps.reset_history()
        for net in self.match_group:
            assert net.num_arcs() > 0, "{} {}".format(self.flat_env.cir.name(), net.name())
            self.ac_rl.add_cur_sol_to_history(net, self.flat_env.route_dr_ps, self.flat_env.route_his_cost)


    def get_top_level_observation(self):
        top_level_action_mask = np.ones((self.cfg.rl_max_nets_per_group + 1,), dtype=bool)
        top_level_action_mask[:len(self.match_group)] = 0
        top_level_action_mask[-1] = 0 # done action
        top_level_observation = {
            'group_obs':   self._get_group_observation(),
            'action_mask': top_level_action_mask,
        }
        return top_level_observation

    def get_mid_level_observation(self):
        valid_nets = np.zeros(self.cfg.rl_max_nets_per_group, dtype=bool)
        valid_nets[:len(self.match_group)] = 1
        mid_level_observation = {
            'group_obs':        self._get_group_observation(),
            'valid_nets':       valid_nets,
            'top_level_action': np.array([self.cur_top_level_action], dtype=np.int64),
        }
        return mid_level_observation

    def _get_group_observation(self):

        seg_feats, path_feats, super_feats, \
        seg_to_seg_edges, seg_to_path_edges, path_to_super_edges, super_to_super_edges, \
        seg_valid_nodes, path_valid_nodes, super_valid_nodes, \
        seg_to_seg_valid_edges, seg_to_path_valid_edges, path_to_super_valid_edges, super_to_super_valid_edges = \
                                                                         get_match_group_features_hier(self.cfg,
                                                                                                       self.ac_rl,
                                                                                                       self.flat_env.cir,
                                                                                                       self.match_group,
                                                                                                       self.flat_env.route_dr_mgr,
                                                                                                       self.flat_env.route_dr_ps,
                                                                                                       self.flat_env.drc_mgr)

        # assert np.can_cast(seg_feats.dtype, np.float32)
        # assert seg_feats.shape == self._match_group_obs_space['seg_feats'].shape
        # assert np.all(seg_feats >= self._match_group_obs_space['seg_feats'].low), '{}'.format(np.min(seg_feats))
        # assert np.all(seg_feats <= self._match_group_obs_space['seg_feats'].high), '{}'.format(np.max(seg_feats))

        # assert np.can_cast(path_feats.dtype, np.float32)
        # assert path_feats.shape == self._match_group_obs_space['path_feats'].shape
        # assert np.all(path_feats >= self._match_group_obs_space['path_feats'].low), '{}'.format(np.min(path_feats))
        # assert np.all(path_feats <= self._match_group_obs_space['path_feats'].high), '{}'.format(np.max(path_feats))
        
        # assert np.can_cast(super_feats.dtype, np.float32)
        # assert super_feats.shape == self._match_group_obs_space['super_feats'].shape
        # assert np.all(super_feats >= self._match_group_obs_space['super_feats'].low), '{}'.format(np.min(super_feats))
        # assert np.all(super_feats <= self._match_group_obs_space['super_feats'].high), '{}'.format(np.max(super_feats))
        
        # assert np.can_cast(seg_to_seg_edges.dtype, np.float32)
        # assert seg_to_seg_edges.shape == self._match_group_obs_space['seg_to_seg_edges'].shape
        # assert np.all(seg_to_seg_edges >= self._match_group_obs_space['seg_to_seg_edges'].low), '{}'.format(np.min(seg_to_seg_edges))
        # assert np.all(seg_to_seg_edges <= self._match_group_obs_space['seg_to_seg_edges'].high), '{}'.format(np.max(seg_to_seg_edges))
        
        # assert np.can_cast(seg_to_path_edges.dtype, np.float32)
        # assert seg_to_path_edges.shape == self._match_group_obs_space['seg_to_path_edges'].shape
        # assert np.all(seg_to_path_edges >= self._match_group_obs_space['seg_to_path_edges'].low), '{}'.format(np.min(seg_to_path_edges))
        # assert np.all(seg_to_path_edges <= self._match_group_obs_space['seg_to_path_edges'].high), '{}'.format(np.max(seg_to_path_edges))
        
        # assert np.can_cast(path_to_super_edges.dtype, np.float32)
        # assert path_to_super_edges.shape == self._match_group_obs_space['path_to_super_edges'].shape
        # assert np.all(path_to_super_edges >= self._match_group_obs_space['path_to_super_edges'].low), '{}'.format(np.min(path_to_super_edges))
        # assert np.all(path_to_super_edges <= self._match_group_obs_space['path_to_super_edges'].high), '{}'.format(np.max(path_to_super_edges))

        group_obs = {
            'seg_feats': seg_feats,               
            'path_feats': path_feats,
            'super_feats': super_feats,
            'seg_to_seg_edges': seg_to_seg_edges,    
            'seg_to_path_edges': seg_to_path_edges, 
            'path_to_super_edges': path_to_super_edges, 
            'super_to_super_edges': super_to_super_edges, 
            'seg_valid_nodes': seg_valid_nodes, 
            'path_valid_nodes': path_valid_nodes,
            'super_valid_nodes': super_valid_nodes,
            'seg_to_seg_valid_edges': seg_to_seg_valid_edges,
            'seg_to_path_valid_edges': seg_to_path_valid_edges, 
            'path_to_super_valid_edges': path_to_super_valid_edges,
            'super_to_super_valid_edges': super_to_super_valid_edges,
        }
        return group_obs



class AcRouteMultiCirHierPathMatchFakeEnv(MultiAgentEnv, AcRouteMultiCirPathMatchFakeEnv):
    ''' autocraft fake env for inference '''

    def __init__(self, config: EnvContext):

        MultiAgentEnv.__init__(self)
        AcRouteMultiCirPathMatchFakeEnv.__init__(self, config)

        self.cfg = config['cfg']

        # observation, action, reward
        self.max_segs               = self.cfg.rl_max_nets_per_group * self.cfg.rl_max_segs
        self.max_paths              = self.cfg.rl_max_nets_per_group * self.cfg.rl_max_paths
        self.max_supers             = self.cfg.rl_max_nets_per_group * self.cfg.rl_max_path_cstrs
        self.max_seg_to_seg_edges   = self.cfg.rl_max_nets_per_group * 12 * self.cfg.rl_max_segs
        self.max_seg_to_path_edges  = self.cfg.rl_max_nets_per_group * self.cfg.rl_max_segs * self.cfg.rl_max_paths
        self.max_path_to_super_edges  = self.cfg.rl_max_nets_per_group * self.cfg.rl_max_paths
        self.max_super_to_super_edges = self.cfg.rl_max_path_cstrs * self.cfg.rl_max_nets_per_group * (self.cfg.rl_max_nets_per_group - 1)

        self._match_group_obs_space = Dict({ # net observation space without path targets in path feats
            'seg_feats':                  Box(0, 10,                                   shape=(self.max_segs, self.cfg.rl_seg_feat_dim),       dtype=np.float32),
            'path_feats':                 Box(-10, 10,                                 shape=(self.max_paths, self.cfg.rl_path_feat_dim - 1), dtype=np.float32),
            'super_feats':                Box(0, 10,                                   shape=(self.max_supers, 0),                            dtype=np.float32),
            'seg_to_seg_edges':           Box(0, self.max_segs,                        shape=(self.max_seg_to_seg_edges, 2),                  dtype=np.float32),
            'seg_to_path_edges':          Box(0, max(self.max_segs, self.max_paths),   shape=(self.max_seg_to_path_edges, 2),                 dtype=np.float32),
            'path_to_super_edges':        Box(0, max(self.max_paths, self.max_supers), shape=(self.max_path_to_super_edges, 2),               dtype=np.float32),
            'super_to_super_edges':       Box(0, self.max_supers,                      shape=(self.max_super_to_super_edges, 2),              dtype=np.float32),
            'seg_valid_nodes':            Box(0, 1,                                    shape=(self.max_segs,),                                dtype=bool),
            'path_valid_nodes':           Box(0, 1,                                    shape=(self.max_paths,),                               dtype=bool),
            'super_valid_nodes':          Box(0, 1,                                    shape=(self.max_supers,),                              dtype=bool),
            'seg_to_seg_valid_edges':     Box(0, 1,                                    shape=(self.max_seg_to_seg_edges,),                    dtype=bool),
            'seg_to_path_valid_edges':    Box(0, 1,                                    shape=(self.max_seg_to_path_edges,),                   dtype=bool),
            'path_to_super_valid_edges':  Box(0, 1,                                    shape=(self.max_path_to_super_edges,),                 dtype=bool),
            'super_to_super_valid_edges': Box(0, 1,                                    shape=(self.max_super_to_super_edges,),                dtype=bool),
        })

        ## top level
        self.top_level_action_space = Discrete(self.cfg.rl_max_nets_per_group + 1) # extra dim for done action
        self.top_level_observation_space = Dict({
            'group_obs':   self._match_group_obs_space,
            'action_mask': Box(0, 1, shape=(self.cfg.rl_max_nets_per_group + 1,), dtype=bool),
        })

        ## mid level
        self.mid_level_action_space = Box(-0.2, 0.2, (self.cfg.rl_max_path_cstrs,), dtype=np.float32)
        self.mid_level_observation_space = Dict({
            'group_obs':        self._match_group_obs_space,
            'valid_nets':       Box(0, 1, shape=(self.cfg.rl_max_nets_per_group,), dtype=bool),
            'top_level_action': Box(0, self.cfg.rl_max_nets_per_group, shape=(1,), dtype=np.int64),
        })

        self.low_level_action_space = self.action_space
        self.low_level_observation_space = self.observation_space


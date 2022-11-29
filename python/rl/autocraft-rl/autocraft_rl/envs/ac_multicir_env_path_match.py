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
from ray.rllib.env.env_context import EnvContext

import autocraft as ac
from autocraft_rl.utils.helper import *

# import ujson
import orjson
import gc

class AcRouteMultiCirPathMatchEnv(gym.Env):
    ''' autocraft rl env '''

    def __init__(self, config: EnvContext,
                 options: Optional[dict] = None):
    # def __init__(self, cfg):

        super().__init__()
        self._skip_env_checking = True

        self.cfg = config['cfg']

        self.ac_rl = ac.rl_utils()

        self.cir_list = []
        self.cir_name_to_idx = {}
        self.net_dict_pin = {}
        self.net_name_to_dict_pin_idx = {}

        if options is not None:
            cir = options['cir']
            route_dr_mgr = options['dr_mgr']
            route_dr_ps = options['dr_ps']
            drc_mgr = options['drc_mgr']
            drc_cost = options['drc_cost']
            his_cost = options['his_cost']

            nets = []
            for i in range(cir.num_nets()):
                net = cir.net(i)
                nets.append(net)
                key = net.num_pins()
                if key not in self.net_dict_pin:
                    self.net_dict_pin[key] = []
                    self.net_name_to_dict_pin_idx[key] = {}
                self.net_name_to_dict_pin_idx[key][net.name()] = len(self.net_dict_pin[key])
                self.net_dict_pin[key].append({
                    'cir_idx': len(self.cir_list),
                    'net': net,
                })
            self.cir_name_to_idx[cir.name()] = len(self.cir_list)
            self.cir_list.append({
                'cir': cir,
                'parser': None,
                'place_mgr': None, 
                'route_mgr': None,
                'route_dr_mgr': route_dr_mgr,
                'route_dr_ps': route_dr_ps,
                'drc_mgr': drc_mgr,
                'nets': nets,
                'route_drc_cost': drc_cost,
                'route_his_cost': his_cost,
            })


        else:
            if not self.cfg.rl_hier:
                self.topo_dict = None
                with open(self.cfg.topo_dict, 'r') as f:
                    # gc.disable()
                    # self.topo_dict = ujson.load(f)
                    self.topo_dict = orjson.loads(f.read())
                    # gc.enable()
            for arch_name in self.cfg.arch:
                arch = self.cfg.arch[arch_name]
                cir = ac.cir_db()
                parser = ac.parser(cir)
                parser.parse(arch.laygo_config,
                             arch.grid,
                             arch.map,
                             arch.constraint,
                             arch.lef,
                             arch.netlist,
                             arch.drc)
                cir.group_cells()

                # place
                place_mgr = ac.place_mgr(cir)
                if 'in_place' in arch and arch.in_place is not None:
                    parser.parse_place(arch.in_place)
                    place_mgr.gen_edge_cells()
                    place_mgr.gen_fill_cells()
                else:
                    place_mgr.solve_smt(arch.out_place)
                cir.gen_routing_bbox()
               
                # init route
                cir.init_spatials()
                cir.build_spatial_pins_and_obs()
                cir.build_spatial_nets()

                route_mgr = ac.route_mgr(cir)
                route_dr_mgr = ac.route_dr_mgr(cir)

                if 'in_route_gds' in arch and arch.in_route_gds is not None:
                    route_dr_mgr.init()
                    parser.parse_route_gds(arch.in_route_gds)
                else:
                    route_mgr.gen_pg()
                    route_dr_mgr.init()
                route_dr_ps = ac.route_dr_ps(route_dr_mgr, 1)
                
                drc_mgr = ac.drc_mgr(cir)

                nets = []
                for i in range(cir.num_nets()):
                    net = cir.net(i)
                    if net.is_critical():
                        route_dr_ps.construct_net_routables_normal(net, False)
                        nets.append(net)
                        key = net.num_pins()
                        if key not in self.net_dict_pin:
                            self.net_dict_pin[key] = []
                            self.net_name_to_dict_pin_idx[key] = {}
                        self.net_name_to_dict_pin_idx[key][net.name()] = len(self.net_dict_pin[key])
                        self.net_dict_pin[key].append({
                            'cir_idx': len(self.cir_list),
                            'net': net,
                        })

                route_dr_ps.reset_history()

                self.cir_name_to_idx[cir.name()] = len(self.cir_list)
                self.cir_list.append({
                    'cir': cir,
                    'parser': parser,
                    'place_mgr': place_mgr, 
                    'route_mgr': route_mgr,
                    'route_dr_mgr': route_dr_mgr,
                    'route_dr_ps': route_dr_ps,
                    'drc_mgr': drc_mgr,
                    'nets': nets,
                    'route_drc_cost': arch.route_dr_ps_drc_cost,
                    'route_his_cost': arch.route_dr_ps_his_cost,
                })

        self.cir_idx = None
        self.cir = None
        self.parser = None
        self.place_mgr = None
        self.route_mgr = None
        self.route_dr_mgr = None
        self.route_dr_ps = None
        self.drc_mgr = None
        self.nets = None
        self.net = None
        self.route_drc_cost = None
        self.route_his_cost = None

        # observation, action, reward
        self.seg_feat_dim = self.cfg.rl_seg_feat_dim

        self.action_space = gym.spaces.Discrete(self.cfg.rl_max_segs + 1)

        self.max_seg_edges = 12 * self.cfg.rl_max_segs
        self.max_path_edges = self.cfg.rl_max_paths * self.cfg.rl_max_segs

        self.observation_space = gym.spaces.Dict({
            'action_mask':      gym.spaces.Box(0, 1, shape=(self.cfg.rl_max_segs + 1, ), dtype=np.float32),
            'seg_feats':        gym.spaces.Box(0, 10, shape=(self.cfg.rl_max_segs, self.seg_feat_dim), dtype=np.float32),
            'seg_edges':        gym.spaces.Box(0, self.cfg.rl_max_segs, shape=(self.max_seg_edges, 2), dtype=np.float32),
            'seg_valid_nodes':  gym.spaces.Box(0, 1, shape=(self.cfg.rl_max_segs, ), dtype=bool),
            'seg_valid_edges':  gym.spaces.Box(0, 1, shape=(self.max_seg_edges, ), dtype=bool),
            'path_feats':       gym.spaces.Box(-10, 10, shape=(self.cfg.rl_max_paths, self.cfg.rl_path_feat_dim), dtype=np.float32),
            'path_edges':       gym.spaces.Box(0, max(self.cfg.rl_max_segs, self.cfg.rl_max_paths), shape=(self.max_path_edges, 2), dtype=np.float32),
            'path_valid_nodes': gym.spaces.Box(0, 1, shape=(self.cfg.rl_max_paths, ), dtype=bool),
            'path_valid_edges': gym.spaces.Box(0, 1, shape=(self.max_path_edges, ), dtype=bool),
        })
        self.observation = None

        self.steps = 0

        self.reward = np.float32('-inf')

        self.drv_pre, self.drv_cur = None, None
        self.path_data_pre, self.path_data_cur = None, None

        self.target = None

        
        self.valid_nets = []
        if self.cfg.rl_net_max_pin is not None:
            for key, value in self.net_dict_pin.items():
                if key <= self.cfg.rl_net_max_pin:
                    self.valid_nets.extend(value)
        else:
            for value in self.net_dict_pin.values():
                self.valid_nets.extend(value)


    def step(self, action):

        done = False

        self.steps += 1
        if action == self.cfg.rl_max_segs or self.steps == self.cfg.rl_max_steps:
            done = True

        if not done:
            success = self.ripup_refine_reroute(self.net, action)
            # success = self.ripup_reroute_refine(self.net, action)
            assert success

            self.path_data_cur = self.ac_rl.get_path_res(self.cir, self.net)
            _, _, self.drv_cur = self.ac_rl.get_wl_via_drv(self.cir,
                                                           self.net,
                                                           self.route_dr_ps,
                                                           self.drc_mgr,
                                                           False)


        self.observation = self.get_observation()

        self.reward = self.get_reward()
        self.path_data_pre, self.drv_pre = self.path_data_cur, self.drv_cur
        info = {}

        return self.observation, self.reward, done, info

    def change_cir(self, cir_idx):
        if self.route_dr_ps is not None:
            if self.net is not None:
                self.route_dr_ps.ripup(self.net)
            self.route_dr_ps.reset_history()

        if cir_idx != self.cir_idx:
            self.cir_idx = cir_idx
            self.cir = self.cir_list[cir_idx]['cir']
            self.parser = self.cir_list[cir_idx]['parser']
            self.place_mgr= self.cir_list[cir_idx]['place_mgr']
            self.route_mgr = self.cir_list[cir_idx]['route_mgr']
            self.route_dr_mgr = self.cir_list[cir_idx]['route_dr_mgr']
            self.route_dr_ps = self.cir_list[cir_idx]['route_dr_ps']
            self.drc_mgr = self.cir_list[cir_idx]['drc_mgr']
            self.nets = self.cir_list[cir_idx]['nets']
            self.route_drc_cost = self.cir_list[cir_idx]['route_drc_cost']
            self.route_his_cost = self.cir_list[cir_idx]['route_his_cost']


    def reset(self,
              *,
              seed: Optional[int] = None,
              return_info: bool = False,
              options: Optional[dict] = None):

        if self.cir is not None:
            assert self.parser is not None
            assert self.place_mgr is not None
            assert self.route_mgr is not None
            assert self.route_dr_mgr is not None
            assert self.route_dr_ps is not None
            assert self.drc_mgr is not None
            assert self.nets is not None
            assert self.route_drc_cost is not None
            assert self.route_his_cost is not None
            # assert self.net is not None

        if options is not None:
            assert 'cir' in options
            assert 'net' in options
            assert isinstance(options['cir'], ac.cir_db)
            assert isinstance(options['net'], ac.net)
            self.change_cir(self.cir_name_to_idx[options['cir'].name()])
            self.net = options['net']

            if 'target' in options:
                assert isinstance(options['target'], dict)
                assert len(options['target']['path_res']) == self.net.num_conns()
                self.target = options['target']
            
            if 'reroute' in options and options['reroute']:
                self.route_dr_ps.ripup(self.net)
                self.dr_ps.run_nrr(ac.VectorInt([self.net.idx()]), False)
            assert self.net.is_routed, self.net.name()

        else:
            item = np.random.choice(self.valid_nets)
            self.change_cir(item['cir_idx'])
            self.net = item['net']

            if not self.cfg.rl_hier:
                self.target = np.random.choice(list(self.topo_dict[self.cir.name()][self.net.name()].values()))

            for i in range(np.random.randint(low=self.cfg.rl_min_reset_reroutes, high=self.cfg.rl_max_reset_reroutes)):
                self.route_dr_ps.ripup(self.net)
                success = self.route(self.net)
                assert success

        assert self.net.num_arcs() > 0, "{} {}".format(self.cir.name(), self.net.name())

        self.route_dr_ps.reset_history()
        self.ac_rl.add_cur_sol_to_history(self.net, self.route_dr_ps, self.route_his_cost)

        
        self.path_data_cur = self.ac_rl.get_path_res(self.cir, self.net)
        _, _, self.drv_cur = self.ac_rl.get_wl_via_drv(self.cir,
                                                       self.net,
                                                       self.route_dr_ps,
                                                       self.drc_mgr,
                                                       False)


        self.observation = self.get_observation()


        self.path_data_pre, self.drv_pre = self.path_data_cur, self.drv_cur

        self.steps = 0


        if not return_info:
            return self.observation
        else:
            return self.observation, {}

    def get_observation(self):
        seg_feats, seg_edges, seg_valid_nodes, seg_valid_edges, action_mask, \
        path_feats, path_edges, path_valid_nodes, path_valid_edges = \
                                            get_features_with_path_target(self.cfg,
                                                                          self.ac_rl,
                                                                          self.cir,
                                                                          self.net,
                                                                          self.route_dr_mgr,
                                                                          self.route_dr_ps,
                                                                          self.drc_mgr,
                                                                          self.path_data_cur,
                                                                          self.target)

        # assert np.can_cast(seg_feats.dtype, np.float32)
        # assert seg_feats.shape == self.observation_space['seg_feats'].shape
        # assert np.all(seg_feats >= self.observation_space['seg_feats'].low), '{}'.format(np.min(seg_feats))
        # assert np.all(seg_feats <= self.observation_space['seg_feats'].high), '{}'.format(np.max(seg_feats))

        # assert np.can_cast(path_feats.dtype, np.float32)
        # assert path_feats.shape == self.observation_space['path_feats'].shape
        # assert np.all(path_feats >= self.observation_space['path_feats'].low), '{}'.format(np.min(path_feats))
        # assert np.all(path_feats <= self.observation_space['path_feats'].high), '{}'.format(np.max(path_feats))

        # assert np.can_cast(action_mask.dtype, np.float32)
        # assert action_mask.shape == self.observation_space['action_mask'].shape
        # assert np.all(action_mask >= self.observation_space['action_mask'].low), '{}'.format(np.min(action_mask))
        # assert np.all(action_mask <= self.observation_space['action_mask'].high), '{}'.format(np.max(action_mask))

        observation = {
            'action_mask':      action_mask,
            'seg_feats':        seg_feats,
            'seg_edges':        seg_edges,
            'seg_valid_nodes':  seg_valid_nodes,
            'seg_valid_edges':  seg_valid_edges,
            'path_feats':       path_feats,
            'path_edges':       path_edges,
            'path_valid_nodes': path_valid_nodes,
            'path_valid_edges': path_valid_edges,
        }
        return observation 

    def get_reward(self):
        reward = self.cfg.rl_reward_constant + \
                 self.cfg.rl_reward_weight_drv * (self.drv_pre - self.drv_cur)
       
        path_res_t = self.target['path_res']
        for i, res_c in enumerate(self.path_data_cur):
            res_p = self.path_data_pre[i]
            res_t = path_res_t[i]
            reward += self.cfg.rl_reward_weight_res * (abs(res_p - res_t) - abs(res_c - res_t)) / self.cfg.rl_res_norm_constant

        return reward

    def ripup_refine_reroute(self, net, action):

        segs = ac.VectorSegment3dInt()
        net.v_arcs(segs)

        assert 0 <= action and action < len(segs)

        remove_seg = segs[action]
        self.route_dr_ps.ripup_segment_refine(net, remove_seg)

        success = self.route(net)
        return success

    def ripup_reroute_refine(self, net, action):

        segs = ac.VectorSegment3dInt()
        net.v_arcs(segs)
        assert 0 <= action and action < len(segs)

        remove_seg = segs[action]
        self.route_dr_ps.ripup_segment(net, remove_seg)

        success = self.route(net)

        self.route_dr_ps.refine_net(net)

        return success
    
    def route(self, net):
        # assert self.net.is_routed() == False
        if not net.is_routed():
            return self.route_dr_ps.route(net, False, self.route_drc_cost, self.route_his_cost)
        return True



    def render(self, mode='human'):
        pass
    def close (self):
        pass

class AcRouteMultiCirPathMatchFakeEnv(gym.Env):
    ''' autocraft fake env for inference '''

    def __init__(self, config: EnvContext):
        super().__init__()

        self.cfg = config['cfg']
        
        self.seg_feat_dim = self.cfg.rl_seg_feat_dim

        self.action_space = gym.spaces.Discrete(self.cfg.rl_max_segs + 1)
        
        self.max_seg_edges = 12 * self.cfg.rl_max_segs
        self.max_path_edges = self.cfg.rl_max_paths * self.cfg.rl_max_segs

        self.observation_space = gym.spaces.Dict({
            'action_mask':      gym.spaces.Box(0, 1, shape=(self.cfg.rl_max_segs + 1, ), dtype=np.float32),
            'seg_feats':        gym.spaces.Box(0, 10, shape=(self.cfg.rl_max_segs, self.seg_feat_dim), dtype=np.float32),
            'seg_edges':        gym.spaces.Box(0, self.cfg.rl_max_segs, shape=(self.max_seg_edges, 2), dtype=np.float32),
            'seg_valid_nodes':  gym.spaces.Box(0, 1, shape=(self.cfg.rl_max_segs, ), dtype=bool),
            'seg_valid_edges':  gym.spaces.Box(0, 1, shape=(self.max_seg_edges, ), dtype=bool),
            'path_feats':       gym.spaces.Box(-10, 10, shape=(self.cfg.rl_max_paths, self.cfg.rl_path_feat_dim), dtype=np.float32),
            'path_edges':       gym.spaces.Box(0, max(self.cfg.rl_max_segs, self.cfg.rl_max_paths), shape=(self.max_path_edges, 2), dtype=np.float32),
            'path_valid_nodes': gym.spaces.Box(0, 1, shape=(self.cfg.rl_max_paths, ), dtype=bool),
            'path_valid_edges': gym.spaces.Box(0, 1, shape=(self.max_path_edges, ), dtype=bool),
        })

    def reset(self,
              *,
              seed: Optional[int] = None,
              return_info: bool = False,
              options: Optional[dict] = None):

        if not return_info:
            return self.observation_space.sample()
        else:
            return self.observation_space.sample(), {}

    def step(self, action):
        obs = self.observation_space.sample()
        done = True
        reward = 0
        info = {}
        return obs, reward, done, info
       


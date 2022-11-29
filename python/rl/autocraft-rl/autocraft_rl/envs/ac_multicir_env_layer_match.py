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

class AcRouteMultiCirLayerMatchEnv(gym.Env):
    ''' autocraft rl env '''

    def __init__(self, config: EnvContext):
    # def __init__(self, cfg):

        super().__init__()
        self._skip_env_checking = True

        self.cfg = config['cfg']

        self.ac_rl = ac.rl_utils()

        self.cir_list = []
        self.cir_name_to_idx = {}
        self.net_dict_pin = {}
        self.net_name_to_dict_pin_idx = {}
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
                    min_wl, min_via = self.ac_rl.get_min_wl_via(net, route_dr_ps, False)
                    self.net_name_to_dict_pin_idx[key][net.name()] = len(self.net_dict_pin[key])
                    self.net_dict_pin[key].append({
                        'cir_idx': len(self.cir_list),
                        'net': net,
                        'min_wl': min_wl,
                        'min_via': min_via,
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
        self.max_segs = self.cfg.rl_max_segs
        self.max_edges = 12 * self.max_segs
        self.seg_feat_dim = self.cfg.rl_seg_feat_dim

        self.action_space = gym.spaces.Discrete(self.max_segs + 1)
        self.action_mask = None 

        self.observation_space = gym.spaces.Dict({
            'action_mask': gym.spaces.Box(0, 1, shape=(self.max_segs + 1, ), dtype=np.float32),
            'feats':       gym.spaces.Box(-100, 100, shape=(self.max_segs, self.seg_feat_dim), dtype=np.float32),
            'edges':       gym.spaces.Box(0, self.max_segs, shape=(self.max_edges, 2), dtype=np.float32),
            'valid_nodes': gym.spaces.Box(0, 1, shape=(self.max_segs, ), dtype=bool),
            'valid_edges': gym.spaces.Box(0, 1, shape=(self.max_edges, ), dtype=bool),
        })
        self.observation = None

        self.max_steps = self.cfg.rl_max_steps
        self.steps = 0

        self.reward = np.float32('-inf')

        self.wl_pre, self.via_pre, self.drv_pre = None, None, None
        self.wl_cur, self.via_cur, self.drv_cur = None, None, None

        self.target = None

        self.valid_nets = []
        if self.cfg.rl_net_max_pin is not None:
            for key, value in self.net_dict_pin.items():
                if key <= self.cfg.rl_net_max_pin:
                    self.valid_nets.extend(value)
        else:
            for value in self.net_dict_pin.values():
                self.valid_nets.extend(value)

        self.topo_dict = None
        with open(self.cfg.topo_dict, 'r') as f:
            # gc.disable()
            # self.topo_dict = ujson.load(f)
            self.topo_dict = orjson.load(f.read())
            # gc.enable()

    def step(self, action):

        assert self.action_space.contains(action)

        done = False

        self.steps += 1
        if action == self.max_segs or self.steps == self.max_steps:
            done = True

        if not done:
            # success = self.ripup_refine_reroute(self.net, action)
            success = self.ripup_reroute_refine(self.net, action)
            assert success

            self.wl_cur = ac.VectorFloat()
            self.via_cur = ac.VectorFloat()
            _, _, self.drv_cur = self.ac_rl.get_layers_wl_via_drv(self.cir,
                                                            self.net,
                                                            self.route_dr_ps,
                                                            self.drc_mgr,
                                                            self.wl_cur,
                                                            self.via_cur,
                                                            False)

        feats, edges, valid_nodes, valid_edges, self.action_mask = get_features_with_layer_target(self.cfg,
                                                                                              self.ac_rl,
                                                                                              self.cir,
                                                                                              self.net,
                                                                                              self.route_dr_mgr,
                                                                                              self.route_dr_ps,
                                                                                              self.drc_mgr,
                                                                                              self.wl_cur,
                                                                                              self.via_cur,
                                                                                              self.target)
         


        self.observation = self.get_observation(feats, edges, valid_nodes, valid_edges)

        # self.reward = self.get_reward(nodes, done)
        self.reward = self.get_reward()
        # print('Steps:', self.steps, 'Action:', action, 'Reward:', self.reward, 'Done:', done)

        self.wl_pre, self.via_pre, self.drv_pre = self.wl_cur, self.via_cur, self.drv_cur
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
            if 'cir' in options:
                self.change_cir(self.cir_name_to_idx[options['cir']])
                if 'net' in options:
                    self.net = self.cir.net(options['net'])
                else:
                    self.net = np.random.choice(self.nets)
                num_pins = self.net.num_pins()
                item = self.net_dict_pin[num_pins][self.net_name_to_dict_pin_idx[num_pins][self.net.name()]]
            else:
                item = np.random.choice(self.valid_nets)
                self.change_cir(item['cir_idx'])
                self.net = item['net']
        else:
            item = np.random.choice(self.valid_nets)
            self.change_cir(item['cir_idx'])
            self.net = item['net']


        for i in range(np.random.randint(low=self.cfg.rl_min_reset_reroutes, high=self.cfg.rl_max_reset_reroutes)):
            self.route_dr_ps.ripup(self.net)
            success = self.route()
            assert success

        assert self.net.num_arcs() > 0, "{} {}".format(self.cir.name(), self.net.name())

        self.route_dr_ps.reset_history()
        self.ac_rl.add_cur_sol_to_history(self.net, self.route_dr_ps, self.route_his_cost)

        
        self.wl_cur = ac.VectorFloat()
        self.via_cur = ac.VectorFloat()
        _, _, self.drv_cur = self.ac_rl.get_layers_wl_via_drv(self.cir,
                                                        self.net,
                                                        self.route_dr_ps,
                                                        self.drc_mgr,
                                                        self.wl_cur,
                                                        self.via_cur,
                                                        False)

        self.target = np.random.choice(list(self.topo_dict[self.cir.name()][self.net.name()].values()))
        
        feats, edges, valid_nodes, valid_edges, self.action_mask = get_features_with_layer_target(self.cfg,
                                                                                              self.ac_rl,
                                                                                              self.cir,
                                                                                              self.net,
                                                                                              self.route_dr_mgr,
                                                                                              self.route_dr_ps,
                                                                                              self.drc_mgr,
                                                                                              self.wl_cur,
                                                                                              self.via_cur,
                                                                                              self.target)
        # if not self.observation_space['feats'].contains(feats):
            # print('cast', np.can_cast(feats.dtype, np.float32))
            # print('min', np.min(feats, axis=0))
            # print('max', np.max(feats, axis=0))
            # exit()


        self.wl_pre, self.via_pre, self.drv_pre = self.wl_cur, self.via_cur, self.drv_cur



        self.observation = self.get_observation(feats, edges, valid_nodes, valid_edges)
        self.steps = 0

        if not return_info:
            return self.observation
        else:
            return self.observation, {}

    def get_observation(self, feats, edges, valid_nodes, valid_edges):
        assert np.can_cast(feats.dtype, np.float32)
        assert feats.shape == self.observation_space['feats'].shape
        assert np.all(feats >= self.observation_space['feats'].low)
        assert np.all(feats <= self.observation_space['feats'].high)

        observation = {
            'action_mask': self.action_mask,
            'feats': feats,
            'edges': edges,
            'valid_nodes': valid_nodes,
            'valid_edges': valid_edges,
        }
        return observation 

    def get_reward(self):
        reward = self.cfg.rl_reward_constant
        for i, wl_c in enumerate(self.wl_cur):
            wl_p = self.wl_pre[i]
            wl_t = self.target['wl_m{:d}'.format(i + 1)]
            reward += self.cfg.rl_reward_weight_wl * (abs(wl_p - wl_t) - abs(wl_c - wl_t)) / self.net.bbox().hpwl()
        
        for i, via_c in enumerate(self.via_cur):
            via_p = self.via_pre[i]
            via_t = self.target['via_v{:d}'.format(i + 1)]
            reward += self.cfg.rl_reward_weight_via * (abs(via_p - via_t) - abs(via_c - via_t)) / self.cfg.rl_via_norm_constant

        reward += self.cfg.rl_reward_weight_drv * (self.drv_pre - self.drv_cur)
        return reward

    def ripup_refine_reroute(self, net, action):

        segs = ac.VectorSegment3dInt()
        self.net.v_arcs(segs)
        # print('action', action)
        # print('segs', len(segs))
        assert 0 <= action and action < len(segs)

        remove_seg = segs[action]
        # self.route_dr_ps.ripup_segment(self.net, remove_seg)
        # self.route_dr_ps.refine_net(self.net)
        self.route_dr_ps.ripup_segment_refine(self.net, remove_seg)

        success = self.route()
        return success

    def ripup_reroute_refine(self, net, action):

        segs = ac.VectorSegment3dInt()
        self.net.v_arcs(segs)
        assert 0 <= action and action < len(segs)

        remove_seg = segs[action]
        self.route_dr_ps.ripup_segment(self.net, remove_seg)

        success = self.route()

        self.route_dr_ps.refine_net(self.net)

        return success
    
    def route(self):
        # assert self.net.is_routed() == False
        if not self.net.is_routed():
            return self.route_dr_ps.route(self.net, False, self.route_drc_cost, self.route_his_cost)
        return True



    def render(self, mode='human'):
        pass
    def close (self):
        pass

class AcRouteMultiCirLayerMatchFakeEnv(gym.Env):
    ''' autocraft fake env for inference '''

    def __init__(self, config: EnvContext):
        super().__init__()

        self.cfg = config['cfg']
        
        self.max_segs = self.cfg.rl_max_segs
        self.max_edges = 12 * self.max_segs
        self.seg_feat_dim = self.cfg.rl_seg_feat_dim

        self.action_space = gym.spaces.Discrete(self.max_segs + 1)

        self.observation_space = gym.spaces.Dict({
            'action_mask': gym.spaces.Box(0, 1, shape=(self.max_segs + 1, ), dtype=np.float32),
            'feats':       gym.spaces.Box(-100, 100, shape=(self.max_segs, self.seg_feat_dim), dtype=np.float32),
            'edges':       gym.spaces.Box(0, self.max_segs, shape=(self.max_edges, 2), dtype=np.float32),
            'valid_nodes': gym.spaces.Box(0, 1, shape=(self.max_segs, ), dtype=bool),
            'valid_edges': gym.spaces.Box(0, 1, shape=(self.max_edges, ), dtype=bool),
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
        reward = 0
        done = True if (action == self.cfg.rl_max_segs or self.steps == self.cfg.rl_max_steps) else False
        info = {}
        return obs, reward, done, info

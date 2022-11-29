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
from ray.rllib.env.env_context import EnvContext

import autocraft as ac
from autocraft_rl.utils.helper import *


# import ujson
import orjson
import gc
import copy

class AcRouteMultiCirPathMatchGroupEnv(gym.Env):
    ''' autocraft rl env '''
    '''
        considering an entire match group:
    '''

    def __init__(self, config: EnvContext,
                 options: Optional[dict] = None):
        super().__init__()

        self._skip_env_checking = True

        self.cfg   = config['cfg']
        self.ac_rl = ac.rl_utils()
        
        self.cir_list = []
        self.cir_name_to_idx = {}
        self.net_dict_pin = {}
        self.net_name_to_dict_pin_idx = {}
        if options is not None:
            cir = options['cir']
            dr_mgr = options['dr_mgr']
            dr_ps = options['dr_ps']
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
                'dr_mgr': dr_mgr,
                'dr_ps': dr_ps,
                'drc_mgr': drc_mgr,
                'nets': nets,
                'drc_cost': drc_cost,
                'his_cost': his_cost,
            })
        else:
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
                dr_mgr = ac.route_dr_mgr(cir)

                power_net_ids = []
                for net in cir.v_nets():
                    if net.is_power():
                        power_net_ids.append(net.idx())

                if 'in_route_gds' in arch and arch.in_route_gds is not None:
                    dr_mgr.init(False, 6)
                    parser.parse_route_gds(arch.in_route_gds, ac.VectorInt(power_net_ids))
                else:
                    route_mgr.gen_pg()
                    dr_mgr.init(False, 6)
                dr_ps = ac.route_dr_ps(dr_mgr, 1)
                
                drc_mgr = ac.drc_mgr(cir)

                nets = []
                for i in range(cir.num_nets()):
                    net = cir.net(i)
                    if net.is_critical():
                        dr_ps.construct_net_routables_normal(net, False)
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

                dr_ps.reset_history()

                self.cir_name_to_idx[cir.name()] = len(self.cir_list)
                self.cir_list.append({
                    'cir': cir,
                    'parser': parser,
                    'place_mgr': place_mgr, 
                    'route_mgr': route_mgr,
                    'dr_mgr': dr_mgr,
                    'dr_ps': dr_ps,
                    'drc_mgr': drc_mgr,
                    'nets': nets,
                    'drc_cost': arch.route_dr_ps_drc_cost,
                    'his_cost': arch.route_dr_ps_his_cost,
                })

        self.cir_idx = None
        self.cir = None
        self.parser = None
        self.place_mgr = None
        self.route_mgr = None
        self.dr_mgr = None
        self.dr_ps = None
        self.drc_mgr = None
        self.nets = None
        self.net = None
        self.drc_cost = None
        self.his_cost = None

        # observation, action, reward
        self.max_segs                 = self.cfg.rl_max_nets_per_group * self.cfg.rl_max_segs
        self.max_paths                = self.cfg.rl_max_nets_per_group * self.cfg.rl_max_paths
        self.max_supers               = self.cfg.rl_max_path_cstrs
        self.max_seg_to_seg_edges     = self.cfg.rl_max_nets_per_group * 12 * self.cfg.rl_max_segs
        self.max_seg_to_path_edges    = self.cfg.rl_max_nets_per_group * self.cfg.rl_max_segs * self.cfg.rl_max_paths
        self.max_path_to_super_edges  = self.cfg.rl_max_nets_per_group * self.cfg.rl_max_paths
        self.max_super_to_seg_edges   = self.cfg.rl_max_nets_per_group * self.cfg.rl_max_path_cstrs * self.cfg.rl_max_segs

        self.observation_space = Dict({
            'seg_feats':                  Box(0, 10,                                   shape=(self.max_segs, self.cfg.rl_seg_feat_dim),       dtype=np.float32),
            'path_feats':                 Box(-10, 10,                                 shape=(self.max_paths, self.cfg.rl_path_feat_dim - 1), dtype=np.float32),
            # 'super_feats':                Box(0, 10,                                   shape=(self.max_supers, 0),                            dtype=np.float32),
            'super_feats':                Box(0, 10,                                   shape=(self.max_supers, 3),                            dtype=np.float32),
            'seg_to_seg_edges':           Box(0, self.max_segs,                        shape=(self.max_seg_to_seg_edges, 2),                  dtype=np.float32),
            'seg_to_path_edges':          Box(0, max(self.max_segs, self.max_paths),   shape=(self.max_seg_to_path_edges, 2),                 dtype=np.float32),
            'path_to_super_edges':        Box(0, max(self.max_paths, self.max_supers), shape=(self.max_path_to_super_edges, 2),               dtype=np.float32),
            'super_to_seg_edges':         Box(0, self.max_super_to_seg_edges,          shape=(self.max_super_to_seg_edges, 2),                dtype=np.float32),
            'seg_valid_nodes':            Box(0, 1,                                    shape=(self.max_segs,),                                dtype=bool),
            'path_valid_nodes':           Box(0, 1,                                    shape=(self.max_paths,),                               dtype=bool),
            'super_valid_nodes':          Box(0, 1,                                    shape=(self.max_supers,),                              dtype=bool),
            'seg_to_seg_valid_edges':     Box(0, 1,                                    shape=(self.max_seg_to_seg_edges,),                    dtype=bool),
            'seg_to_path_valid_edges':    Box(0, 1,                                    shape=(self.max_seg_to_path_edges,),                   dtype=bool),
            'path_to_super_valid_edges':  Box(0, 1,                                    shape=(self.max_path_to_super_edges,),                 dtype=bool),
            'super_to_seg_valid_edges':   Box(0, 1,                                    shape=(self.max_super_to_seg_edges,),                  dtype=bool),
            'action_mask':                Box(0, 1,                                    shape=(self.max_segs + 1,),                            dtype=bool),
        })
        self.action_space = Discrete(self.max_segs + 1)
        self.obs = None

        self.steps = 0

        self.path_match_cstrs = None
        self.path_match_cstr = None # current path match cstr
        self.match_group = None # current match group
        self.match_group_net_ids = None

        self.paths_data_pre, self.paths_data_cur = None, None
        self.net_wl_pre, self.net_wl_cur = None, None
        self.net_via_pre, self.net_via_cur = None, None
        self.net_drv_pre, self.net_drv_cur = None, None

    def step(self, action):

        done = False
        rew = 0

        self.steps += 1
        if action == self.action_space.n - 1 or self.steps == self.cfg.rl_max_steps:
            done = True

        if not done:
            net_idx, seg_idx = divmod(action, self.cfg.rl_max_segs)
            net = self.match_group[net_idx]
            
            success = self.ripup_refine_reroute(net, seg_idx)
            assert success

            self.obs = self.get_observation()
            
            self.update_data_cur()

            rew = self.get_reward()

            self.paths_data_pre = self.paths_data_cur
            self.net_wl_pre = self.net_wl_cur
            self.net_via_pre = self.net_via_cur
            self.net_drv_pre = self.net_drv_cur

        return self.obs, rew, done, {}

    def get_reward(self):
        net_wl_pre,  net_wl_cur  = np.array(self.net_wl_pre),  np.array(self.net_wl_cur)
        net_via_pre, net_via_cur = np.array(self.net_via_pre), np.array(self.net_via_cur)
        net_drv_pre, net_drv_cur = np.array(self.net_drv_pre), np.array(self.net_drv_cur)

        ptp_pre = np.array([np.ptp(x) for x in self.paths_data_pre])
        ptp_cur = np.array([np.ptp(x) for x in self.paths_data_cur])
    
        group_wl_pre, group_wl_cur = net_wl_pre.sum(), net_wl_cur.sum()

        # rew_wl_ptp = np.ptp(net_wl_pre) - np.ptp(net_wl_cur)
        # rew_via_ptp = np.ptp(net_via_pre) - np.ptp(net_via_cur)
        rew_res_ptp = (ptp_pre - ptp_cur).mean()
        
        rew_wl = (group_wl_pre - group_wl_cur)
        rew_via = (net_via_pre - net_via_cur).mean()
        rew_drv = (net_drv_pre - net_drv_cur).mean()

        return self.cfg.rl_reward_constant + \
               self.cfg.rl_reward_weight_wl * rew_wl / self.cfg.rl_wl_norm_constant + \
               self.cfg.rl_reward_weight_via * rew_via / self.cfg.rl_via_norm_constant + \
               self.cfg.rl_reward_weight_drv * rew_drv + \
               self.cfg.rl_reward_weight_res * rew_res_ptp / self.cfg.rl_res_norm_constant 
               # self.cfg.rl_reward_weight_wl_ptp * rew_wl_ptp / self.cfg.rl_wl_norm_constant + \
               # self.cfg.rl_reward_weight_via_ptp * rew_via_ptp / self.cfg.rl_via_norm_constant


    def change_cir(self, cir_idx):
        if self.dr_ps is not None:
            if self.match_group is not None:
                for net in self.match_group:
                    self.dr_ps.ripup(net)
            self.dr_ps.reset_history()

        if cir_idx != self.cir_idx:
            self.cir_idx   = cir_idx
            self.cir       = self.cir_list[cir_idx]['cir']
            self.parser    = self.cir_list[cir_idx]['parser']
            self.place_mgr = self.cir_list[cir_idx]['place_mgr']
            self.route_mgr = self.cir_list[cir_idx]['route_mgr']
            self.dr_mgr    = self.cir_list[cir_idx]['dr_mgr']
            self.dr_ps     = self.cir_list[cir_idx]['dr_ps']
            self.drc_mgr   = self.cir_list[cir_idx]['drc_mgr']
            self.nets      = self.cir_list[cir_idx]['nets']
            self.drc_cost  = self.cir_list[cir_idx]['drc_cost']
            self.his_cost  = self.cir_list[cir_idx]['his_cost']
            self.path_match_cstrs = self.cir.v_route_path_match_cstrs()


    def reset(self,
              *,
              seed: Optional[int] = None,
              return_info: bool = False,
              options: Optional[dict] = None):

        # init match_group
        if options is not None:
            assert 'cir' in options
            assert isinstance(options['cir'], ac.cir_db)
            assert isinstance(options['path_match_cstr'], ac.route_path_match_cstr)
            self.change_cir(self.cir_name_to_idx[options['cir'].name()])
            self.path_match_cstr = options['path_match_cstr']
            self.match_group = self.path_match_cstr.v_nets()
            self.match_group_net_ids = [net.idx() for net in self.match_group]

            if 'reroute' in options and options['reroute']:
                for net in self.match_group:
                    self.dr_ps.ripup(net)
                self.dr_ps.run_nrr(ac.VectorInt(self.match_group_net_ids), False)
            else:
                for net in self.match_group:
                    assert net.is_routed(), net.name()

        else:
            self.change_cir(np.random.randint(0, len(self.cir_list)))
            self.path_match_cstr = np.random.choice(self.path_match_cstrs)
            self.match_group = self.path_match_cstr.v_nets()
            self.match_group_net_ids = [net.idx() for net in self.match_group]

            for net in self.match_group:
                self.dr_ps.ripup(net)

            if self.cir.name() in ['dv5t_crg_fll_ana_obsd', 'dv5t_crg_fll_ana_vco4_core']:
                success = self.dr_ps.run_nrr(ac.VectorInt(self.match_group_net_ids), False)
                if not success:
                    self.dr_ps.reset_history()
                    for net in self.match_group:
                        self.dr_ps.ripup(net)
                    a = self.match_group_net_ids.copy()
                    np.random.shuffle(a)
                    for net_idx in a:
                        net = self.cir.net(net_idx)
                        if not net.is_routed():
                            self.dr_ps.route(net, False, self.drc_cost, self.his_cost)

            elif self.cir.name() in ['dv5t_crg_fll_ana_vco4', 'isr_clk_pi_pruned', 'isr_clk_pi']:
                self.dr_ps.reset_history()
                a = self.match_group_net_ids.copy()
                np.random.shuffle(a)
                for net_idx in a:
                    net = self.cir.net(net_idx)
                    if not net.is_routed():
                        self.dr_ps.route(net, False, self.drc_cost, self.his_cost)

            for i in range(np.random.randint(low=self.cfg.rl_min_reset_reroutes, high=self.cfg.rl_max_reset_reroutes)):
                net = np.random.choice(self.match_group)
                self.dr_ps.ripup(net)
                self.dr_ps.route(net, False, self.drc_cost, self.his_cost)

        # init history
        self.dr_ps.reset_history()
        for net in self.match_group:
            assert net.is_routed()
            assert net.num_arcs() > 0, "{} {}".format(self.cir.name(), net.name())
            self.ac_rl.add_cur_sol_to_history(net, self.dr_ps, self.his_cost)

       
        self.update_data_cur()
        self.paths_data_pre = self.paths_data_cur
        self.net_wl_pre = self.net_wl_cur
        self.net_via_pre = self.net_via_cur
        self.net_drv_pre = self.net_drv_cur

        # print(self.paths_data_cur)
        # print(self.net_wl_cur)
        # print(self.net_via_cur)
        # print(self.net_drv_cur)

        self.obs = self.get_observation()

        self.steps = 0
        return self.obs

    def update_data_cur(self):
        self.net_wl_cur = []
        self.net_via_cur = []
        self.net_drv_cur = []
        self.paths_data_cur = []
        for net in self.match_group:
            wl, via, drv = self.ac_rl.get_wl_via_drv(self.cir, net, self.dr_ps, self.drc_mgr, False)
            self.net_wl_cur.append(wl)
            self.net_via_cur.append(via)
            self.net_drv_cur.append(drv)
        for path_cstr in self.path_match_cstr.v_path_cstrs():
            path_res = [self.cir.path_res(*conn) for conn in path_cstr.v_conns()]
            self.paths_data_cur.append(path_res)


    def get_observation(self):
        seg_feats, path_feats, super_feats, \
        seg_to_seg_edges, seg_to_path_edges, path_to_super_edges, super_to_seg_edges, \
        seg_valid_nodes, path_valid_nodes, super_valid_nodes, \
        seg_to_seg_valid_edges, seg_to_path_valid_edges, path_to_super_valid_edges, super_to_seg_valid_edges, \
        action_mask = \
            get_match_group_features_flat(self.cfg,
                                          self.ac_rl,
                                          self.cir,
                                          self.match_group,
                                          self.paths_data_cur,
                                          self.dr_mgr,
                                          self.dr_ps,
                                          self.drc_mgr)

        assert np.can_cast(seg_feats.dtype, np.float32)
        assert seg_feats.shape == self.observation_space['seg_feats'].shape, f"{seg_feats.shape} {self.observation_space['seg_feats'].shape}"
        assert np.all(seg_feats >= self.observation_space['seg_feats'].low), f'{np.min(seg_feats)}'
        assert np.all(seg_feats <= self.observation_space['seg_feats'].high), f'{np.max(seg_feats)}'

        assert np.can_cast(path_feats.dtype, np.float32)
        assert path_feats.shape == self.observation_space['path_feats'].shape, f"{path_feats.shape} {self.observation_space['path_feats'].shape}"
        assert np.all(path_feats >= self.observation_space['path_feats'].low), f'{np.min(path_feats)}'
        assert np.all(path_feats <= self.observation_space['path_feats'].high), f'{np.max(path_feats)}'
        
        assert np.can_cast(super_feats.dtype, np.float32)
        assert super_feats.shape == self.observation_space['super_feats'].shape, f"{super_feats.shape} {self.observation_space['super_feats'].shape}"
        assert np.all(super_feats >= self.observation_space['super_feats'].low), f'{np.min(super_feats)}'
        assert np.all(super_feats <= self.observation_space['super_feats'].high), f'{np.max(super_feats)}'
        
        assert np.can_cast(seg_to_seg_edges.dtype, np.float32)
        assert seg_to_seg_edges.shape == self.observation_space['seg_to_seg_edges'].shape
        assert np.all(seg_to_seg_edges >= self.observation_space['seg_to_seg_edges'].low), f'{np.min(seg_to_seg_edges)}'
        assert np.all(seg_to_seg_edges <= self.observation_space['seg_to_seg_edges'].high), f'{np.max(seg_to_seg_edges)}'
        
        assert np.can_cast(seg_to_path_edges.dtype, np.float32)
        assert seg_to_path_edges.shape == self.observation_space['seg_to_path_edges'].shape
        assert np.all(seg_to_path_edges >= self.observation_space['seg_to_path_edges'].low), f'{np.min(seg_to_path_edges)}'
        assert np.all(seg_to_path_edges <= self.observation_space['seg_to_path_edges'].high), f'{np.max(seg_to_path_edges)}'
        
        assert np.can_cast(path_to_super_edges.dtype, np.float32)
        assert path_to_super_edges.shape == self.observation_space['path_to_super_edges'].shape
        assert np.all(path_to_super_edges >= self.observation_space['path_to_super_edges'].low), f'{np.min(path_to_super_edges)}'
        assert np.all(path_to_super_edges <= self.observation_space['path_to_super_edges'].high), f'{np.max(path_to_super_edges)}'
        
        assert np.can_cast(super_to_seg_edges.dtype, np.float32)
        assert super_to_seg_edges.shape == self.observation_space['super_to_seg_edges'].shape
        assert np.all(super_to_seg_edges >= self.observation_space['super_to_seg_edges'].low), f'{np.min(super_to_seg_edges)}'
        assert np.all(super_to_seg_edges <= self.observation_space['super_to_seg_edges'].high), f'{np.max(super_to_seg_edges)}'

        obs = {
            'seg_feats': seg_feats,               
            'path_feats': path_feats,
            'super_feats': super_feats,
            'seg_to_seg_edges': seg_to_seg_edges,    
            'seg_to_path_edges': seg_to_path_edges, 
            'path_to_super_edges': path_to_super_edges, 
            'super_to_seg_edges': super_to_seg_edges, 
            'seg_valid_nodes': seg_valid_nodes, 
            'path_valid_nodes': path_valid_nodes,
            'super_valid_nodes': super_valid_nodes,
            'seg_to_seg_valid_edges': seg_to_seg_valid_edges,
            'seg_to_path_valid_edges': seg_to_path_valid_edges, 
            'path_to_super_valid_edges': path_to_super_valid_edges,
            'super_to_seg_valid_edges': super_to_seg_valid_edges,
            'action_mask': action_mask,
        }
        return obs
    
    def ripup_refine_reroute(self, net, action):

        segs = ac.VectorSegment3dInt()
        net.v_arcs(segs)

        assert 0 <= action and action < len(segs)

        remove_seg = segs[action]
        self.dr_ps.ripup_segment_refine(net, remove_seg)

        if not net.is_routed():
            return self.dr_ps.route(net, False, self.drc_cost, self.his_cost)
        return True


class AcRouteMultiCirPathMatchGroupFakeEnv(gym.Env):
    ''' autocraft fake env for inference '''
    def __init__(self, config: EnvContext):
        super().__init__()

        self._skip_env_checking = True

        self.cfg = config['cfg']
        
        # observation, action, reward
        self.max_segs                 = self.cfg.rl_max_nets_per_group * self.cfg.rl_max_segs
        self.max_paths                = self.cfg.rl_max_nets_per_group * self.cfg.rl_max_paths
        self.max_supers               = self.cfg.rl_max_path_cstrs
        self.max_seg_to_seg_edges     = self.cfg.rl_max_nets_per_group * 12 * self.cfg.rl_max_segs
        self.max_seg_to_path_edges    = self.cfg.rl_max_nets_per_group * self.cfg.rl_max_segs * self.cfg.rl_max_paths
        self.max_path_to_super_edges  = self.cfg.rl_max_nets_per_group * self.cfg.rl_max_paths
        self.max_super_to_seg_edges   = self.cfg.rl_max_nets_per_group * self.cfg.rl_max_path_cstrs * self.cfg.rl_max_segs

        self.observation_space = Dict({
            'seg_feats':                  Box(0, 10,                                   shape=(self.max_segs, self.cfg.rl_seg_feat_dim),       dtype=np.float32),
            'path_feats':                 Box(-10, 10,                                 shape=(self.max_paths, self.cfg.rl_path_feat_dim - 1), dtype=np.float32),
            # 'super_feats':                Box(0, 10,                                   shape=(self.max_supers, 0),                            dtype=np.float32),
            'super_feats':                Box(0, 10,                                   shape=(self.max_supers, 3),                            dtype=np.float32),
            'seg_to_seg_edges':           Box(0, self.max_segs,                        shape=(self.max_seg_to_seg_edges, 2),                  dtype=np.float32),
            'seg_to_path_edges':          Box(0, max(self.max_segs, self.max_paths),   shape=(self.max_seg_to_path_edges, 2),                 dtype=np.float32),
            'path_to_super_edges':        Box(0, max(self.max_paths, self.max_supers), shape=(self.max_path_to_super_edges, 2),               dtype=np.float32),
            'super_to_seg_edges':         Box(0, self.max_super_to_seg_edges,          shape=(self.max_super_to_seg_edges, 2),                dtype=np.float32),
            'seg_valid_nodes':            Box(0, 1,                                    shape=(self.max_segs,),                                dtype=bool),
            'path_valid_nodes':           Box(0, 1,                                    shape=(self.max_paths,),                               dtype=bool),
            'super_valid_nodes':          Box(0, 1,                                    shape=(self.max_supers,),                              dtype=bool),
            'seg_to_seg_valid_edges':     Box(0, 1,                                    shape=(self.max_seg_to_seg_edges,),                    dtype=bool),
            'seg_to_path_valid_edges':    Box(0, 1,                                    shape=(self.max_seg_to_path_edges,),                   dtype=bool),
            'path_to_super_valid_edges':  Box(0, 1,                                    shape=(self.max_path_to_super_edges,),                 dtype=bool),
            'super_to_seg_valid_edges':   Box(0, 1,                                    shape=(self.max_super_to_seg_edges,),                  dtype=bool),
            'action_mask':                Box(0, 1,                                    shape=(self.max_segs + 1,),                            dtype=bool),
        })
        self.action_space = Discrete(self.max_segs + 1)


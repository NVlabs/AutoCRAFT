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
from autocraft_rl.envs.ac_multicir_env_path_match import AcRouteMultiCirPathMatchEnv
from autocraft_rl.utils.helper import *

import ast
from pprint import pprint


def get_observation_path_match(cfg,
                               cir: ac.cir_db,
                               net: ac.net,
                               dr_mgr: ac.route_dr_mgr,
                               dr_ps: ac.route_dr_ps,
                               drc_mgr: ac.drc_mgr,
                               target):

    ac_rl = ac.rl_utils()

    path_data = ac_rl.get_path_res(cir, net)

    seg_feats, seg_edges, seg_valid_nodes, seg_valid_edges, seg_masks, \
    path_feats, path_edges, path_valid_nodes, path_valid_edges = \
        get_features_with_path_target(cfg,
                                      ac_rl,
                                      cir,
                                      net,
                                      dr_mgr,
                                      dr_ps,
                                      drc_mgr,
                                      path_data,
                                      target)
    
    observation = {
        'action_mask':      seg_masks,
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


def get_path_target(cir: ac.cir_db,
                    net: ac.net,
                    cur_path_res: ac.VectorFloat):


    target = {'path_res': []}

    # has_same_net_matching = False
    # path_cstr_ids = set()
    # for path_cstr, idx in net.v_conns():
        # if path_cstr.idx() in path_cstr_ids:
            # has_same_net_matching = True
            # break
        # path_cstr_ids.add(path_cstr.idx())
        
    for i, (path_cstr, idx) in enumerate(net.v_conns()):
        res_avg = 0.
        cnt = 0
        
        for j, (src, tar) in enumerate(path_cstr.v_conns()):
            # if src.net().idx() != net.idx() or has_same_net_matching:
            if True:
                res, has_path = cir.path_res(src, tar)
                if has_path:
                    res_avg += res
                    cnt += 1

        if cnt > 0:
            res_avg /= cnt
        else:
            res_avg = cur_path_res[i]
            
        target['path_res'].append(res_avg)
            

    return target


def execute_action(bench,
                   cir: ac.cir_db,
                   net: ac.net,
                   dr_ps: ac.route_dr_ps,
                   agent,
                   observation):

    env = agent.workers.local_worker().env

    action = agent.compute_action(observation)
    assert env.action_space.contains(action)

    done = True if action == env.action_space.n - 1 else False

    if not done:
        segs = ac.VectorSegment3dInt()
        net.v_arcs(segs)
        s = segs[action]
        dr_ps.ripup_segment_refine(net, s)

        success = True
        if not net.is_routed():
            success = dr_ps.route(net, False, bench.route_dr_ps_drc_cost, bench.route_dr_ps_his_cost)
        assert success

    else:
        dr_ps.reset_history()

    return done

# def rl_route_path_match_flat(bench,
                             # cir: ac.cir_db,
                             # net: ac.net,
                             # dr_mgr: ac.route_dr_mgr,
                             # dr_ps: ac.route_dr_ps,
                             # drc_mgr: ac.drc_mgr,
                             # agent,
                             # agent_config):

    # cfg = agent_config['env_config']['cfg']

    # dr_ps.reset_history()
    # # dr_ps.ripup(net)
    # # dr_ps.route(net, False, bench.route_dr_ps_drc_cost, bench.route_dr_ps_his_cost)
    # # dr_ps.reset_history()

    # ac_rl = ac.rl_utils()
    # ac_rl.add_cur_sol_to_history(net, dr_ps, bench.route_dr_ps_his_cost)

    # cur_path_res = ac_rl.get_path_res(cir, net)

    # print(net.name(), 'path_res_before: ', end='')
    # print_path_res(cur_path_res)
    # print()
    
    # target = get_path_target(cir, net, cur_path_res)

    # done = False
    # for step in range(agent_config['env_config']['cfg']['rl_max_steps']):
        # # print(target)
        # observation = get_observation_path_match(cfg, cir, net, dr_mgr, dr_ps, drc_mgr, target)
        # done = execute_action(bench, cir, net, dr_ps, agent, observation)
        # if done == True:
            # break

    # cur_path_res = ac_rl.get_path_res(cir, net)
    # print(net.name(), 'path_res_after:  ', end='')
    # print_path_res(cur_path_res)
    # print(f' (episode len: {step})')
    
    # print(net.name(), 'path_res_target: ', end='')
    # print_path_res(target['path_res'])
    # print()

    # print()
def rl_route_path_match_flat(bench,
                             cir: ac.cir_db,
                             net: ac.net,
                             dr_mgr: ac.route_dr_mgr,
                             dr_ps: ac.route_dr_ps,
                             drc_mgr: ac.drc_mgr,
                             agent, 
                             config):

    ac_rl = ac.rl_utils()
    cur_path_res = ac_rl.get_path_res(cir, net)

    print(net.name(), 'path_res_before: ', end='')
    print_path_res(cur_path_res)
    print()

    target = get_path_target(cir, net, cur_path_res)
    env = AcRouteMultiCirPathMatchEnv(config['env_config'],
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
        'net': net,
        'target': target,
        'reroute': False,
    })
    done = False
    rew_tot = 0

    while not done:
        action = agent.compute_action(obs)
        obs, rew, done, _ = env.step(action)
        rew_tot += rew
        # print(f'step: {env.steps}, rew: {rew:.4f}')
    # print(f'Total rew: {rew_tot:.4f}')
    
    cur_path_res = ac_rl.get_path_res(cir, net)
    print(net.name(), 'path_res_after:  ', end='')
    print_path_res(cur_path_res)
    print(f' (episode len: {env.steps})')
    
    print(net.name(), 'path_res_target: ', end='')
    print_path_res(target['path_res'])
    print()



def ptp_net(cir: ac.cir_db,
            cstr: ac.route_path_match_cstr):

    paths_data = []
    for path_cstr in cstr.v_path_cstrs():
        path_res = np.array([cir.path_res(*conn)[0] for conn in path_cstr.v_conns()])
        paths_data.append(path_res)

    ptp = [np.ptp(x) / np.mean(x) for x in paths_data]
    path_cstr_idx = np.argmax(ptp)
    path_cstr = cstr.path_cstr_const(path_cstr_idx)
    max_net = path_cstr.src_pin(np.argmax(paths_data[path_cstr_idx])).net()
    min_net = path_cstr.src_pin(np.argmin(paths_data[path_cstr_idx])).net()

    max_res  = paths_data[path_cstr_idx].max()
    min_res  = paths_data[path_cstr_idx].min()
    mean_res = paths_data[path_cstr_idx].mean()

    d_max_to_mean = abs(max_res - mean_res)
    d_min_to_mean = abs(min_res - mean_res)

    if d_max_to_mean >= d_min_to_mean:
        net = max_net
    else:
        net = min_net
    print(f'ptp cstr: {path_cstr.idx()}, net: {net.name()}')
    return net


def print_path_res(path_res):
    with np.printoptions(precision=2, suppress=True):
        print(np.array(path_res), end='')

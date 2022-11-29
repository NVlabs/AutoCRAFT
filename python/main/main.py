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

import sys
import os
import argparse
import hydra
import yaml 
import ujson
from omegaconf import DictConfig, OmegaConf, open_dict 
import time

import numpy as np
import torch as th
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
from autocraft_rl.models.ac_ppo_model_path_group import AcRoutePPOPathGroupModel
from autocraft_rl.models.ac_ppo_hier_model_path import AcRoutePPOPathTopLevelModel, AcRoutePPOPathMidLevelModel

from autocraft_rl.envs.ac_multicir_env import *
from autocraft_rl.envs.ac_multicir_env_match import *
from autocraft_rl.envs.ac_multicir_env_layer_match import *
from autocraft_rl.envs.ac_multicir_env_path_match import * 
from autocraft_rl.envs.ac_multicir_env_path_match_group import * 
from autocraft_rl.envs.ac_multicir_hier_env_path_match import * 

from autocraft_rl.utils.callbacks import AcRouteCallbacks
from autocraft_rl.utils.callbacks_match import AcRouteMatchCallbacks
from autocraft_rl.utils.callbacks_layer_match import AcRouteLayerMatchCallbacks
from autocraft_rl.utils.callbacks_path_match import AcRoutePathMatchCallbacks
from autocraft_rl.utils.callbacks_hier_path_match import AcRouteHierPathMatchCallbacks
from autocraft_rl.utils.helper import *


from rl_route_path_match_flat import rl_route_path_match_flat, ptp_net
# from rl_route_path_match_hier import *
from rl_route_path_match_flat_group import rl_route_path_match_group

import pprint

def load_rl_model(arch, config_path, checkpoint_path):
    with open(config_path, 'rb') as f:
        config = pickle.load(f)
    config['num_workers'] = 0
    config['evaluation_num_workers'] = 0


    # cfg_str = config['env_config']['cfg']
    # cfg = ast.literal_eval(cfg_str)
    cfg = config['env_config']['cfg']
    cfg['arch'] = arch
    # pprint.pprint(cfg['arch'])
    # pprint.pprint(type(cfg['arch']))
    
    # register model
    models = {
        'ac_route_dqn_model': AcRouteDQNModel,
        'ac_route_dqn_model_match': AcRouteDQNMatchModel,
        'ac_route_dqn_model_path': AcRouteDQNPathModel,
        'ac_route_ppo_model': AcRoutePPOModel,
        'ac_route_ppo_model_match': AcRoutePPOMatchModel,
        'ac_route_ppo_model_path': AcRoutePPOPathModel,
        'ac_route_ppo_model_path_group': AcRoutePPOPathGroupModel,
        'ac_route_ppo_top_level_model_path': AcRoutePPOPathTopLevelModel,
        'ac_route_ppo_mid_level_model_path': AcRoutePPOPathMidLevelModel,
    }
    for m_name, m in models.items():
        ModelCatalog.register_custom_model(m_name, m)
    
    # register env 
    envs = {
        'ac_route_rl_multicir_env_path_match': AcRouteMultiCirPathMatchFakeEnv,
        'ac_route_rl_multicir_env_path_match_group': AcRouteMultiCirPathMatchGroupFakeEnv,
        'ac_route_rl_multicir_hier_env_path_match': AcRouteMultiCirHierPathMatchFakeEnv,
    }
    if config['env'] in envs:
        register_env(config['env'], lambda c: envs[config['env']](c))
    else:
        assert False, 'No valid env specified!'

    
    trainers = {
        'dqn':    (dqn.DQNTrainer, dqn.DEFAULT_CONFIG),
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

def path_matching_results(cir: ac.cir_db,
                          cstr: ac.route_path_match_cstr):
    paths_res = []
    for j in range(cstr.num_path_cstrs()):
        path_cstr = cstr.path_cstr_const(j)
        res = []
        for k in range(path_cstr.num_conns()):
            src = path_cstr.src_pin_const(k)
            tar = path_cstr.tar_pin_const(k)
            r = cir.path_res(src, tar)
            res.append(r)
        paths_res.append(np.array(res))
    return paths_res


def print_path_res(path_res):
    with np.printoptions(precision=2, suppress=True):
        print(np.array(path_res), end='')



@hydra.main(config_path='.', config_name='config')
def main(cfg: DictConfig):

    bench = cfg.arch[cfg.bench]

    # init AutoCRAFT circuit
    cir = ac.cir_db()
    parser = ac.parser(cir)
    parser.parse(bench.laygo_config,
                 bench.grid,
                 bench.map,
                 bench.constraint,
                 bench.lef,
                 bench.netlist,
                 bench.drc)
    cir.group_cells()
    print('Circuit: {}'.format(cir.name()))

    # place
    place_mgr = ac.place_mgr(cir)
    if 'in_place' in bench and bench.in_place is not None:
        parser.parse_place(bench.in_place)
        place_mgr.gen_edge_cells()
        place_mgr.gen_fill_cells()
    else:
        place_mgr.solve_smt(bench.out_place)
    cir.gen_routing_bbox()

    # route
    cir.init_spatials()
    cir.build_spatial_pins_and_obs()
    cir.build_spatial_nets()

    route_mgr = ac.route_mgr(cir)
    dr_mgr = ac.route_dr_mgr(cir)

    power_net_ids = []
    for net in cir.v_nets():
        if net.is_power():
            power_net_ids.append(net.idx())
    
    if 'in_route_gds' in bench and bench.in_route_gds is not None:
        dr_mgr.init()
        parser.parse_route_gds(bench.in_route_gds, ac.VectorInt(power_net_ids))
    else:
        route_mgr.gen_pg()
        dr_mgr.init()

    dr_ps = ac.route_dr_ps(dr_mgr, 1)

    drc_mgr = ac.drc_mgr(cir)

    dr_ps.reset_history()
    # print(f'Cell: {cir.num_cells()}')
    # print(f'Net: {cir.num_nets()}')
    # print(f'Pin: {cir.num_pins()}')
    # path_cstrs = np.sum([c.num_path_cstrs() for c in cir.v_route_path_match_cstrs()])
    # print(f'Cstr: {path_cstrs}')
    # paths = np.sum([ np.sum([cc.num_conns() for cc in c.v_path_cstrs()]) for c in cir.v_route_path_match_cstrs() ])
    # print(f'Path: {paths}')
    # exit()

    # load trained RL
    arch = {cir.name(): cfg.arch[cir.name()]}
    agent, config = load_rl_model(arch,
                                  os.path.join(os.getcwd(), cfg.rl_config),
                                  os.path.join(os.getcwd(), cfg.rl_checkpoint))

    start = time.time()
    # init routables
    rl = ac.rl_utils()
    for i in range(cir.num_nets()):
        net = cir.net(i)
        if net.is_critical():
            dr_ps.construct_net_routables(net, False)
            # print(net.num_routables())
            # dr_ps.route(net, False, bench.route_dr_ps_drc_cost, bench.route_dr_ps_his_cost)
            # print(net.name(), net.is_routed(), net.num_arcs(), net.num_wires())
            # rl.to_vis_gds(cir, net, 'test.gds')

    # print('============== Iter 0 ==================')



    ac_rl = ac.rl_utils()

    # flat
    # for cstr in cir.v_route_path_match_cstrs():
        # paths_res_start = None

        # # init solution
        # dr_ps.run_nrr(ac.VectorInt([n.idx() for n in cstr.v_nets()]), False)
        # # dr_ps.route(cir.net('ckint090'), False, bench.route_dr_ps_drc_cost, bench.route_dr_ps_his_cost)
        # # dr_ps.route(cir.net('ckint000'), False, bench.route_dr_ps_drc_cost, bench.route_dr_ps_his_cost)
        # # dr_ps.route(cir.net('ckint180'), False, bench.route_dr_ps_drc_cost, bench.route_dr_ps_his_cost)
        # # dr_ps.route(cir.net('ckint270'), False, bench.route_dr_ps_drc_cost, bench.route_dr_ps_his_cost)
        # # k = np.random.randint(0, 3)
        # # for i in range(k):
            # # net_ = np.random.choice(cstr.v_nets())
            # # dr_ps.ripup(net_)
            # # dr_ps.route(net_, False, bench.route_dr_ps_drc_cost, bench.route_dr_ps_his_cost)

        # paths_res_start = path_matching_results(cir, cstr)
        # for i in range(cfg.rl_num_iter):
            # print('============== Iter {} =================='.format(i + 1))
            # net = ptp_net(cir, cstr)
            # print(f'pick {net.name()}')
            # rl_route_path_match_flat(bench, cir, net, dr_mgr, dr_ps, drc_mgr, agent, config)
            # print('----------Matching results----------')
            # print(cstr.name())
            # paths_res = path_matching_results(cir, cstr)
            # # if i == 0:
                # # paths_res_start = paths_res

            # d_avgs = []
            # d_stds = []
            # d_ptps = []
            # for j, (path_res_s, path_res_e) in enumerate(zip(paths_res_start, paths_res)):
                # avg_s, avg_e = path_res_s.mean(), path_res_e.mean()
                # std_s, std_e = path_res_s.std(), path_res_e.std()
                # ptp_s, ptp_e = path_res_s.ptp(), path_res_e.ptp()
                # d_avgs.append((avg_e - avg_s) / avg_s * 100)
                # d_stds.append((std_e - std_s) / std_s * 100)
                # d_ptps.append((ptp_e - ptp_s) / ptp_s * 100)
                # print(f'path_res_{j}')
                # print(f'Avg change: {avg_s:<10.3f} -> {avg_e:<10.3f} ({d_avgs[-1]}%)')
                # print(f'Std change: {std_s:<10.3f} -> {std_e:<10.3f} ({d_stds[-1]}%)')
                # print(f'Ptp change: {ptp_s:<10.3f} -> {ptp_e:<10.3f} ({d_ptps[-1]}%)')
            # print()

            # # for j, net in enumerate(cstr.v_nets()):
                # # p = [-1] * cstr.num_path_cstrs()
                # # for k, (path_cstr, idx) in enumerate(net.v_conns()):
                    # # p[path_cstr.idx()] = paths_res[path_cstr.idx()][idx]
                # # print(net.name(), 'path_res: ', end='')
                # # print_path_res(p)
                # # print()

            # print(f'match group: {cstr.name()}')
            # for path_cstr in cstr.v_path_cstrs():
                # print(f'path_cstr_{path_cstr.idx()}')
                # for j, (src, tar) in enumerate(path_cstr.v_conns()):
                    # print(f'{src.net().name()} ({src.name()} -> {tar.name()}): {paths_res[path_cstr.idx()][j]}')
                # print()
            
            # print(f'Mean Avg change: {np.array(d_avgs).mean()}')
            # print(f'Mean Std change: {np.array(d_stds).mean()}')
            # print(f'Mean Ptp change: {np.array(d_ptps).mean()}')
            # print()
    
    # flat (group)
    # dr_ps.run_nrr(False, ac.net_type_e.critical)
    # vv_ids = [0, 2, 1]
    # vv = [cir.route_path_match_cstr(i) for i in vv_ids]
    # for cstr in vv:
    for cstr in cir.v_route_path_match_cstrs():
        for net in cstr.v_nets():
            dr_ps.ripup(net)

        net_ids = [n.idx() for n in cstr.v_nets()]
        dr_ps.run_nrr(ac.VectorInt(net_ids), False)

        # dr_ps.reset_history()
        # np.random.shuffle(net_ids)
        # for net_idx in net_ids:
            # net = cir.net(net_idx)
            # if not net.is_routed():
                # dr_ps.route(net, False, bench.route_dr_ps_drc_cost, bench.route_dr_ps_his_cost)
      
        # for i in range(np.random.randint(5, 10)):
            # net_ = np.random.choice(cstr.v_nets())
            # dr_ps.ripup(net_)
            # dr_ps.route(net_, False, bench.route_dr_ps_drc_cost, bench.route_dr_ps_his_cost)

        for i in range(cfg.rl_num_iter):
            paths_res_start = path_matching_results(cir, cstr)
            print(f'Cstr {cstr.name()} Iteration: {i}')
            rl_route_path_match_group(bench, cir, cstr, dr_mgr, dr_ps, drc_mgr, agent, config)
            paths_res = path_matching_results(cir, cstr)
            print('----------Matching results----------')
            print(cstr.name())
            d_avgs = []
            d_stds = []
            d_ptps = []
            for j, (path_res_s, path_res_e) in enumerate(zip(paths_res_start, paths_res)):
                avg_s, avg_e = path_res_s.mean(), path_res_e.mean()
                std_s, std_e = path_res_s.std(), path_res_e.std()
                ptp_s, ptp_e = path_res_s.ptp(), path_res_e.ptp()
                d_avgs.append((avg_e - avg_s) / avg_s * 100)
                d_stds.append((std_e - std_s) / std_s * 100)
                d_ptps.append((ptp_e - ptp_s) / ptp_s * 100)
                print(f'path_res_{j}')
                print(f'Avg change: {avg_s:<10.3f} -> {avg_e:<10.3f} ({d_avgs[-1]}%)')
                print(f'Std change: {std_s:<10.3f} -> {std_e:<10.3f} ({d_stds[-1]}%)')
                print(f'Ptp change: {ptp_s:<10.3f} -> {ptp_e:<10.3f} ({d_ptps[-1]}%)')
            print()
            # print(f'match group: {cstr.name()}')
            # for path_cstr in cstr.v_path_cstrs():
                # print(f'path_cstr_{path_cstr.idx()}')
                # for j, (src, tar) in enumerate(path_cstr.v_conns()):
                    # print(f'{src.net().name()} ({src.name()} -> {tar.name()}): {paths_res[path_cstr.idx()][j]:.3f}')
                # print()

        # dr_ps.check_drc(ac.VectorInt(net_ids), False)
    print(time.time() - start)
    cir.show_route_wl()
    # print('AVG')
    # for i, cstr in enumerate(cir.v_route_path_match_cstrs()):
        # paths_res = path_matching_results(cir, cstr)
        # for j, path_res in enumerate(paths_res):
            # # print(f'Group{i+1}-{j+1}: {round(path_res.mean(), 3)}')
            # print(f'{round(path_res.mean(), 3)}')
    # print('STD')
    # for i, cstr in enumerate(cir.v_route_path_match_cstrs()):
        # paths_res = path_matching_results(cir, cstr)
        # for j, path_res in enumerate(paths_res):
            # # print(f'Group{i+1}-{j+1}: {round(path_res.std(), 3)}')
            # print(f'{round(path_res.std(), 3)}')
    # print('PTP')
    # for i, cstr in enumerate(cir.v_route_path_match_cstrs()):
        # paths_res = path_matching_results(cir, cstr)
        # for j, path_res in enumerate(paths_res):
            # # print(f'Group{i+1}-{j+1}: {round(path_res.ptp(), 3)}')
            # print(f'{round(path_res.ptp(), 3)}')

        

    # hier (group)
    # match_group = ['ckint000', 'ckint270', 'ckint180', 'ckint090']
    # dr_ps.run_nrr(ac.VectorInt([cir.net(n).idx() for n in match_group]), False)
    # rl_route_path_match_hier(bench, cir, match_group, dr_mgr, dr_ps, drc_mgr, agent, config)
    
    # exit()


    # post route
    pr = ac.post_route(cir)
    pr.patch()

    cir.gen_wsp()

    # output
    writer = ac.writer(cir)

    icc_path = cfg.icc_path
    writer.to_icc_tcl(icc_path + 'tcl/magic/' + cir.name() + '_pnr.tcl')
    writer.to_gv(icc_path + 'verilog/' + cir.name() + '.gv')


if __name__ == '__main__':
    main()

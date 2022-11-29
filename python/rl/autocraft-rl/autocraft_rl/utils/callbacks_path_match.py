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

from ray.rllib.evaluation import MultiAgentEpisode, RolloutWorker
from ray.rllib.agents.callbacks import DefaultCallbacks
from ray.rllib.env import BaseEnv
from typing import Callable, Dict, List, Optional, Tuple, Type, Union
from ray.rllib.policy import Policy
from ray.rllib.policy.sample_batch import SampleBatch
from ray.rllib.utils.typing import AgentID, PolicyID
from ray.rllib.evaluation.episode import Episode

import autocraft as ac
from autocraft_rl.utils.helper import segs_to_str, str_to_segs

import os
import pickle as pk
# import ujson
import orjson
import numpy as np


class AcRoutePathMatchCallbacks(DefaultCallbacks):
    # this callback add historgram of x,y positions of episodic state space
    def __init__(self,
                 *args,
                 **kwargs):
        super().__init__(*args, **kwargs)

        # os.makedirs('./topo_dict/', exist_ok=True)

        self.topo_dict = {}
        self.max_topo = 80000
    
    def on_episode_start(self,
                         *,
                         worker: RolloutWorker,
                         base_env: BaseEnv,
                         policies: Dict[str, Policy],
                         episode: Episode,
                         env_index: int,
                         **kwargs):
        # reset
        env_list = base_env.get_sub_environments()
        env = env_list[env_index]

        cir = env.cir
        net = env.net
        
        wl, via, drv = env.ac_rl.get_wl_via_drv(cir, net, env.route_dr_ps, env.drc_mgr, False)
        path_res = env.ac_rl.get_path_res(cir, net)

        episode.user_data['path_res_start'] = path_res
        episode.user_data['drv_start'] = drv

        if env.cfg.collect_topo and drv == 0:
            segs = ac.VectorSegment3dInt()
            net.v_arcs(segs)
            # segs = tuple(sorted(segs))
            num_segs = len(segs)
            segs = segs_to_str(sorted(segs))
            
            if cir.name() not in self.topo_dict:
                self.topo_dict[cir.name()] = {}
            if net.name() not in self.topo_dict[cir.name()]:
                self.topo_dict[cir.name()][net.name()] = {
                    # 'num_pins': net.num_pins()
                }
            nd = self.topo_dict[cir.name()][net.name()]
            if segs not in nd and len(nd) < self.max_topo:
                nd[segs] = {
                    'drv': drv,
                    'num_segs': num_segs,
                }
                d = self.topo_dict[cir.name()][net.name()][segs]
                d['wl'] = wl
                d['via'] = via
                d['res'] = cir.net_res(net)
                d['path_res'] = list(path_res)
                d['path_src'] = []
                d['path_snk'] = []
                for i in range(net.num_conns()):
                    d['path_src'].append(net.conn_src_pin(i).name()) 
                    d['path_snk'].append(net.conn_tar_pin(i).name()) 

    def on_episode_step(self,
                         *,
                         worker: RolloutWorker,
                         base_env: BaseEnv,
                         policies: Dict[str, Policy],
                         episode: Episode,
                         env_index: int,
                         **kwargs):

        env_list = base_env.get_sub_environments()
        env = env_list[env_index]

        if env.cfg.collect_topo:

            cir = env.cir
            net = env.net

            wl, via, drv = env.ac_rl.get_wl_via_drv(cir, net, env.route_dr_ps, env.drc_mgr, False)
            path_res = env.ac_rl.get_path_res(cir, net)

            if drv == 0:
                segs = ac.VectorSegment3dInt()
                net.v_arcs(segs)
                # segs = tuple(sorted(segs))
                num_segs = len(segs)
                segs = segs_to_str(sorted(segs))

                if cir.name() not in self.topo_dict:
                    self.topo_dict[cir.name()] = {}
                if net.name() not in self.topo_dict[cir.name()]:
                    self.topo_dict[cir.name()][net.name()] = {
                        # 'num_pins': net.num_pins()
                    }
                nd = self.topo_dict[cir.name()][net.name()]
                if segs not in nd and len(nd) < self.max_topo:
                    self.topo_dict[cir.name()][net.name()][segs] = {
                        'drv': drv,
                        'num_segs': num_segs,
                    }
                    d = self.topo_dict[cir.name()][net.name()][segs]
                    d['wl'] = wl
                    d['via'] = via
                    d['res'] = cir.net_res(net)
                    d['path_res'] = list(path_res)
                    d['path_src'] = []
                    d['path_snk'] = []
                    for i in range(net.num_conns()):
                        d['path_src'].append(net.conn_src_pin(i).name()) 
                        d['path_snk'].append(net.conn_tar_pin(i).name()) 


    def on_episode_end(self,
                       worker: RolloutWorker,
                       base_env: BaseEnv,
                       policies: Dict[str, Policy],
                       episode: Episode,
                       env_index: int,
                       **kwargs):
        env_list = base_env.get_sub_environments()
        env = env_list[env_index]

        cir = env.cir
        net = env.net
        
        wl, via, drv = env.ac_rl.get_wl_via_drv(cir, net, env.route_dr_ps, env.drc_mgr, False)
        path_res = env.ac_rl.get_path_res(cir, net)

        path_res_start = episode.user_data['path_res_start']
       
        path_res_target = np.array(env.target['path_res'])
        path_res_start = np.array(path_res_start)
        path_res = np.array(path_res)

        path_to_target_start = path_res_target - path_res_start
        path_to_target_end = path_res_target - path_res

        max_diff_end = np.abs(path_to_target_end)
        improvement = np.abs(path_to_target_start) - max_diff_end

        episode.custom_metrics['min_path_res_improvements'] = improvement.min()
        episode.custom_metrics['max_path_res_to_target_abs'] = max_diff_end.max()
        episode.custom_metrics['mean_path_res_to_target_abs'] = max_diff_end.mean()

        episode.hist_data['min_path_res_improvements'] = [improvement.min()]
        episode.hist_data['max_path_res_to_target_abs'] = [max_diff_end.max()]
        episode.hist_data['mean_path_res_to_target_abs'] = [max_diff_end.mean()]

        
        episode.custom_metrics['drv_improvements'] = episode.user_data['drv_start'] - drv
        episode.hist_data['drv_improvements'] = [episode.user_data['drv_start'] - drv]

        if env.cfg.collect_topo:
            # with open('./topo_dict/topo_dict_{}.pkl'.format(worker.worker_index), 'wb') as f:
                # pk.dump(self.topo_dict, f, protocol=pk.HIGHEST_PROTOCOL)
            with open(f'./topo_dict/topo_dict_{worker.worker_index}.json', 'w', encoding='utf-8') as f:
                # ujson.dump(self.topo_dict, f, ensure_ascii=False, indent=4)
                f.write(orjson.dumps(self.topo_dict, f).decode("utf-8"))

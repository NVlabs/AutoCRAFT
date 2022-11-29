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


class AcRouteLayerMatchCallbacks(DefaultCallbacks):
    # this callback add historgram of x,y positions of episodic state space
    def __init__(self,
                 *args,
                 **kwargs):
        super().__init__(*args, **kwargs)

        os.makedirs('./topo_dict/', exist_ok=True)

        self.topo_dict = {}
        self.max_topo = 50000
    
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


        wl = ac.VectorFloat()
        via = ac.VectorFloat()
        wl_tot, via_tot, drv = env.ac_rl.get_layers_wl_via_drv(cir, net, env.route_dr_ps, env.drc_mgr, wl, via, False)

        episode.user_data['wl_start'] = (wl, wl_tot)
        episode.user_data['via_start'] = (via, via_tot)
        episode.user_data['drv_start'] = drv

        if env.cfg.collect_topo and drv == 0:
            segs = ac.VectorSegment3dInt()
            net.v_arcs(segs)
            # segs = tuple(sorted(segs))
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
                    'num_segs': len(segs),
                }
                d = self.topo_dict[cir.name()][net.name()][segs]
                d['wl'] = wl_tot
                for i, l in enumerate(wl):
                    d['wl_m{}'.format(i + 1)] = l
                d['via'] = via_tot
                for i, v in enumerate(via):
                    d['via_v{}'.format(i + 1)] = v




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

            wl = ac.VectorFloat()
            via = ac.VectorFloat()
            wl_tot, via_tot, drv = env.ac_rl.get_layers_wl_via_drv(cir, net, env.route_dr_ps, env.drc_mgr, wl, via, False)

            if drv == 0:
                segs = ac.VectorSegment3dInt()
                net.v_arcs(segs)
                # segs = tuple(sorted(segs))
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
                        'num_segs': len(segs),
                    }
                    d = self.topo_dict[cir.name()][net.name()][segs]
                    d['wl'] = wl_tot
                    for i, l in enumerate(wl):
                        d['wl_m{}'.format(i + 1)] = l
                    d['via'] = via_tot
                    for i, v in enumerate(via):
                        d['via_v{}'.format(i + 1)] = v

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
        
        wl = ac.VectorFloat()
        via = ac.VectorFloat()
        wl_tot, via_tot, drv = env.ac_rl.get_layers_wl_via_drv(cir, net, env.route_dr_ps, env.drc_mgr, wl, via, False)
      

        wl_start, wl_tot_start = episode.user_data['wl_start']
        wl_tot_t = env.target['wl']
        episode.custom_metrics['wl_to_target'] = wl_tot_t - wl_tot
        for i, wl_c in enumerate(wl):
            wl_t = env.target['wl_m{:d}'.format(i + 1)]
            wl_p = wl_start[i]
            episode.custom_metrics['wl_to_target_m{:d}'.format(i + 1)] = wl_t - wl_c
            episode.custom_metrics['wl_improvements_m{:d}'.format(i + 1)] = (abs(wl_p - wl_t) - abs(wl_c - wl_t)) / net.bbox().hpwl()


        via_start, via_tot_start = episode.user_data['via_start']
        via_tot_t = env.target['via']
        episode.custom_metrics['via_to_target'] = via_tot_t - via_tot
        for i, via_c in enumerate(via):
            via_t = env.target['via_v{:d}'.format(i + 1)]
            via_p = via_start[i]
            episode.custom_metrics['via_to_target_v{:d}'.format(i + 1)] = via_t - via_c
            episode.custom_metrics['via_improvements_v{:d}'.format(i + 1)] = (abs(via_p - via_t) - abs(via_c - via_t)) / env.cfg.rl_via_norm_constant
        
        episode.custom_metrics['drv_improvements'] = episode.user_data['drv_start'] - drv

        if env.cfg.collect_topo:
            # with open('./topo_dict/topo_dict_{}.pkl'.format(worker.worker_index), 'wb') as f:
                # pk.dump(self.topo_dict, f, protocol=pk.HIGHEST_PROTOCOL)
            with open('./topo_dict/topo_dict_{}.json'.format(worker.worker_index), 'w', encoding='utf-8') as f:
                # ujson.dump(self.topo_dict, f, ensure_ascii=False, indent=4)
                f.write(orjson.dumps(self.topo_dict, f).decode("utf-8"))

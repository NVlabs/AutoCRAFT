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


class AcRouteMatchCallbacks(DefaultCallbacks):
    # this callback add historgram of x,y positions of episodic state space
    def __init__(self,
                 *args,
                 **kwargs):
        super().__init__(*args, **kwargs)

        os.makedirs('./topo_dict/', exist_ok=True)

        self.topo_dict = {}
    
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


        (wl, via, drv) = env.ac_rl.get_wl_via_drv(cir, net, env.route_dr_ps, env.drc_mgr, False)

        episode.user_data['wl_start'] = wl
        episode.user_data['via_start'] = via
        episode.user_data['drv_start'] = drv
        # episode.user_data['segs_start'] = net.num_arcs()

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
            if segs not in self.topo_dict[cir.name()][net.name()]:
                self.topo_dict[cir.name()][net.name()][segs] = {
                    'wl': wl,
                    'via': via,
                    'drv': drv,
                    'num_segs': num_segs,
                }


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

            (wl, via, drv) = env.ac_rl.get_wl_via_drv(cir, net, env.route_dr_ps, env.drc_mgr, False)

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
                if segs not in self.topo_dict[cir.name()][net.name()]:
                    self.topo_dict[cir.name()][net.name()][segs] = {
                        'wl': wl,
                        'via': via,
                        'drv': drv,
                        'num_segs': num_segs,
                    }

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
        
        (wl, via, drv) = env.ac_rl.get_wl_via_drv(cir, net, env.route_dr_ps, env.drc_mgr, False)
       
        wl_to_target = env.target_wl - wl
        via_to_target = env.target_via - via
        
        # episode.custom_metrics['wl_improvements'] = (abs(episode.user_data['wl_start'] - env.target_wl) - abs(wl - env.target_wl)) / env.target_wl
        # episode.custom_metrics['via_improvements'] = (abs(episode.user_data['via_start'] - env.target_via) - abs(via - env.target_via)) / env.target_via
        episode.custom_metrics['wl_improvements'] = (abs(episode.user_data['wl_start'] - env.target_wl) - abs(wl - env.target_wl)) / env.target_wl
        episode.custom_metrics['via_improvements'] = (abs(episode.user_data['via_start'] - env.target_via) - abs(via - env.target_via)) / env.target_via
        episode.custom_metrics['drv_improvements'] = episode.user_data['drv_start'] - drv
        episode.custom_metrics['wl_to_target'] = wl_to_target
        episode.custom_metrics['via_to_target'] = via_to_target
        # episode.custom_metrics['drv_start'] = episode.user_data['drv_start']
        episode.custom_metrics['drv_end'] = drv
        # episode.custom_metrics['segs_start'] = episode.user_data['segs_start']
        # episode.custom_metrics['segs_end'] = net.num_arcs()

        if env.cfg.collect_topo:
            # with open('./topo_dict/topo_dict_{}.pkl'.format(worker.worker_index), 'wb') as f:
                # pk.dump(self.topo_dict, f, protocol=pk.HIGHEST_PROTOCOL)
            with open('./topo_dict/topo_dict_{}.json'.format(worker.worker_index), 'w', encoding='utf-8') as f:
                # ujson.dump(self.topo_dict, f, ensure_ascii=False, indent=4)
                f.write(orjson.dumps(self.topo_dict, f).decode("utf-8"))

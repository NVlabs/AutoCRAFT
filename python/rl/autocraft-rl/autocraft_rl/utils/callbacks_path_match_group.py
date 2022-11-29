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


class AcRoutePathMatchGroupCallbacks(DefaultCallbacks):
    # this callback add historgram of x,y positions of episodic state space
    def __init__(self,
                 *args,
                 **kwargs):
        super().__init__(*args, **kwargs)

    
    def on_episode_start(self,
                         *,
                         worker: RolloutWorker,
                         base_env: BaseEnv,
                         policies: Dict[str, Policy],
                         episode: Episode,
                         env_index: int,
                         **kwargs):
        # env_list = base_env.get_sub_environments()
        # env = env_list[env_index]

        # cir = env.cir
        # match_group = env.match_group

        # wls = []
        # vias = []
        # drvs = []
        # pd = {}
        # for net in match_group:
            # path_data = env.ac_rl.get_path_res(cir, net)
            # wl, via, drv = env.ac_rl.get_wl_via_drv(cir,
                                                 # net,
                                                 # env.flat_env.route_dr_ps,
                                                 # env.flat_env.drc_mgr,
                                                 # False)
            # wls.append(wl)
            # vias.append(via)
            # drvs.append(drv)
            # for j in range(net.num_conns()):
                # cstr = net.conn_cstr_const(j)
                # if cstr.idx() not in pd:
                    # pd[cstr.idx()] = []
                # pd[cstr.idx()].append(path_data[j])

        # ptp = np.array([np.ptp(x) for x in pd.values()])
        # std = np.array([np.std(x) for x in pd.values()])

        # wls = np.array(wls)
        # vias = np.array(vias)
        # drvs = np.array(drvs)

        # episode.user_data['ptp_start'] = ptp
        # episode.user_data['wl_start'] = wls
        # episode.user_data['via_start'] = vias
        # episode.user_data['drvs_start'] = drvs
        pass


    def on_episode_step(self,
                         *,
                         worker: RolloutWorker,
                         base_env: BaseEnv,
                         policies: Dict[str, Policy],
                         episode: Episode,
                         env_index: int,
                         **kwargs):
        pass


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
        match_group = env.match_group
       
        wls = []
        vias = []
        drvs = []
        pd = {}
        for net in match_group:
            path_data = env.ac_rl.get_path_res(cir, net)
            wl, via, drv = env.ac_rl.get_wl_via_drv(cir,
                                                    net,
                                                    env.dr_ps,
                                                    env.drc_mgr,
                                                    False)
            wls.append(wl)
            vias.append(via)
            drvs.append(drv)
            for j in range(net.num_conns()):
                cstr = net.conn_cstr_const(j)
                if cstr.idx() not in pd:
                    pd[cstr.idx()] = []
                pd[cstr.idx()].append(path_data[j])

        ptp = np.array([np.ptp(x) for x in pd.values()])
        std = np.array([np.std(x) for x in pd.values()])

        wls = np.array(wls)
        vias = np.array(vias)
        drvs = np.array(drvs)

        # paths
        max_ptp = ptp.max()
        min_ptp = ptp.min()
        avg_ptp = ptp.mean()
        episode.custom_metrics['max_ptp'] = max_ptp
        episode.custom_metrics['min_ptp'] = min_ptp
        episode.custom_metrics['avg_ptp'] = avg_ptp
        episode.hist_data['max_ptp'] = [max_ptp]
        episode.hist_data['min_ptp'] = [min_ptp]
        episode.hist_data['avg_ptp'] = [avg_ptp]

       
        # wl, via, drv
        avg_wl = wls.mean()
        avg_via = vias.mean()
        max_drv, min_drv, avg_drv = drvs.max(), drvs.min(), drvs.mean()

        episode.custom_metrics['avg_wl']  = avg_wl
        episode.custom_metrics['avg_via'] = avg_via
        episode.custom_metrics['max_drv'] = max_drv
        episode.custom_metrics['min_drv'] = min_drv
        episode.custom_metrics['avg_drv'] = avg_drv
        episode.hist_data['avg_wl']  = [avg_wl]
        episode.hist_data['avg_via'] = [avg_via]
        episode.hist_data['max_drv'] = [max_drv]
        episode.hist_data['min_drv'] = [min_drv]
        episode.hist_data['avg_drv'] = [avg_drv]



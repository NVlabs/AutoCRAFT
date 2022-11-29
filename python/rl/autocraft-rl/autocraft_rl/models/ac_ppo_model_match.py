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

import gym
from typing import Dict, List

import numpy as np

import torch as th

from ray.tune.logger import pretty_print
from ray.rllib.models.torch.torch_modelv2 import TorchModelV2
from ray.rllib.utils.annotations import override
from ray.rllib.utils.typing import ModelConfigDict, TensorType

import dgl
from dgl.nn import SAGEConv, GATConv


class AcRoutePPOMatchModel(TorchModelV2, th.nn.Module):
    def __init__(self,
                 obs_space: gym.spaces.Space,
                 action_space: gym.spaces.Space,
                 num_outputs: int,
                 model_config: ModelConfigDict,
                 name: str,
                 **customized_model_kwargs):

        TorchModelV2.__init__(self, obs_space, action_space, num_outputs, model_config, name)
        th.nn.Module.__init__(self)

        # print(pretty_print(customized_model_kwargs))

        self.max_segs = model_config['custom_model_config']['max_segs']
        self.input_dim = model_config['custom_model_config']['seg_feat_dim']
        self.gnn_feat_dim = model_config['custom_model_config']['seg_to_seg_gnn_feat_dim']
        self.q_module_hiddens = model_config['custom_model_config']['advantage_module_hiddens']
        self.value_module_hiddens = model_config['custom_model_config']['value_module_hiddens']
        self.gnn_sage_aggregator = model_config['custom_model_config']['seg_to_seg_gnn_sage_aggregator']

        # custom embedding module
        assert len(self.gnn_feat_dim) > 0
        self.embed_gnn_module = dgl.nn.Sequential()
        in_feats = self.input_dim
        for i, n in enumerate(self.gnn_feat_dim):
            self.embed_gnn_module.add_module('sage_{}'.format(i), SAGEConv(in_feats=in_feats, out_feats=n, aggregator_type=self.gnn_sage_aggregator, activation=th.nn.ReLU()))
            in_feats = n
        

        embed_dim = in_feats

        # model to action
        self.q_module = th.nn.Sequential()
        in_feats = embed_dim * 2 + 2
        for i, n in enumerate(self.q_module_hiddens):
            self.q_module.add_module('fc_{}'.format(i), th.nn.Linear(in_features=in_feats, out_features=n))
            self.q_module.add_module('relu_{}'.format(i), th.nn.ReLU())
            in_feats = n
        self.q_module.add_module('fc_final', th.nn.Linear(in_features=in_feats, out_features=1))
       
        # value
        self.value_module = th.nn.Sequential()
        in_feats = embed_dim + 2
        for i, n in enumerate(self.value_module_hiddens):
            self.value_module.add_module('fc_{}'.format(i), th.nn.Linear(in_features=in_feats, out_features=n))
            self.value_module.add_module('relu_{}'.format(i), th.nn.ReLU())
            in_feats = n
        self.value_module.add_module('fc_final', th.nn.Linear(in_features=in_feats, out_features=1))


        self._last_batch_size = None
        self._max_batch_size = model_config['custom_model_config']['batch_size']

        self.max_edges = 12 * self.max_segs
        
        self.edges_shift_mask = th.zeros(self._max_batch_size * self.max_edges, dtype=th.int32)
        for i in range(self._max_batch_size):
            self.edges_shift_mask[i * self.max_edges : (i + 1) * self.max_edges] = th.full((1, self.max_edges), i * self.max_segs)
        
        # to train or not
        # if 'fixed_weights' in model_config['custom_model_config'] and model_config['custom_model_config']['fixed_weights']:
            # for _, param in self._parameters.items():
                # param.requires_grad = False


    @override(TorchModelV2)
    def forward(self,
               input_dict: Dict[str, TensorType],
               state: List[TensorType],
               seq_lens: TensorType):

        obs = input_dict['obs']

        action_mask = obs['action_mask']
        feats = obs['feats']
        edges = obs['edges']
        valid_nodes = obs['valid_nodes']
        valid_edges = obs['valid_edges']
        
        target_wl = obs['target_wl_norm']
        target_via = obs['target_via_norm']

        self._last_batch_size = feats.shape[0]

        device = feats.device
        
        edges = th.as_tensor(edges, dtype=th.int32)
        edges = th.flatten(edges, start_dim=0, end_dim=1)
        edges = th.transpose(edges, 0, 1)

        edges_shift_mask = self.edges_shift_mask[:edges.shape[1]].to(device)

        edges = th.add(edges, edges_shift_mask)

        valid_edges = th.as_tensor(valid_edges, dtype=bool)
        valid_edges = th.flatten(valid_edges)

        edges = edges[:, valid_edges]


        bg = dgl.graph((edges[0], edges[1]), num_nodes=self._last_batch_size * feats.shape[1])

        feats = th.flatten(feats, start_dim=0, end_dim=1)
        bg.ndata['feat'] = feats

        h = self.embed_gnn_module(bg, bg.ndata['feat'])
        # if self.embed_fc_module is not None:
            # h = self.embed_fc_module(h)

        h = h.view(self._last_batch_size, self.max_segs, -1)

        valid_nodes = th.as_tensor(valid_nodes, dtype=bool)
        vn = th.zeros(h.shape, dtype=bool).to(device)
        vn[valid_nodes] = True
        avg = (th.sum(h * vn.float(), dim=1) / th.sum(vn, dim=1))
        avg[avg != avg] = 0 # nan -> 0
        avg = avg.view(avg.shape[0], 1, -1)

        target_wl = th.unsqueeze(target_wl, dim=1)
        target_via = th.unsqueeze(target_via, dim=1)

        h = th.cat((h, avg), dim=1)
        avg_repeat = avg.repeat(1, h.shape[1], 1)
        target_wl_repeat = target_wl.repeat(1, h.shape[1], 1)
        target_via_repeat = target_via.repeat(1, h.shape[1], 1)

        h = th.cat((h, avg_repeat, target_wl_repeat, target_via_repeat), dim=-1)

        h = self.q_module(h)
        h = th.squeeze(h, -1)

        action_mask = (action_mask == 1)
        assert action_mask.shape == h.shape

        h[action_mask] = float('-inf')

        avg = th.squeeze(avg, 1)
        target_wl = th.squeeze(target_wl, 1)
        target_via = th.squeeze(target_via, 1)
        y = th.cat((avg, target_wl, target_via), dim=-1)
        self._value_out = th.flatten(self.value_module(y))
    
        return h, state

    @override(TorchModelV2)
    def value_function(self) -> TensorType:
        return self._value_out

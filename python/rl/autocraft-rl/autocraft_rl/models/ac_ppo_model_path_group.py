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
from dgl.nn import SAGEConv
from dgl.nn.pytorch import HeteroGraphConv

from pprint import pprint


class AcRoutePPOPathGroupModel(TorchModelV2, th.nn.Module):
    def __init__(self,
                 obs_space: gym.spaces.Space,
                 action_space: gym.spaces.Space,
                 num_outputs: int,
                 model_config: ModelConfigDict,
                 name: str,
                 **customized_model_kwargs):

        TorchModelV2.__init__(self, obs_space, action_space, num_outputs, model_config, name)
        th.nn.Module.__init__(self)

        self.seg_feat_dim = model_config['custom_model_config']['seg_feat_dim']
        self.path_feat_dim = model_config['custom_model_config']['path_feat_dim']
        self.super_feat_dim = model_config['custom_model_config']['super_feat_dim']

        self.seg_to_seg_gnn_feat_dim = model_config['custom_model_config']['seg_to_seg_gnn_feat_dim']
        self.seg_to_path_gnn_feat_dim = model_config['custom_model_config']['seg_to_path_gnn_feat_dim']
        self.path_to_super_gnn_feat_dim = model_config['custom_model_config']['path_to_super_gnn_feat_dim']
        self.super_to_seg_gnn_feat_dim = model_config['custom_model_config']['super_to_seg_gnn_feat_dim']

        self.seg_to_seg_gnn_sage_aggregator = model_config['custom_model_config']['seg_to_seg_gnn_sage_aggregator']
        self.seg_to_path_gnn_sage_aggregator = model_config['custom_model_config']['seg_to_path_gnn_sage_aggregator']
        self.path_to_super_gnn_sage_aggregator = model_config['custom_model_config']['path_to_super_gnn_sage_aggregator']
        self.super_to_seg_gnn_sage_aggregator = model_config['custom_model_config']['super_to_seg_gnn_sage_aggregator']
        
        self.q_module_hiddens = model_config['custom_model_config']['advantage_module_hiddens']
        self.value_module_hiddens = model_config['custom_model_config']['value_module_hiddens']

        # seg-to-seg module
        assert len(self.seg_to_seg_gnn_feat_dim) > 0
        self.seg_to_seg_module = dgl.nn.Sequential()
        x_seg = self.seg_feat_dim
        for i, n in enumerate(self.seg_to_seg_gnn_feat_dim):
            self.seg_to_seg_module.add_module(f'sage_{i}', SAGEConv(in_feats=(x_seg, x_seg), out_feats=n, aggregator_type=self.seg_to_seg_gnn_sage_aggregator, activation=th.nn.ReLU()))
            x_seg = n

        # seg-to-path module
        assert len(self.seg_to_path_gnn_feat_dim) == 1
        x_path = self.path_feat_dim - 1 # no need for target feature here
        self.seg_to_path_module = dgl.nn.Sequential()
        self.seg_to_path_module.add_module('sage_0', SAGEConv(in_feats=(x_seg, x_path), out_feats=self.seg_to_path_gnn_feat_dim[-1], aggregator_type=self.seg_to_path_gnn_sage_aggregator, activation=th.nn.ReLU()))
        x_path = self.seg_to_path_gnn_feat_dim[-1]

        # path_to_super module
        assert len(self.path_to_super_gnn_feat_dim) == 1
        x_super = self.super_feat_dim
        self.path_to_super_module = dgl.nn.Sequential()
        self.path_to_super_module.add_module('sage_0', SAGEConv(in_feats=(x_path, x_super), out_feats=self.path_to_super_gnn_feat_dim[-1], aggregator_type=self.path_to_super_gnn_sage_aggregator, activation=th.nn.ReLU()))
        x_super = self.path_to_super_gnn_feat_dim[-1]

        # super_to_seg module
        assert len(self.super_to_seg_gnn_feat_dim) == 1
        self.super_to_seg_module = dgl.nn.Sequential()
        self.super_to_seg_module.add_module('sage_0', SAGEConv(in_feats=(x_super, x_seg), out_feats=self.super_to_seg_gnn_feat_dim[-1], aggregator_type=self.super_to_seg_gnn_sage_aggregator, activation=th.nn.ReLU()))

        # GNN
        def my_agg_func(tensors, dsttype):
            h = th.cat(tensors, dim=1)
            return h

        self.hetero_conv = HeteroGraphConv({
            'seg_to_seg': self.seg_to_seg_module,
            'seg_to_path': self.seg_to_path_module,
            'path_to_super': self.path_to_super_module,
            'super_to_seg': self.super_to_seg_module,
        }, aggregate=my_agg_func)
        
        self._max_batch_size = model_config['custom_model_config']['batch_size']
        
        self.max_segs_per_net = model_config['custom_model_config']['max_segs'] 
        self.max_paths_per_net = model_config['custom_model_config']['max_paths']

        self.max_nets_per_group = model_config['custom_model_config']['max_nets_per_group']
        self.max_segs_per_group = self.max_nets_per_group * self.max_segs_per_net
        self.max_paths_per_group = self.max_nets_per_group * self.max_paths_per_net
        self.max_supers_per_group = model_config['custom_model_config']['max_supers']

        self.max_seg_to_seg_edges = self.max_nets_per_group * 12 * self.max_segs_per_net
        self.max_seg_to_path_edges = self.max_nets_per_group * self.max_segs_per_net * self.max_paths_per_net
        self.max_path_to_super_edges = self.max_nets_per_group * self.max_paths_per_net
        self.max_super_to_seg_edges =self.max_supers_per_group * self.max_segs_per_group

        self.seg_to_seg_edges_shift_mask = th.zeros(self._max_batch_size * self.max_seg_to_seg_edges, dtype=th.int32)
        self.seg_to_path_edges_shift_mask = th.zeros(2, self._max_batch_size * self.max_seg_to_path_edges, dtype=th.int32)
        self.path_to_super_edges_shift_mask = th.zeros(2, self._max_batch_size * self.max_path_to_super_edges, dtype=th.int32)
        self.super_to_seg_edges_shift_mask = th.zeros(self._max_batch_size * self.max_super_to_seg_edges, dtype=th.int32)

        for i in range(self._max_batch_size):
            self.seg_to_seg_edges_shift_mask[i * self.max_seg_to_seg_edges : (i + 1) * self.max_seg_to_seg_edges] = th.full((1, self.max_seg_to_seg_edges), i * self.max_segs_per_group)
            self.seg_to_path_edges_shift_mask[0][i * self.max_seg_to_path_edges : (i + 1) * self.max_seg_to_path_edges] = th.full((1, self.max_seg_to_path_edges), i * self.max_segs_per_group)
            self.seg_to_path_edges_shift_mask[1][i * self.max_seg_to_path_edges : (i + 1) * self.max_seg_to_path_edges] = th.full((1, self.max_seg_to_path_edges), i * self.max_paths_per_group)
            self.path_to_super_edges_shift_mask[0][i * self.max_path_to_super_edges : (i + 1) * self.max_path_to_super_edges] = th.full((1, self.max_path_to_super_edges), i * self.max_paths_per_group)
            self.path_to_super_edges_shift_mask[1][i * self.max_path_to_super_edges : (i + 1) * self.max_path_to_super_edges] = th.full((1, self.max_path_to_super_edges), i * self.max_supers_per_group)
            self.super_to_seg_edges_shift_mask[i * self.max_super_to_seg_edges : (i + 1) * self.max_super_to_seg_edges] = th.full((1, self.max_super_to_seg_edges), i * self.max_supers_per_group)


        val_in_feats = self.super_to_seg_gnn_feat_dim[-1]
        adv_in_feats = val_in_feats

        # Q module
        self.q_module = th.nn.Sequential()
        for i, n in enumerate(self.q_module_hiddens):
            self.q_module.add_module(f'fc_{i}', th.nn.Linear(in_features=adv_in_feats, out_features=n))
            self.q_module.add_module(f'relu_{i}', th.nn.ReLU())
            adv_in_feats = n
        self.q_module.add_module('fc_final', th.nn.Linear(in_features=adv_in_feats, out_features=1))

        # value module
        self.value_module = th.nn.Sequential()
        for i, n in enumerate(self.value_module_hiddens):
            self.value_module.add_module(f'fc_{i}', th.nn.Linear(in_features=val_in_feats, out_features=n))
            self.value_module.add_module(f'relu_{i}', th.nn.ReLU())
            val_in_feats = n
        self.value_module.add_module('fc_final', th.nn.Linear(in_features=val_in_feats, out_features=1))

    def build_hetero_graph(self,
                           obs: Dict[str, TensorType],
                           device):
        seg_feats = obs['seg_feats']
        path_feats = obs['path_feats']
        super_feats = obs['super_feats']

        seg_to_seg_edges = th.as_tensor(obs['seg_to_seg_edges'], dtype=th.int32)
        seg_to_path_edges = th.as_tensor(obs['seg_to_path_edges'], dtype=th.int32)
        path_to_super_edges = th.as_tensor(obs['path_to_super_edges'], dtype=th.int32)
        super_to_seg_edges = th.as_tensor(obs['super_to_seg_edges'], dtype=th.int32)

        seg_to_seg_valid_edges = th.as_tensor(obs['seg_to_seg_valid_edges'], dtype=bool)
        seg_to_path_valid_edges = th.as_tensor(obs['seg_to_path_valid_edges'], dtype=bool)
        path_to_super_valid_edges = th.as_tensor(obs['path_to_super_valid_edges'], dtype=bool)
        super_to_seg_valid_edges = th.as_tensor(obs['super_to_seg_valid_edges'], dtype=bool)

        seg_to_seg_edges = th.transpose(th.flatten(seg_to_seg_edges, start_dim=0, end_dim=1), 0, 1)
        seg_to_path_edges = th.transpose(th.flatten(seg_to_path_edges, start_dim=0, end_dim=1), 0, 1)
        path_to_super_edges = th.transpose(th.flatten(path_to_super_edges, start_dim=0, end_dim=1), 0, 1)
        super_to_seg_edges = th.transpose(th.flatten(super_to_seg_edges, start_dim=0, end_dim=1), 0, 1)

        seg_to_seg_edges_shift_mask = self.seg_to_seg_edges_shift_mask[:seg_to_seg_edges.shape[1]].to(device)
        seg_to_path_edges_shift_mask = self.seg_to_path_edges_shift_mask[:, :seg_to_path_edges.shape[1]].to(device)
        path_to_super_edges_shift_mask = self.path_to_super_edges_shift_mask[:, :path_to_super_edges.shape[1]].to(device)
        super_to_seg_edges_shift_mask = self.super_to_seg_edges_shift_mask[:super_to_seg_edges.shape[1]].to(device)

        seg_to_seg_edges = th.add(seg_to_seg_edges, seg_to_seg_edges_shift_mask)
        seg_to_path_edges = th.add(seg_to_path_edges, seg_to_path_edges_shift_mask)
        path_to_super_edges = th.add(path_to_super_edges, path_to_super_edges_shift_mask)
        super_to_seg_edges = th.add(super_to_seg_edges, super_to_seg_edges_shift_mask)

        seg_to_seg_valid_edges = th.flatten(seg_to_seg_valid_edges)
        seg_to_path_valid_edges = th.flatten(seg_to_path_valid_edges)
        path_to_super_valid_edges = th.flatten(path_to_super_valid_edges)
        super_to_seg_valid_edges = th.flatten(super_to_seg_valid_edges)
        
        seg_to_seg_edges = seg_to_seg_edges[:, seg_to_seg_valid_edges]
        seg_to_path_edges = seg_to_path_edges[:, seg_to_path_valid_edges]
        path_to_super_edges = path_to_super_edges[:, path_to_super_valid_edges]
        super_to_seg_edges = super_to_seg_edges[:, super_to_seg_valid_edges]

        data_dict = {
            ('seg', 'seg_to_seg', 'seg'): (seg_to_seg_edges[0], seg_to_seg_edges[1]),
            ('seg', 'seg_to_path', 'path'): (seg_to_path_edges[0], seg_to_path_edges[1]),
            ('path', 'path_to_super', 'super'): (path_to_super_edges[0], path_to_super_edges[1]),
            ('super', 'super_to_seg', 'seg'): (super_to_seg_edges[0], super_to_seg_edges[1]),
        }
        num_nodes_dict = {
            'seg': self._last_batch_size * seg_feats.shape[1],
            'path': self._last_batch_size * path_feats.shape[1],
            'super': self._last_batch_size * super_feats.shape[1],
        }
        
        seg_feats = th.flatten(seg_feats, start_dim=0, end_dim=1)
        path_feats = th.flatten(path_feats, start_dim=0, end_dim=1)
        super_feats = th.flatten(super_feats, start_dim=0, end_dim=1)

        bhg = dgl.heterograph(data_dict, num_nodes_dict, device=device)
        
        bhg.nodes['seg'].data['feat'] = seg_feats
        bhg.nodes['path'].data['feat'] = path_feats
        bhg.nodes['super'].data['feat'] = super_feats

        return bhg


    @override(TorchModelV2)
    def forward(self,
               input_dict: Dict[str, TensorType],
               state: List[TensorType],
               seq_lens: TensorType):

        obs = input_dict['obs']


        action_mask = obs['action_mask']
        self._last_batch_size = action_mask.shape[0]
        device = action_mask.device

        bhg = self.build_hetero_graph(obs, device)

        h1 = self.hetero_conv(bhg, (
            {
                'seg': bhg.nodes['seg'].data['feat'],
            },
            {
                'seg': bhg.nodes['seg'].data['feat'],
            }))
        h2 = self.hetero_conv(bhg, (
            {
                'seg': h1['seg'],
            },
            {
                'path': bhg.nodes['path'].data['feat'],
            }))
        h3 = self.hetero_conv(bhg, (
            {
                'path': h2['path'],
            },
            {
                'super': bhg.nodes['super'].data['feat'],
            }))
        h4 = self.hetero_conv(bhg, (
            {
                'super': h3['super'],
            },
            {
                'seg': h1['seg'],
            }))
        h = h4['seg']
        h = h.view(self._last_batch_size, self.max_segs_per_group, -1)
       
        # avg segs 
        seg_valid_nodes = th.as_tensor(obs['seg_valid_nodes'], dtype=bool)
        vn = th.zeros(h.shape, dtype=bool).to(device)
        vn[seg_valid_nodes] = True
        avg_all = (th.sum(h * vn.float(), dim=1) / th.sum(vn, dim=1))
        avg_all[avg_all != avg_all] = 0 # nan -> 0
        avg_all = avg_all.view(avg_all.shape[0], 1, -1)

        h = th.cat((h, avg_all), dim=1)

        h = self.q_module(h)
        h = th.squeeze(h, -1)

        action_mask = (action_mask == 1)

        h[action_mask] = float('-inf')

        # avg supers (for value function)
        # super_valid_nodes = th.as_tensor(obs['super_valid_nodes'], dtype=bool)
        # h_sup = h3['super'].view(self._last_batch_size, self.max_supers_per_group, -1)
        # vs = th.zeros(h_sup.shape, dtype=bool).to(device)
        # vs[super_valid_nodes] = True
        # avg_sup = (th.sum(h_sup * vs.float(), dim=1) / th.sum(vs, dim=1))
        # avg_sup[avg_sup != avg_sup] = 0 # nan -> 0
        # avg_sup = avg_sup.view(avg_sup.shape[0], 1, -1)
        # y = th.squeeze(avg_sup, 1)

        y = th.squeeze(avg_all, 1)
        self._value_out = th.flatten(self.value_module(y))

        return h, state



    @override(TorchModelV2)
    def value_function(self) -> TensorType:
        return self._value_out

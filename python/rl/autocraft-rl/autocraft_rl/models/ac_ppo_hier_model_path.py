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
################################################
#                Shared                        #
################################################
class AcRoutePathHierModel(th.nn.Module):
    def __init__(self,
                 model_config: ModelConfigDict):
        th.nn.Module.__init__(self)

        self.seg_feat_dim = model_config['custom_model_config']['seg_feat_dim']
        self.path_feat_dim = model_config['custom_model_config']['path_feat_dim']

        self.seg_to_seg_gnn_feat_dim = model_config['custom_model_config']['seg_to_seg_gnn_feat_dim']
        self.seg_to_path_gnn_feat_dim = model_config['custom_model_config']['seg_to_path_gnn_feat_dim']
        self.path_to_super_gnn_feat_dim = model_config['custom_model_config']['path_to_super_gnn_feat_dim']
        self.super_to_super_gnn_feat_dim = model_config['custom_model_config']['super_to_super_gnn_feat_dim']
        
        self.seg_to_seg_gnn_sage_aggregator = model_config['custom_model_config']['seg_to_seg_gnn_sage_aggregator']
        self.seg_to_path_gnn_sage_aggregator = model_config['custom_model_config']['seg_to_path_gnn_sage_aggregator']
        self.path_to_super_gnn_sage_aggregator = model_config['custom_model_config']['path_to_super_gnn_sage_aggregator']
        self.super_to_super_gnn_sage_aggregator = model_config['custom_model_config']['super_to_super_gnn_sage_aggregator']

        self.q_module_hiddens = model_config['custom_model_config']['advantage_module_hiddens']
        self.value_module_hiddens = model_config['custom_model_config']['value_module_hiddens']

        # seg-to-seg module
        assert len(self.seg_to_seg_gnn_feat_dim) > 0
        self.seg_to_seg_module = dgl.nn.Sequential()
        seg_in_feats = self.seg_feat_dim
        for i, n in enumerate(self.seg_to_seg_gnn_feat_dim):
            self.seg_to_seg_module.add_module('sage_{}'.format(i), SAGEConv(in_feats=(seg_in_feats, seg_in_feats), out_feats=n, aggregator_type=self.seg_to_seg_gnn_sage_aggregator, activation=th.nn.ReLU()))
            seg_in_feats = n

        # seg-to-path module
        assert len(self.seg_to_path_gnn_feat_dim) == 1
        path_in_feats = self.path_feat_dim - 1 # no target feature in top/mid level path nodes
        self.seg_to_path_module = dgl.nn.Sequential()
        self.seg_to_path_module.add_module('sage_0', SAGEConv(in_feats=(seg_in_feats, path_in_feats), out_feats=self.seg_to_path_gnn_feat_dim[-1], aggregator_type=self.seg_to_path_gnn_sage_aggregator, activation=th.nn.ReLU()))

        # path-to-super module
        assert len(self.path_to_super_gnn_feat_dim) == 1
        path_in_feats = self.seg_to_path_gnn_feat_dim[-1]
        self.path_to_super_module = dgl.nn.Sequential()
        self.path_to_super_module.add_module('sage_0', SAGEConv(in_feats=(path_in_feats, 0), out_feats=self.path_to_super_gnn_feat_dim[-1], aggregator_type=self.path_to_super_gnn_sage_aggregator, activation=th.nn.ReLU()))

        # super-to-super module
        super_in_feats = self.path_to_super_gnn_feat_dim[-1]
        self.super_to_super_module = dgl.nn.Sequential()
        for i, n, in enumerate(self.super_to_super_gnn_feat_dim):
            self.super_to_super_module.add_module('sage_{}'.format(i), SAGEConv(in_feats=(super_in_feats, super_in_feats), out_feats=n, aggregator_type=self.super_to_super_gnn_sage_aggregator, activation=th.nn.ReLU()))
            super_in_feats = n

        # GNN
        def my_agg_func(tensors, dsttype):
            # tensors: list[th.Tensor]
            h = th.cat(tensors, dim=1)
            return h

        self.hetero_conv = HeteroGraphConv({
            'seg_to_seg':   self.seg_to_seg_module,
            'seg_to_path':  self.seg_to_path_module,
            'path_to_super': self.path_to_super_module,
            'super_to_super': self.super_to_super_module,
        }, aggregate=my_agg_func)



        self._max_batch_size = model_config['custom_model_config']['batch_size']

        self.max_segs_per_net = model_config['custom_model_config']['max_segs'] 
        self.max_paths_per_net = model_config['custom_model_config']['max_paths']
        self.max_supers_per_net = model_config['custom_model_config']['max_supers']

        self.max_nets_per_group = model_config['custom_model_config']['max_nets_per_group']
        self.max_segs_per_group = self.max_nets_per_group * self.max_segs_per_net
        self.max_paths_per_group = self.max_nets_per_group * self.max_paths_per_net
        self.max_supers_per_group = self.max_nets_per_group * self.max_supers_per_net

        self.max_seg_to_seg_edges = self.max_nets_per_group * 12 * self.max_segs_per_net
        self.max_seg_to_path_edges = self.max_nets_per_group * self.max_segs_per_net * self.max_paths_per_net
        self.max_path_to_super_edges = self.max_nets_per_group * self.max_paths_per_net
        self.max_super_to_super_edges =self.max_supers_per_net * self.max_nets_per_group * (self.max_nets_per_group - 1)

        self.seg_to_seg_edges_shift_mask = th.zeros(self._max_batch_size * self.max_seg_to_seg_edges, dtype=th.int32)
        self.seg_to_path_edges_shift_mask = th.zeros(2, self._max_batch_size * self.max_seg_to_path_edges, dtype=th.int32)
        self.path_to_super_edges_shift_mask = th.zeros(2, self._max_batch_size * self.max_path_to_super_edges, dtype=th.int32)
        self.super_to_super_edges_shift_mask = th.zeros(self._max_batch_size * self.max_super_to_super_edges, dtype=th.int32)

        for i in range(self._max_batch_size):
            self.seg_to_seg_edges_shift_mask[i * self.max_seg_to_seg_edges : (i + 1) * self.max_seg_to_seg_edges] = th.full((1, self.max_seg_to_seg_edges), i * self.max_segs_per_group)
            self.seg_to_path_edges_shift_mask[0][i * self.max_seg_to_path_edges : (i + 1) * self.max_seg_to_path_edges] = th.full((1, self.max_seg_to_path_edges), i * self.max_segs_per_group)
            self.seg_to_path_edges_shift_mask[1][i * self.max_seg_to_path_edges : (i + 1) * self.max_seg_to_path_edges] = th.full((1, self.max_seg_to_path_edges), i * self.max_paths_per_group)
            self.path_to_super_edges_shift_mask[0][i * self.max_path_to_super_edges : (i + 1) * self.max_path_to_super_edges] = th.full((1, self.max_path_to_super_edges), i * self.max_paths_per_group)
            self.path_to_super_edges_shift_mask[1][i * self.max_path_to_super_edges : (i + 1) * self.max_path_to_super_edges] = th.full((1, self.max_path_to_super_edges), i * self.max_supers_per_group)
            self.super_to_super_edges_shift_mask[i * self.max_super_to_super_edges : (i + 1) * self.max_super_to_super_edges] = th.full((1, self.max_super_to_super_edges), i * self.max_supers_per_group)

    def build_hetero_graph(self, group_obs: Dict[str, TensorType],
                           device):
        seg_feats = group_obs['seg_feats']
        path_feats = group_obs['path_feats']
        super_feats = group_obs['super_feats']

        seg_to_seg_edges = group_obs['seg_to_seg_edges']
        seg_to_path_edges = group_obs['seg_to_path_edges']
        path_to_super_edges = group_obs['path_to_super_edges']
        super_to_super_edges = group_obs['super_to_super_edges']
        
        seg_to_seg_valid_edges = group_obs['seg_to_seg_valid_edges']
        seg_to_path_valid_edges = group_obs['seg_to_path_valid_edges']
        path_to_super_valid_edges = group_obs['path_to_super_valid_edges']
        super_to_super_valid_edges = group_obs['super_to_super_valid_edges']

        seg_to_seg_edges = th.as_tensor(seg_to_seg_edges, dtype=th.int32)
        seg_to_path_edges = th.as_tensor(seg_to_path_edges, dtype=th.int32)
        path_to_super_edges = th.as_tensor(path_to_super_edges, dtype=th.int32)
        super_to_super_edges = th.as_tensor(super_to_super_edges, dtype=th.int32)

        seg_to_seg_edges = th.flatten(seg_to_seg_edges, start_dim=0, end_dim=1)
        seg_to_path_edges = th.flatten(seg_to_path_edges, start_dim=0, end_dim=1)
        path_to_super_edges = th.flatten(path_to_super_edges, start_dim=0, end_dim=1)
        super_to_super_edges = th.flatten(super_to_super_edges, start_dim=0, end_dim=1)

        seg_to_seg_edges = th.transpose(seg_to_seg_edges, 0, 1)
        seg_to_path_edges = th.transpose(seg_to_path_edges, 0, 1)
        path_to_super_edges = th.transpose(path_to_super_edges, 0, 1)
        super_to_super_edges = th.transpose(super_to_super_edges, 0, 1)

        seg_to_seg_edges_shift_mask = self.seg_to_seg_edges_shift_mask[:seg_to_seg_edges.shape[1]].to(device)
        seg_to_path_edges_shift_mask = self.seg_to_path_edges_shift_mask[:, :seg_to_path_edges.shape[1]].to(device)
        path_to_super_edges_shift_mask = self.path_to_super_edges_shift_mask[:, :path_to_super_edges.shape[1]].to(device)
        super_to_super_edges_shift_mask = self.super_to_super_edges_shift_mask[:super_to_super_edges.shape[1]].to(device)

        seg_to_seg_edges = th.add(seg_to_seg_edges, seg_to_seg_edges_shift_mask)
        seg_to_path_edges = th.add(seg_to_path_edges, seg_to_path_edges_shift_mask)
        path_to_super_edges = th.add(path_to_super_edges, path_to_super_edges_shift_mask)
        super_to_super_edges = th.add(super_to_super_edges, super_to_super_edges_shift_mask)

        seg_to_seg_valid_edges = th.as_tensor(seg_to_seg_valid_edges, dtype=bool)
        seg_to_path_valid_edges = th.as_tensor(seg_to_path_valid_edges, dtype=bool)
        path_to_super_valid_edges = th.as_tensor(path_to_super_valid_edges, dtype=bool)
        super_to_super_valid_edges = th.as_tensor(super_to_super_valid_edges, dtype=bool)

        seg_to_seg_valid_edges = th.flatten(seg_to_seg_valid_edges)
        seg_to_path_valid_edges = th.flatten(seg_to_path_valid_edges)
        path_to_super_valid_edges = th.flatten(path_to_super_valid_edges)
        super_to_super_valid_edges = th.flatten(super_to_super_valid_edges)

        seg_to_seg_edges = seg_to_seg_edges[:, seg_to_seg_valid_edges]
        seg_to_path_edges = seg_to_path_edges[:, seg_to_path_valid_edges]
        path_to_super_edges = path_to_super_edges[:, path_to_super_valid_edges]
        super_to_super_edges = super_to_super_edges[:, super_to_super_valid_edges]

        data_dict = {
            ('seg', 'seg_to_seg', 'seg'): (seg_to_seg_edges[0], seg_to_seg_edges[1]),
            ('seg', 'seg_to_path', 'path'): (seg_to_path_edges[0], seg_to_path_edges[1]),
            ('path', 'path_to_super', 'super'): (path_to_super_edges[0], path_to_super_edges[1]),
            ('super', 'super_to_super', 'super'): (super_to_super_edges[0], super_to_super_edges[1]),
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
        bhg = dgl.add_self_loop(bhg, etype='super_to_super')

        bhg.nodes['seg'].data['feat'] = seg_feats
        bhg.nodes['path'].data['feat'] = path_feats
        bhg.nodes['super'].data['feat'] = super_feats

        return bhg


################################################
#                Top Level                     #
################################################
class AcRoutePPOPathTopLevelModel(TorchModelV2, AcRoutePathHierModel):
    def __init__(self,
                 obs_space: gym.spaces.Space,
                 action_space: gym.spaces.Space,
                 num_outputs: int,
                 model_config: ModelConfigDict,
                 name: str,
                 **customized_model_kwargs):
        TorchModelV2.__init__(self, obs_space, action_space, num_outputs, model_config, name)
        AcRoutePathHierModel.__init__(self, model_config)


        val_in_feats = self.super_to_super_gnn_feat_dim[-1]
        adv_in_feats = 2 * val_in_feats

        # Q module (model to action)
        self.q_module = th.nn.Sequential()
        for i, n in enumerate(self.q_module_hiddens):
            self.q_module.add_module('fc_{}'.format(i), th.nn.Linear(in_features=adv_in_feats, out_features=n))
            self.q_module.add_module('relu_{}'.format(i), th.nn.ReLU())
            adv_in_feats = n
        self.q_module.add_module('fc_final', th.nn.Linear(in_features=adv_in_feats, out_features=1))

        # value module (model to state value)
        self.value_module = th.nn.Sequential()
        for i, n in enumerate(self.value_module_hiddens):
            self.value_module.add_module('fc_{}'.format(i), th.nn.Linear(in_features=val_in_feats, out_features=n))
            self.value_module.add_module('relu_{}'.format(i), th.nn.ReLU())
            val_in_feats = n
        self.value_module.add_module('fc_final', th.nn.Linear(in_features=val_in_feats, out_features=1))


    @override(TorchModelV2)
    def forward(self,
                input_dict: Dict[str, TensorType],
                state: List[TensorType],
                seq_lens: TensorType):

        obs = input_dict['obs']

        group_obs = obs['group_obs']
        action_mask = obs['action_mask']

        self._last_batch_size = action_mask.shape[0]
        device = action_mask.device

        bhg = self.build_hetero_graph(group_obs, device)


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
                'super': h3['super'],
            }))
        h = h4['super']


        # avg per net
        h = h.view(self._last_batch_size, self.max_nets_per_group, self.max_supers_per_net, -1)
        super_valid_nodes = th.as_tensor(group_obs['super_valid_nodes'], dtype=bool)
        super_valid_nodes = super_valid_nodes.view(self._last_batch_size, self.max_nets_per_group, self.max_supers_per_net)

        vn = th.zeros(h.shape, dtype=bool).to(device)
        vn[super_valid_nodes] = True
        avg_net = (th.sum(h * vn.float(), dim=-2) / th.sum(vn, dim=-2))
        avg_net[avg_net != avg_net] = 0


        # avg all group
        vn = th.zeros(avg_net.shape, dtype=bool).to(device)

        vn_trim = (action_mask == 0)[:, :-1]
        vn[vn_trim] = True
        avg_all = (th.sum(avg_net * vn.float(), dim=-2) / th.sum(vn, dim=-2))
        avg_all[avg_all != avg_all] = 0
        avg_all = avg_all.view(avg_all.shape[0], 1, -1)

        h = th.cat((avg_net, avg_all), dim=1)
        avg_all_repeat = avg_all.repeat(1, h.shape[1], 1)

        h = th.cat((h, avg_all_repeat), dim=-1)

        h = self.q_module(h)
        h = th.squeeze(h, -1)

        action_mask = (action_mask == 1)
        h[action_mask] = float('-inf')

        y = th.squeeze(avg_all, 1)
        self._value_out = th.flatten(self.value_module(y))

        return h, state


    @override(TorchModelV2)
    def value_function(self) -> TensorType:
        return self._value_out



################################################
#                Mid Level                     #
################################################
class AcRoutePPOPathMidLevelModel(TorchModelV2, AcRoutePathHierModel):
    def __init__(self,
                 obs_space: gym.spaces.Space,
                 action_space: gym.spaces.Space,
                 num_outputs: int,
                 model_config: ModelConfigDict,
                 name: str,
                 **customized_model_kwargs):
        TorchModelV2.__init__(self, obs_space, action_space, num_outputs, model_config, name)
        AcRoutePathHierModel.__init__(self, model_config)


        val_in_feats = 2 * self.super_to_super_gnn_feat_dim[-1]
        adv_in_feats = val_in_feats

        # Q module (model to action)
        self.q_module = th.nn.Sequential()
        for i, n in enumerate(self.q_module_hiddens):
            self.q_module.add_module('fc_{}'.format(i), th.nn.Linear(in_features=adv_in_feats, out_features=n))
            self.q_module.add_module('relu_{}'.format(i), th.nn.ReLU())
            adv_in_feats = n
        self.q_module.add_module('fc_final', th.nn.Linear(in_features=adv_in_feats, out_features=2))

        # value module (model to state value)
        self.value_module = th.nn.Sequential()
        for i, n in enumerate(self.value_module_hiddens):
            self.value_module.add_module('fc_{}'.format(i), th.nn.Linear(in_features=val_in_feats, out_features=n))
            self.value_module.add_module('relu_{}'.format(i), th.nn.ReLU())
            val_in_feats = n
        self.value_module.add_module('fc_final', th.nn.Linear(in_features=val_in_feats, out_features=1))
        

    @override(TorchModelV2)
    def forward(self,
                input_dict: Dict[str, TensorType],
                state: List[TensorType],
                seq_lens: TensorType):

        obs = input_dict['obs']

        group_obs = obs['group_obs']
        valid_nets = obs['valid_nets']
        top_level_action = obs['top_level_action']

        self._last_batch_size = top_level_action.shape[0]
        device = top_level_action.device

        bhg = self.build_hetero_graph(group_obs, device)

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
                'super': h3['super'],
            }))

        h = h4['super']

        # avg net
        valid_nets = th.as_tensor(valid_nets, dtype=bool)

        h = h.view(self._last_batch_size, self.max_nets_per_group, self.max_supers_per_net, -1)
        vn = th.zeros(h.shape, dtype=bool).to(device)
        vn[valid_nets] = True

        h_group = (th.sum(h * vn.float(), dim=1) / th.sum(vn, dim=1))
        h_group[h_group != h_group] = 0

        stride = th.arange(self._last_batch_size)
        top_level_action = th.squeeze(th.as_tensor(top_level_action, dtype=th.long), -1)

        super_valid_nodes = th.as_tensor(group_obs['super_valid_nodes'], dtype=bool)
        super_valid_nodes = super_valid_nodes.view(self._last_batch_size, self.max_nets_per_group, self.max_supers_per_net, -1)

        h_target = h[stride, top_level_action]
        vp_target = super_valid_nodes[stride, top_level_action]
        vp_q = vp_target.repeat(1, 1, 2)

        h = th.cat((h_target, h_group), dim=-1)

        q_out = self.q_module(h)
        q_out = q_out * vp_q.float()

        # q_out[:, :, 0] = th.tanh(q_out[:, :, 0])
        q_out = th.cat((q_out[:, :, 0], q_out[:, :, 1]), dim=-1)
       
        vp_y = vp_target.repeat(1, 1, h.shape[-1])
        y = th.sum(h * vp_y.float(), dim=1) / th.sum(vp_y, dim=1)
        self._value_out = th.flatten(self.value_module(y))

        return q_out, state

    
    @override(TorchModelV2)
    def value_function(self) -> TensorType:
        return self._value_out



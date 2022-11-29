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
from ray.rllib.agents.dqn.dqn_torch_model import DQNTorchModel
from ray.rllib.utils.annotations import override
from ray.rllib.utils.typing import ModelConfigDict, TensorType

import dgl
from dgl.nn import SAGEConv
from dgl.nn.pytorch import HeteroGraphConv

class AcRouteDQNPathModel(DQNTorchModel):
    def __init__(self,
                 obs_space: gym.spaces.Space,
                 action_space: gym.spaces.Space,
                 num_outputs: int,
                 model_config: ModelConfigDict,
                 name: str,
                 **customized_model_kwargs):

        # TorchModelV2.__init__(self, obs_space, action_space, num_outputs, model_config, name)
        # th.nn.Module.__init__(self)
        super().__init__(obs_space, action_space, num_outputs, model_config, name, **customized_model_kwargs)

        # print(pretty_print(customized_model_kwargs))

        self.max_segs = model_config['custom_model_config']['max_segs']
        self.seg_feat_dim = model_config['custom_model_config']['seg_feat_dim']
        self.seg_to_seg_gnn_feat_dim = model_config['custom_model_config']['seg_to_seg_gnn_feat_dim']

        self.max_paths = model_config['custom_model_config']['max_paths']
        self.path_feat_dim = model_config['custom_model_config']['path_feat_dim']
        self.path_to_seg_gnn_feat_dim = model_config['custom_model_config']['path_to_seg_gnn_feat_dim']

        self.advantage_module_hiddens = model_config['custom_model_config']['advantage_module_hiddens']
        self.value_module_hiddens = model_config['custom_model_config']['value_module_hiddens']


        self.node_feat_dim = self.seg_feat_dim + self.path_feat_dim

        # seg-to-seg module
        assert len(self.seg_to_seg_gnn_feat_dim) > 0
        self.seg_to_seg_module = dgl.nn.Sequential()
        # seg_in_feats = self.node_feat_dim
        seg_in_feats = self.seg_feat_dim
        for i, n in enumerate(self.seg_to_seg_gnn_feat_dim):
            self.seg_to_seg_module.add_module('sage_{}'.format(i), SAGEConv(in_feats=(seg_in_feats, seg_in_feats), out_feats=n, aggregator_type='mean', activation=th.nn.ReLU()))
            seg_in_feats = n

        # path-to-seg module
        assert len(self.path_to_seg_gnn_feat_dim) > 0
        self.path_to_seg_module = dgl.nn.Sequential()
        # path_in_feats = self.node_feat_dim
        path_in_feats = self.path_feat_dim
        for i, n in enumerate(self.path_to_seg_gnn_feat_dim):
           self.path_to_seg_module.add_module('sage_{}'.format(i), SAGEConv(in_feats=(path_in_feats, seg_in_feats), out_feats=n, aggregator_type='mean', activation=th.nn.ReLU()))
           path_in_feats = n


        def my_agg_func(tensors, dsttype):
            # tensors: list[th.Tensor]
            h = th.cat(tensors, dim=1)
            return h
            
        self.hetero_conv = HeteroGraphConv({
            'seg_to_seg': self.seg_to_seg_module,
            'path_to_seg': self.path_to_seg_module,
            }, aggregate=my_agg_func)


        val_in_feats = self.seg_to_seg_gnn_feat_dim[-1] + self.path_to_seg_gnn_feat_dim[-1]
        adv_in_feats = 2 * val_in_feats

        # advantage module
        self.advantage_module = th.nn.Sequential()
        for i, n in enumerate(self.advantage_module_hiddens):
            self.advantage_module.add_module('fc_{}'.format(i), th.nn.Linear(in_features=adv_in_feats, out_features=n))
            self.advantage_module.add_module('relu_{}'.format(i), th.nn.ReLU())
            adv_in_feats = n
        self.advantage_module.add_module('fc_final', th.nn.Linear(in_features=adv_in_feats, out_features=1))
           
        # value module
        self.value_module = th.nn.Sequential()
        for i, n in enumerate(self.value_module_hiddens):
            self.value_module.add_module('fc_{}'.format(i), th.nn.Linear(in_features=val_in_feats, out_features=n))
            self.value_module.add_module('relu_{}'.format(i), th.nn.ReLU())
            val_in_feats = n
        self.value_module.add_module('fc_final', th.nn.Linear(in_features=val_in_feats, out_features=1))

        self._last_batch_size = None
        self._max_batch_size = model_config['custom_model_config']['batch_size']
        
        self.max_seg_edges = 12 * self.max_segs
        self.max_path_edges = self.max_paths * self.max_segs

        self.seg_edges_shift_mask = th.zeros(self._max_batch_size * self.max_seg_edges, dtype=th.int32)
        for i in range(self._max_batch_size):
            self.seg_edges_shift_mask[i * self.max_seg_edges : (i + 1) * self.max_seg_edges] = th.full((1, self.max_seg_edges), i * self.max_segs)
        
        self.path_edges_shift_mask = th.zeros(2, self._max_batch_size * self.max_path_edges, dtype=th.int32)
        for i in range(self._max_batch_size):
            self.path_edges_shift_mask[0][i * self.max_path_edges : (i + 1) * self.max_path_edges] = th.full((1, self.max_path_edges), i * self.max_paths)
            self.path_edges_shift_mask[1][i * self.max_path_edges : (i + 1) * self.max_path_edges] = th.full((1, self.max_path_edges), i * self.max_segs)

        # to train or not
        # if 'fixed_weights' in model_config['custom_model_config'] and model_config['custom_model_config']['fixed_weights']:
            # for _, param in self._parameters.items():
                # param.requires_grad = False


    @override(DQNTorchModel)
    def forward(self,
               input_dict: Dict[str, TensorType],
               state: List[TensorType],
               seq_lens: TensorType):

        obs = input_dict['obs']
	
        action_mask = obs['action_mask']
       	seg_feats = obs['seg_feats']
        seg_edges = obs['seg_edges']
        seg_valid_nodes = obs['seg_valid_nodes']
        seg_valid_edges = obs['seg_valid_edges']
       	path_feats = obs['path_feats']
        path_edges = obs['path_edges']
        path_valid_nodes = obs['path_valid_nodes']
        path_valid_edges = obs['path_valid_edges']
        
        self._last_batch_size = seg_feats.shape[0]

        device = seg_feats.device
        
        seg_edges = th.as_tensor(seg_edges, dtype=th.int32)
        path_edges = th.as_tensor(path_edges, dtype=th.int32)

        seg_edges = th.flatten(seg_edges, start_dim=0, end_dim=1)
        path_edges = th.flatten(path_edges, start_dim=0, end_dim=1)
        
        seg_edges = th.transpose(seg_edges, 0, 1)
        path_edges = th.transpose(path_edges, 0, 1)

        seg_edges_shift_mask = self.seg_edges_shift_mask[:seg_edges.shape[1]].to(device)
        path_edges_shift_mask = self.path_edges_shift_mask[:, :path_edges.shape[1]].to(device)

        seg_edges = th.add(seg_edges, seg_edges_shift_mask)
        path_edges = th.add(path_edges, path_edges_shift_mask)

        seg_valid_edges = th.as_tensor(seg_valid_edges, dtype=bool)
        path_valid_edges = th.as_tensor(path_valid_edges, dtype=bool)

        seg_valid_edges = th.flatten(seg_valid_edges)
        path_valid_edges = th.flatten(path_valid_edges)

        seg_edges = seg_edges[:, seg_valid_edges]
        path_edges = path_edges[:, path_valid_edges]


        data_dict = {
            ('seg', 'seg_to_seg', 'seg'): (seg_edges[0], seg_edges[1]),
            ('path', 'path_to_seg', 'seg'): (path_edges[0], path_edges[1]),
        }
        num_nodes_dict = {
            'seg': self._last_batch_size * seg_feats.shape[1],
            'path': self._last_batch_size * path_feats.shape[1],
        }

        seg_feats = th.flatten(seg_feats, start_dim=0, end_dim=1)
        path_feats = th.flatten(path_feats, start_dim=0, end_dim=1)

        bhg = dgl.heterograph(data_dict, num_nodes_dict, device=device)
        # bhg = dgl.to_simple(bhg)
        # bhg = bhg.to(device)
        # bhg = dgl.remove_self_loop(bhg, 'seg_to_seg')

        # seg_feats = th.cat((seg_feats, th.zeros(seg_feats.shape[0], self.path_feat_dim).to(device)), dim=-1)
        # path_feats = th.cat((th.zeros(path_feats.shape[0], self.seg_feat_dim).to(device), path_feats), dim=-1)
        bhg.nodes['seg'].data['feat'] = seg_feats
        bhg.nodes['path'].data['feat'] = path_feats

        h = self.hetero_conv(bhg, (
            {
                'seg': bhg.nodes['seg'].data['feat'],
            },
            {
                'seg': bhg.nodes['seg'].data['feat'],
            }))
        h2 = self.hetero_conv(bhg, (
            {
                'path': bhg.nodes['path'].data['feat'],
            },
            {
                'seg': h['seg'],
            }))

        h = th.cat((h['seg'], h2['seg']), dim=1)
        h = h.view(self._last_batch_size, self.max_segs, -1)


        # avg
        seg_valid_nodes = th.as_tensor(seg_valid_nodes, dtype=bool)
        vn = th.zeros(h.shape, dtype=bool).to(device)
        vn[seg_valid_nodes] = True
        avg_all = (th.sum(h * vn.float(), dim=1) / th.sum(vn, dim=1))
        avg_all[avg_all != avg_all] = 0 # nan -> 0
        avg_all = avg_all.view(avg_all.shape[0], 1, -1)
        
        return (h, avg_all, action_mask), state

    @override(DQNTorchModel)
    def get_q_value_distributions(self, model_out):
        '''Returns distributional values for Q(s, a) given a state embedding.
        Override this in your custom model to customize the Q output head.
        Args:
            model_out (Tuple): Embedding from the model layers, action_mask
        Returns:
            (action_scores, logits, dist) if num_atoms == 1, otherwise
            (action_scores, z, support_logits_per_action, logits, dist)
        '''
        h = model_out[0]
        avg_all = model_out[1]
        
        h = th.cat((h, avg_all), dim=1)

        avg_all_repeat = avg_all.repeat(1, h.shape[1], 1)
        h = th.cat((h, avg_all_repeat), dim=-1)
        action_scores = self.advantage_module(h)
        action_scores = th.squeeze(action_scores, -1)

        action_mask = (model_out[3] == 1)
        assert action_mask.shape == action_scores.shape

        action_scores[action_mask] = float('-inf')
        # print('action_mask', action_mask)
        # print('action_scores', action_scores)

        if self.num_atoms > 1: # FIXME?? might need adjustment for action mask
            # Distributional Q-learning uses a discrete support z
            # to represent the action value distribution
            z = th.range(
                0.0, self.num_atoms - 1,
                dtype=th.float32).to(action_scores.device)
            z = self.v_min + \
                z * (self.v_max - self.v_min) / float(self.num_atoms - 1)

            support_logits_per_action = th.reshape(action_scores, shape=(-1, self.action_space.n, self.num_atoms))
            support_prob_per_action = nn.functional.softmax(support_logits_per_action, dim=-1)
            action_scores = th.sum(z * support_prob_per_action, dim=-1)
            logits = support_logits_per_action
            probs = support_prob_per_action
            return action_scores, z, support_logits_per_action, logits, probs
        else:
            logits = th.unsqueeze(th.ones_like(action_scores), -1)
            return action_scores, logits, logits

    @override(DQNTorchModel)
    def get_state_value(self, model_out):
        avg = th.squeeze(model_out[1], 1);
        return self.value_module(avg)

    @override(DQNTorchModel)
    def value_function(self) -> TensorType:
        return th.from_numpy(np.zeros(shape=(self._last_batch_size, )))

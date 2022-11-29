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

class AcRouteDQNModel(DQNTorchModel):
    def __init__(self,
                 obs_space: gym.spaces.Space,
                 action_space: gym.spaces.Space,
                 num_outputs: int,
                 model_config: ModelConfigDict,
                 name: str,
                 **customized_model_kwargs):

        # TorchModelV2.__init__(self, obs_space, action_space, num_outputs, model_config, name)
        # th.nn.Module.__init__(self)
        super(AcRouteDQNModel, self).__init__(obs_space, action_space, num_outputs, model_config, name, **customized_model_kwargs)
        # print(self)

        # print(pretty_print(customized_model_kwargs))

        self.max_segs = model_config['custom_model_config']['max_segs']
        self.input_dim = model_config['custom_model_config']['seg_feat_dim']
        self.gnn_feat_dim = model_config['custom_model_config']['seg_to_seg_gnn_feat_dim']
        self.advantage_module_hiddens = model_config['custom_model_config']['advantage_module_hiddens']
        self.value_module_hiddens = model_config['custom_model_config']['value_module_hiddens']

        # custom embedding module
        assert len(self.gnn_feat_dim) > 0
        self.embed_gnn_module = dgl.nn.Sequential()
        in_feats = self.input_dim
        for i, n in enumerate(self.gnn_feat_dim):
            self.embed_gnn_module.add_module('sage_{}'.format(i), SAGEConv(in_feats=in_feats, out_feats=n, aggregator_type='mean', activation=th.nn.ReLU()))
            in_feats = n


        embed_dim = in_feats
        # custom advantage module
        self.advantage_module = th.nn.Sequential()
        in_feats = embed_dim * 2
        for i, n in enumerate(self.advantage_module_hiddens):
            self.advantage_module.add_module('fc_{}'.format(i), th.nn.Linear(in_features=in_feats, out_features=n))
            self.advantage_module.add_module('relu_{}'.format(i), th.nn.ReLU())
            in_feats = n
        self.advantage_module.add_module('fc_final', th.nn.Linear(in_features=in_feats, out_features=1))
           
        # custom value module
        self.value_module = th.nn.Sequential()
        in_feats = embed_dim
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


    @override(DQNTorchModel)
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

        # print('f h shape', h.shape)
        return (h, avg, action_mask), state

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
        avg = model_out[1]
        h = th.cat((h, avg), dim=1)
        avg_repeat = avg.repeat(1, h.shape[1], 1)
        h = th.cat((h, avg_repeat), dim=-1)
        action_scores = self.advantage_module(h)
        action_scores = th.squeeze(action_scores, -1)

        action_mask = (model_out[2] == 1)
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

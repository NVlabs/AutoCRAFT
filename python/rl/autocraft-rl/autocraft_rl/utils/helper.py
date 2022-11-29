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
import numpy as np
import torch as th

import autocraft as ac
import pickle

##########################################
#           Single net                   #
##########################################
def get_features(cfg, 
                 ac_rl: ac.rl_utils,
                 cir: ac.cir_db,
                 net: ac.net,
                 route_dr_mgr: ac.route_dr_mgr, 
                 route_dr_ps: ac.route_dr_ps, 
                 drc_mgr: ac.drc_mgr):

    feats = ac.VectorVectorFloat()
    edges = ac.VectorVectorInt()
    masks = ac.VectorFloat()
    ac_rl.gen_feats_edges_masks(cir, net, route_dr_ps, drc_mgr, feats, edges, masks,
                                cfg.rl_wl_norm_constant,
                                cfg.rl_his_norm_constant)
    feats = np.array(feats, dtype=np.float32)
    edges = np.array(edges, dtype=np.float32)
    masks = np.array(masks, dtype=np.float32)

    assert len(feats) == len(masks)

    # total_drv = feats[:, -1].sum()

    num_nodes = len(feats)
    num_max_nodes = cfg.rl_max_segs
    num_edges = len(edges)
    num_max_edges = 12 * cfg.rl_max_segs # 12
    

    assert num_max_nodes >= num_nodes, '{} {} {}'.format(num_max_nodes, num_nodes, net.name())
    assert num_max_edges >= num_edges, '{} {} {}'.format(num_max_edges, num_edges, net.name())

    # masks = np.pad(masks, (0, num_max_nodes - num_nodes + 1), 'constant', constant_values=(1., 1.))
    # masks[-1] = 0. # extra dim for 'done' action
    masks_pad = np.ones(num_max_nodes + 1, dtype=np.float32)
    masks_pad[:masks.shape[0]] = masks
    masks_pad[-1] = 0.

    # feats = np.pad(feats, ((0, num_max_nodes - num_nodes), (0, 0)), 'constant', constant_values=((0., 0.), (0., 0.)))
    # edges = np.pad(edges, ((0, num_max_edges - num_edges), (0, 0)), 'constant', constant_values=((num_max_nodes, num_max_nodes)))
    feats_pad = np.full((num_max_nodes, feats.shape[1]), np.zeros(feats.shape[1]), dtype=np.float32)
    edges_pad = np.full((num_max_edges, 2), [num_max_nodes, num_max_nodes], dtype=np.float32)
    feats_pad[:num_nodes] = feats
    edges_pad[:num_edges] = edges

    valid_nodes = np.zeros(num_max_nodes, dtype=bool)
    valid_edges = np.zeros(num_max_edges, dtype=bool)
    valid_nodes[:num_nodes] = True
    valid_edges[:num_edges] = True

    return feats_pad, edges_pad, valid_nodes, valid_edges, masks_pad

def get_features_with_target(cfg,
                             ac_rl: ac.rl_utils,
                             cir: ac.cir_db,
                             net: ac.net,
                             route_dr_mgr: ac.route_dr_mgr, 
                             route_dr_ps: ac.route_dr_ps, 
                             drc_mgr: ac.drc_mgr,
                             target_wl,
                             target_via):
    feats, edges, valid_nodes, valid_edges, masks = get_features(cfg,
                                                                 ac_rl,
                                                                 cir,
                                                                 net,
                                                                 route_dr_mgr,
                                                                 route_dr_ps,
                                                                 drc_mgr)
    n = np.full((len(feats), 2), [target_wl / net.bbox().hpwl(), target_via / cfg.rl_via_norm_constant], dtype=np.float32)
    feats = np.concatenate((feats, n), axis=1, dtype=np.float32)

    return feats, edges, valid_nodes, valid_edges , masks



def get_features_with_layer_target(cfg,
                                   ac_rl: ac.rl_utils,
                                   cir: ac.cir_db,
                                   net: ac.net,
                                   route_dr_mgr: ac.route_dr_mgr, 
                                   route_dr_ps: ac.route_dr_ps, 
                                   drc_mgr: ac.drc_mgr,
                                   layer_wl: ac.VectorFloat,
                                   layer_via: ac.VectorFloat,
                                   target):

    feats, edges, valid_nodes, valid_edges, masks = get_features(cfg,
                                                                 ac_rl,
                                                                 cir,
                                                                 net,
                                                                 route_dr_mgr,
                                                                 route_dr_ps,
                                                                 drc_mgr)
    n = np.zeros((len(feats), 2), dtype=np.float32)
    for i in range(np.sum(valid_nodes)):
        feat = feats[i]
        layer_idx = int(feat[5]) * cir.num_layers()
        idx = cir.layer(layer_idx).self_idx()

        if feat[6] == 1: # is via
            via_t = target['via_v{:d}'.format(idx + 1)]
            via_c = layer_via[idx]
            n[i][0] = 0
            n[i][1] = (via_t - via_c) / cfg.rl_via_norm_constant

        else: # not via
            wl_t = target['wl_m{:d}'.format(idx + 1)]
            wl_c = layer_wl[idx]
            n[i][0] = (wl_t - wl_c) / net.bbox().hpwl()
            n[i][1] = 0

    feats = np.concatenate((feats, n), axis=1, dtype=np.float32)

    return feats, edges, valid_nodes, valid_edges, masks

    
def get_features_with_path(cfg,
                           ac_rl: ac.rl_utils,
                           cir: ac.cir_db,
                           net: ac.net,
                           route_dr_mgr: ac.route_dr_mgr,
                           route_dr_ps: ac.route_dr_ps,
                           drc_mgr: ac.drc_mgr):

    seg_feats = ac.VectorVectorFloat()
    seg_edges = ac.VectorVectorInt()
    seg_masks = ac.VectorFloat()
    path_feats = ac.VectorVectorFloat()
    path_edges = ac.VectorVectorInt()

    ac_rl.gen_feats_edges_masks_with_path(cir, net, route_dr_ps, drc_mgr,
                                          seg_feats, path_feats,
                                          seg_edges, path_edges,
                                          seg_masks,
                                          cfg.rl_path_segs_norm_constant,
                                          cfg.rl_wl_norm_constant,
                                          cfg.rl_via_norm_constant,
                                          cfg.rl_res_norm_constant,
                                          cfg.rl_his_norm_constant)

    seg_feats = np.array(seg_feats, dtype=np.float32)
    seg_edges = np.array(seg_edges, dtype=np.float32)
    seg_masks = np.array(seg_masks, dtype=np.float32)
    path_feats = np.array(path_feats, dtype=np.float32)
    path_edges = np.array(path_edges, dtype=np.float32)


    num_seg_nodes = len(seg_feats)
    num_seg_edges = len(seg_edges)
    num_path_nodes = len(path_feats)
    num_path_edges = len(path_edges)
    max_seg_nodes = cfg.rl_max_segs
    max_seg_edges = 12 * cfg.rl_max_segs
    max_path_nodes = cfg.rl_max_paths
    max_path_edges = max_path_nodes * max_seg_nodes

    # assert max_seg_nodes >= num_seg_nodes, '{} {} {}'.format(max_seg_nodes, num_seg_nodes, net.name())
    # assert max_seg_edges >= num_seg_edges, '{} {} {}'.format(max_seg_edges, num_seg_edges, net.name())
    # assert max_path_nodes >= num_path_nodes, '{} {} {}'.format(max_path_nodes, num_path_nodes, net.name())
    # assert max_path_edges >= num_path_edges, '{} {} {}'.format(max_path_edges, num_path_edges, net.name())
    # assert len(seg_edges.shape) == 2
    # assert len(path_edges.shape) == 2

    seg_masks_pad = np.ones(max_seg_nodes + 1, dtype=np.float32)
    seg_masks_pad[:seg_masks.shape[0]] = seg_masks
    seg_masks_pad[-1] = 0.

    seg_feats_pad  = np.zeros((max_seg_nodes, seg_feats.shape[1]), dtype=np.float32)
    path_feats_pad = np.zeros((max_path_nodes, path_feats.shape[1]), dtype=np.float32)
    seg_edges_pad  = np.full((max_seg_edges, 2), [max_seg_nodes, max_seg_nodes], dtype=np.float32)
    path_edges_pad = np.full((max_path_edges, 2), [max_path_nodes, max_seg_nodes], dtype=np.float32)

    seg_feats_pad[:num_seg_nodes] = seg_feats
    seg_edges_pad[:num_seg_edges] = seg_edges
    path_feats_pad[:num_path_nodes] = path_feats
    path_edges_pad[:num_path_edges] = path_edges


    seg_valid_nodes = np.zeros(max_seg_nodes, dtype=bool)
    seg_valid_edges = np.zeros(max_seg_edges, dtype=bool)
    seg_valid_nodes[:num_seg_nodes] = True
    seg_valid_edges[:num_seg_edges] = True

    path_valid_nodes = np.zeros(max_path_nodes, dtype=bool)
    path_valid_edges = np.zeros(max_path_edges, dtype=bool)
    path_valid_nodes[:num_path_nodes] = True
    path_valid_edges[:num_path_edges] = True

    return seg_feats_pad, seg_edges_pad, seg_valid_nodes, seg_valid_edges, seg_masks_pad, \
           path_feats_pad, path_edges_pad, path_valid_nodes, path_valid_edges

def get_features_with_path_target(cfg,
                                  ac_rl: ac.rl_utils,
                                  cir: ac.cir_db,
                                  net: ac.net,
                                  route_dr_mgr: ac.route_dr_mgr,
                                  route_dr_ps: ac.route_dr_ps,
                                  drc_mgr: ac.drc_mgr,
                                  path_res: ac.VectorFloat,
                                  target):

    seg_feats, seg_edges, seg_valid_nodes, seg_valid_edges, seg_masks, \
    path_feats, path_edges, path_valid_nodes, path_valid_edges = get_features_with_path(cfg,
                                                                                        ac_rl,
                                                                                        cir,
                                                                                        net,
                                                                                        route_dr_mgr,
                                                                                        route_dr_ps,
                                                                                        drc_mgr)
    n = np.zeros((len(path_feats), 1), dtype=np.float32)

    path_res_t = target['path_res']
    for i in range(np.sum(path_valid_nodes)):
        path_feat = path_feats[i]
        
        res_t = path_res_t[i]
        res_c = path_res[i]

        n[i][0] = np.float32((res_t - res_c) / cfg.rl_res_norm_constant)
       
    path_feats = np.concatenate((path_feats, n), axis=1, dtype=np.float32)

    return seg_feats, seg_edges, seg_valid_nodes, seg_valid_edges, seg_masks, \
           path_feats, path_edges, path_valid_nodes, path_valid_edges, 


# def get_image_features(cfg,
                      # ac_rl: ac.rl_utils,
                      # cir: ac.cir_db,
                      # net: ac.net,
                      # route_dr_mgr: ac.route_dr_mgr,
                      # route_dr_ps: ac.route_dr_ps):

    # image = ac.VectorVectorVectorVectorFloat()
    # ac_rl.get_surrounding_image(cir, net, route_dr_ps,
                                # cfg.rl_image_x_size, cfg.rl_image_y_size,
                                # cfg.rl_image_x_step, cfg.rl_image_y_step,
                                # cfg.rl_image_bbox_expand_fc, image)

    # image = np.array(image, dtype=np.float32)
    # print(net.name(), image.shape)
    # ac_rl.to_vis_gds(cir, net, 'test.gds')
    # with open('image.pkl', 'wb') as f:
        # pickle.dump(image, f, protocol=pickle.HIGHEST_PROTOCOL)
    # return image



##########################################
#           Matching group               #
##########################################
def get_super_feats_edges(cfg,
                          ac_rl,
                          cir,
                          net):

    super_feats = ac.VectorVectorFloat()
    path_to_super_edges = ac.VectorVectorInt()

    ac_rl.gen_super_path_feats_edges(cir, net, super_feats, path_to_super_edges)

    super_feats = np.array(super_feats, dtype=np.float32)
    path_to_super_edges = np.array(path_to_super_edges, dtype=np.float32)

    num_super_nodes = len(super_feats)
    num_path_to_super_edges = len(path_to_super_edges)
    max_path_nodes = cfg.rl_max_paths
    max_super_nodes = cfg.rl_max_path_cstrs
    max_path_to_super_edges = cfg.rl_max_paths

    super_feats_pad = np.zeros((max_super_nodes, 0), dtype=np.float32)
    path_to_super_edges_pad = np.full((max_path_to_super_edges, 2), [max_path_nodes, max_super_nodes], dtype=np.float32)

    super_feats_pad[:num_super_nodes] = super_feats
    path_to_super_edges_pad[:num_path_to_super_edges] = path_to_super_edges

    super_valid_nodes = np.zeros(max_super_nodes, dtype=bool)
    super_valid_nodes[:num_super_nodes] = True

    path_to_super_valid_edges = np.zeros(max_path_to_super_edges, dtype=bool)
    path_to_super_valid_edges[:num_path_to_super_edges] = True

    return super_feats_pad, path_to_super_edges_pad, super_valid_nodes, path_to_super_valid_edges, num_super_nodes


def get_match_group_features_hier(cfg,
                                  ac_rl: ac.rl_utils,
                                  cir: ac.cir_db,
                                  match_group: list,
                                  route_dr_mgr: ac.route_dr_mgr,
                                  route_dr_ps: ac.route_dr_ps,
                                  drc_mgr: ac.drc_mgr):

    max_segs_per_net   = cfg.rl_max_segs
    max_paths_per_net  = cfg.rl_max_paths
    max_supers_per_net = cfg.rl_max_path_cstrs # super path nodes (for intra net path matching handling)
    
    max_nets_per_group   = cfg.rl_max_nets_per_group
    max_segs_per_group   = max_nets_per_group * max_segs_per_net
    max_paths_per_group  = max_nets_per_group * max_paths_per_net
    max_supers_per_group = max_nets_per_group * max_supers_per_net

    max_seg_to_seg_edges_per_net    = 12 * max_segs_per_net
    max_seg_to_path_edges_per_net   = max_segs_per_net * max_paths_per_net
    max_path_to_super_edges_per_net = max_paths_per_net

    max_seg_to_seg_edges     = max_nets_per_group * max_seg_to_seg_edges_per_net
    max_seg_to_path_edges    = max_nets_per_group * max_seg_to_path_edges_per_net
    max_path_to_super_edges  = max_nets_per_group * max_path_to_super_edges_per_net
    max_super_to_super_edges = max_supers_per_net * max_nets_per_group * (max_nets_per_group - 1)

    seg_feats                  = np.zeros((max_segs_per_group, cfg.rl_seg_feat_dim),                                  dtype=np.float32)
    # no target in path features here
    path_feats                 = np.zeros((max_paths_per_group, cfg.rl_path_feat_dim - 1),                            dtype=np.float32)
    super_feats                = np.zeros((max_supers_per_group, 0),                                                  dtype=np.float32)
    seg_to_seg_edges           = np.full((max_seg_to_seg_edges, 2), [max_segs_per_group, max_segs_per_group],         dtype=np.float32)
    seg_to_path_edges          = np.full((max_seg_to_path_edges, 2), [max_paths_per_group, max_segs_per_group],       dtype=np.float32)
    path_to_super_edges        = np.full((max_path_to_super_edges, 2), [max_paths_per_group, max_supers_per_group],   dtype=np.float32)
    super_to_super_edges       = np.full((max_super_to_super_edges, 2), [max_supers_per_group, max_supers_per_group], dtype=np.float32)
    seg_valid_nodes            = np.zeros((max_segs_per_group,),                                                      dtype=bool)
    path_valid_nodes           = np.zeros((max_paths_per_group,),                                                     dtype=bool)
    super_valid_nodes          = np.zeros((max_supers_per_group,),                                                    dtype=bool)
    seg_to_seg_valid_edges     = np.zeros((max_seg_to_seg_edges,),                                                    dtype=bool)
    seg_to_path_valid_edges    = np.zeros((max_seg_to_path_edges,),                                                   dtype=bool)
    path_to_super_valid_edges  = np.zeros((max_path_to_super_edges,),                                                 dtype=bool)
    super_to_super_valid_edges = np.zeros((max_super_to_super_edges,),                                                dtype=bool)

    nets = match_group
    num_supers_in_group = 0
    for i, net in enumerate(nets):
        _seg_feats, _seg_to_seg_edges, _seg_valid_nodes, _seg_to_seg_valid_edges, _seg_masks, \
        _path_feats, _path_to_seg_edges, _path_valid_nodes, _path_to_seg_valid_edges = get_features_with_path(cfg,
                                                                                                              ac_rl,
                                                                                                              cir,
                                                                                                              net,
                                                                                                              route_dr_mgr,
                                                                                                              route_dr_ps,
                                                                                                              drc_mgr)

        _super_feats, _path_to_super_edges, \
        _super_valid_nodes, _path_to_super_valid_edges, \
        _num_super_nodes = get_super_feats_edges(cfg,
                                                 ac_rl,
                                                 cir,
                                                 net)
        
        # every net should have same super node count
        assert num_supers_in_group == _num_super_nodes if i > 0 else True
        num_supers_in_group = _num_super_nodes

        seg_shift   = i * max_segs_per_net
        path_shift  = i * max_paths_per_net
        super_shift = i * max_supers_per_net
        
        _seg_to_seg_edges          += seg_shift
        _path_to_seg_edges[:, 0]   += path_shift
        _path_to_seg_edges[:, 1]   += seg_shift
        _path_to_super_edges[:, 0] += path_shift
        _path_to_super_edges[:, 1] += super_shift

        n_s_start, n_s_end         = seg_shift, seg_shift + max_segs_per_net
        n_p_start, n_p_end         = path_shift, path_shift + max_paths_per_net
        n_super_start, n_super_end = super_shift, super_shift + max_supers_per_net

        e_ss_start, e_ss_end = i * max_seg_to_seg_edges_per_net, (i + 1) * max_seg_to_seg_edges_per_net
        e_sp_start, e_sp_end = i * max_seg_to_path_edges_per_net, (i + 1) * max_seg_to_path_edges_per_net
        # path-to-super
        e_ps_start, e_ps_end = i * max_path_to_super_edges_per_net, (i + 1) * max_path_to_super_edges_per_net

        seg_feats[n_s_start:n_s_end]             = _seg_feats
        path_feats[n_p_start:n_p_end]            = _path_feats
        super_feats[n_super_start:n_super_end]   = _super_feats

        seg_to_seg_edges[e_ss_start:e_ss_end]    = _seg_to_seg_edges
        seg_to_path_edges[e_sp_start:e_sp_end]   = _path_to_seg_edges[:, [1, 0]]
        path_to_super_edges[e_ps_start:e_ps_end] = _path_to_super_edges
        
        seg_valid_nodes[n_s_start:n_s_end]             = _seg_valid_nodes
        path_valid_nodes[n_p_start:n_p_end]            = _path_valid_nodes
        super_valid_nodes[n_super_start:n_super_end]   = _super_valid_nodes

        seg_to_seg_valid_edges[e_ss_start:e_ss_end]    = _seg_to_seg_valid_edges
        seg_to_path_valid_edges[e_sp_start:e_sp_end]   = _path_to_seg_valid_edges
        path_to_super_valid_edges[e_ps_start:e_ps_end] = _path_to_super_valid_edges


    
    # add super-to-super edges
    a = np.arange(len(nets)) * max_supers_per_net
    b = np.tile(a, (num_supers_in_group, 1))
    c = np.arange(num_supers_in_group)
    d = np.tile(c, (len(nets), 1)).T
    same_super_nodes_id = b + d
    cnt = 0

    path_match_cstr = nets[0].route_path_match_cstr_const()
    for i in range(path_match_cstr.num_path_cstrs()):
        path_cstr = path_match_cstr.path_cstr_const(i)
        for j in range(len(nets) - 1):
            net_j = nets[j]
            if path_cstr.has_net(net_j):
                for k in range(j, len(nets)):
                    net_k = nets[k]
                    if path_cstr.has_net(net_k):
                        super_to_super_edges[cnt] = [same_super_nodes_id[i, j], same_super_nodes_id[i, k]]
                        super_to_super_valid_edges[cnt] = True
                        cnt += 1


    return seg_feats, path_feats, super_feats, \
           seg_to_seg_edges, seg_to_path_edges, path_to_super_edges, super_to_super_edges, \
           seg_valid_nodes, path_valid_nodes, super_valid_nodes, \
           seg_to_seg_valid_edges, seg_to_path_valid_edges, path_to_super_valid_edges, super_to_super_valid_edges

def get_match_group_features_flat(cfg,
                                  ac_rl: ac.rl_utils,
                                  cir: ac.cir_db,
                                  match_group: list,
                                  paths_data, # from env
                                  route_dr_mgr: ac.route_dr_mgr,
                                  route_dr_ps: ac.route_dr_ps,
                                  drc_mgr: ac.drc_mgr):

    max_segs_per_net   = cfg.rl_max_segs
    max_paths_per_net  = cfg.rl_max_paths
    
    max_nets_per_group   = cfg.rl_max_nets_per_group
    max_segs_per_group   = max_nets_per_group * max_segs_per_net
    max_paths_per_group  = max_nets_per_group * max_paths_per_net
    max_supers_per_group = cfg.rl_max_path_cstrs # super path nodes (for intra net path matching handling)

    max_seg_to_seg_edges_per_net    = 12 * max_segs_per_net
    max_seg_to_path_edges_per_net   = max_segs_per_net * max_paths_per_net
    max_path_to_super_edges_per_net = max_paths_per_net

    max_seg_to_seg_edges     = max_nets_per_group * max_seg_to_seg_edges_per_net
    max_seg_to_path_edges    = max_nets_per_group * max_seg_to_path_edges_per_net
    max_path_to_super_edges  = max_nets_per_group * max_path_to_super_edges_per_net
    max_super_to_seg_edges   = max_supers_per_group * max_segs_per_group

    seg_feats                  = np.zeros((max_segs_per_group, cfg.rl_seg_feat_dim),                                  dtype=np.float32)
    path_feats                 = np.zeros((max_paths_per_group, cfg.rl_path_feat_dim - 1),                            dtype=np.float32) # no target in path features here
    # super_feats                = np.zeros((max_supers_per_group, 0),                                                  dtype=np.float32)
    super_feats                = np.zeros((max_supers_per_group, 3),                                                  dtype=np.float32)
    seg_to_seg_edges           = np.full((max_seg_to_seg_edges, 2), [max_segs_per_group, max_segs_per_group],         dtype=np.float32)
    seg_to_path_edges          = np.full((max_seg_to_path_edges, 2), [max_paths_per_group, max_segs_per_group],       dtype=np.float32)
    path_to_super_edges        = np.full((max_path_to_super_edges, 2), [max_paths_per_group, max_supers_per_group],   dtype=np.float32)
    super_to_seg_edges         = np.full((max_super_to_seg_edges, 2), [max_supers_per_group, max_segs_per_group],     dtype=np.float32)
    seg_valid_nodes            = np.zeros((max_segs_per_group,),                                                      dtype=bool)
    path_valid_nodes           = np.zeros((max_paths_per_group,),                                                     dtype=bool)
    super_valid_nodes          = np.zeros((max_supers_per_group,),                                                    dtype=bool)
    seg_to_seg_valid_edges     = np.zeros((max_seg_to_seg_edges,),                                                    dtype=bool)
    seg_to_path_valid_edges    = np.zeros((max_seg_to_path_edges,),                                                   dtype=bool)
    path_to_super_valid_edges  = np.zeros((max_path_to_super_edges,),                                                 dtype=bool)
    super_to_seg_valid_edges   = np.zeros((max_super_to_seg_edges,),                                                  dtype=bool)
    action_mask                = np.ones((max_segs_per_group + 1,),                                                   dtype=bool)

    nets = match_group
    num_super_to_seg_edges = 0
    for i, net in enumerate(nets):
        _seg_feats, _seg_to_seg_edges, _seg_valid_nodes, _seg_to_seg_valid_edges, _seg_masks, \
        _path_feats, _path_to_seg_edges, _path_valid_nodes, _path_to_seg_valid_edges = get_features_with_path(cfg,
                                                                                                              ac_rl,
                                                                                                              cir,
                                                                                                              net,
                                                                                                              route_dr_mgr,
                                                                                                              route_dr_ps,
                                                                                                              drc_mgr)
        _super_feats, _path_to_super_edges, \
        _super_valid_nodes, _path_to_super_valid_edges, \
        _num_super_nodes = get_super_feats_edges(cfg,
                                                 ac_rl,
                                                 cir,
                                                 net)


        seg_shift   = i * max_segs_per_net
        path_shift  = i * max_paths_per_net
      
        # super to seg
        # print(np.array(_path_to_seg_edges).shape)
        # print(np.array(_path_to_super_edges).shape)
        ps_edges = _path_to_seg_edges[_path_to_seg_valid_edges]
        for edge in ps_edges:
            path_idx, seg_idx = edge[0], edge[1]
            super_idx = net.conn_cstr_const(path_idx).idx()
            super_to_seg_edges[num_super_to_seg_edges] = [super_idx, seg_idx + seg_shift]
            super_to_seg_valid_edges[num_super_to_seg_edges] = True
            num_super_to_seg_edges += 1

        _seg_to_seg_edges          += seg_shift
        _path_to_seg_edges[:, 0]   += path_shift
        _path_to_seg_edges[:, 1]   += seg_shift
        _path_to_super_edges[:, 0] += path_shift
        
        n_s_start, n_s_end = seg_shift, seg_shift + max_segs_per_net
        n_p_start, n_p_end = path_shift, path_shift + max_paths_per_net
        
        e_ss_start, e_ss_end = i * max_seg_to_seg_edges_per_net, (i + 1) * max_seg_to_seg_edges_per_net
        e_sp_start, e_sp_end = i * max_seg_to_path_edges_per_net, (i + 1) * max_seg_to_path_edges_per_net
        e_ps_start, e_ps_end = i * max_path_to_super_edges_per_net, (i + 1) * max_path_to_super_edges_per_net # path-to-super
        
        seg_feats[n_s_start:n_s_end]             = _seg_feats
        path_feats[n_p_start:n_p_end]            = _path_feats
        
        seg_to_seg_edges[e_ss_start:e_ss_end]    = _seg_to_seg_edges
        seg_to_path_edges[e_sp_start:e_sp_end]   = _path_to_seg_edges[:, [1, 0]]
        path_to_super_edges[e_ps_start:e_ps_end] = _path_to_super_edges
        
        seg_valid_nodes[n_s_start:n_s_end]       = _seg_valid_nodes
        path_valid_nodes[n_p_start:n_p_end]      = _path_valid_nodes
        
        seg_to_seg_valid_edges[e_ss_start:e_ss_end]    = _seg_to_seg_valid_edges
        seg_to_path_valid_edges[e_sp_start:e_sp_end]   = _path_to_seg_valid_edges
        path_to_super_valid_edges[e_ps_start:e_ps_end] = _path_to_super_valid_edges

        action_mask[n_s_start:n_s_end] = _seg_masks[:-1]

    action_mask[-1] = 0

    for i, path_res in enumerate(paths_data):
        ptp = np.ptp(path_res) / cfg.rl_res_norm_constant
        avg = np.mean(path_res) / cfg.rl_res_norm_constant
        std = np.std(path_res) / cfg.rl_res_norm_constant
        super_feats[i] = [ptp, avg, std]

    super_valid_nodes[:len(paths_data)] = True


    return seg_feats, path_feats, super_feats, \
           seg_to_seg_edges, seg_to_path_edges, path_to_super_edges, super_to_seg_edges, \
           seg_valid_nodes, path_valid_nodes, super_valid_nodes, \
           seg_to_seg_valid_edges, seg_to_path_valid_edges, path_to_super_valid_edges, super_to_seg_valid_edges, \
           action_mask




##########################################
#                Utils                   #
##########################################
def seg_to_str(seg):
    return '({},{},{},{},{},{})'.format(seg.p0().x(),
                                        seg.p0().y(),
                                        seg.p0().z(),
                                        seg.p1().x(),
                                        seg.p1().y(),
                                        seg.p1().z())

def segs_to_str(segs):
    segs_str = ''
    for seg in segs:
        segs_str += '{} '.format(seg_to_str(seg))
    return segs_str[:-1]

def str_to_seg(seg_str):
    t = seg_str.split(',')
    assert len(t) == 6
    ux = int(t[0][1:])
    uy = int(t[1])
    uz = int(t[2])
    vx = int(t[3])
    vy = int(t[4])
    vz = int(t[5][:-1])
    return ac.segment3d_int(ux, uy, uz, vx, vy, vz)


def str_to_segs(segs_str):
    segs = []
    toks = segs_str.split()
    for tok in toks:
        segs.append(str_to_seg(tok))
    return segs


/*
* SPDX-FileCopyrightText: Copyright (c) 2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
* SPDX-License-Identifier: Apache-2.0
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
* http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/
#include "db/dbCir.hpp"
#include "io/parser.hpp"
#include "io/writer.hpp"
#include "place/placeMgr.hpp"
#include "route/routeMgr.hpp"
#include "route/routeGrMgr.hpp"
#include "route/routeDrMgr.hpp"
#include "route/routeDrPS.hpp"
#include "route/routeDrPost.hpp"
#include "cxxopts.hpp"
#include "rl/rlUtil.hpp"

//using PROJECT_NAMESPACE::CirDB;
//using PROJECT_NAMESPACE::Parser;
//using PROJECT_NAMESPACE::Writer;
//using PROJECT_NAMESPACE::PlaceMgr;
//using PROJECT_NAMESPACE::RouteMgr;
//using PROJECT_NAMESPACE::RouteGrMgr;
//using PROJECT_NAMESPACE::RouteDrMgr;
using namespace PROJECT_NAMESPACE;

int main(int argc, char** argv) {

  std::srand(1234);
  spdlog::set_pattern("[%^%l%$] %v");

  cxxopts::Options options(argv[0], " - command line options");
  options.add_options()
    ("laygo_config", "laygo configuration file (YAML)", cxxopts::value<std::string>())
    ("tech_grid", "technology grids file (YAML)", cxxopts::value<std::string>())
    ("tech_map", "device-primitive mapping file (YAML)", cxxopts::value<std::string>())
    ("constraint", "constraint files (YAML)", cxxopts::value<std::string>())
    ("lef", "lef file", cxxopts::value<std::vector<std::string>>())
    ("netlist", "Hspice netlist file", cxxopts::value<std::string>())
    ("drc", "Layer design rules file", cxxopts::value<std::string>())
    ("out_place", "placement output", cxxopts::value<std::string>())
    ("out_vis", "visualize", cxxopts::value<std::string>())
    ("out_gds", "GDSII output file", cxxopts::value<std::string>())
    ("out_def", "DEF output file", cxxopts::value<std::string>())
    ("out_tcl", "Tcl commands output file", cxxopts::value<std::string>())
    ("out_gv", "Gated Verilog netlist output file", cxxopts::value<std::string>())
    ("in_place", "placement input", cxxopts::value<std::string>())
    ("in_route_gds", "routing input (gds)", cxxopts::value<std::string>())
    ("threads", "max parallel threads", cxxopts::value<int>()->default_value("1"))
    ("place_ar", "placement aspect ratio", cxxopts::value<double>()->default_value("1"))
    ("place_ur", "placement utilization ratio", cxxopts::value<double>()->default_value("0.7"))
    ("place_ur_reg_name", "region name w.r.t. place_ur_reg_val", cxxopts::value<std::vector<std::string>>()->default_value(""))
    ("place_ur_reg_val", "placement utilization ratio value for regions", cxxopts::value<std::vector<double>>()->default_value(""))
    ("place_bbox", "placement bounding box", cxxopts::value<std::vector<double>>()->default_value(""))
    ("place_iter", "placement optimization iteration", cxxopts::value<int>()->default_value("1"))
    ("h,help", "Print usage")
    ;

  auto args = options.parse(argc, argv);

  if (args.count("help")) {
    std::cout << options.help() << std::endl;
    return 0;
  }

  CirDB cir;
  Parser par(cir);
  par.parse(args["laygo_config"].as<std::string>(),
            args["tech_grid"].as<std::string>(),
            args["tech_map"].as<std::string>(),
            args["constraint"].as<std::string>(),
            args["lef"].as<std::vector<std::string>>(),
            args["netlist"].as<std::string>(),
            args["drc"].as<std::string>()
            );
  cir.showInfo();
  cir.groupCells();
  //const auto& rg = cir.routeGrid(5);
  //std::cerr << rg.floorY(245) << std::endl;
  //std::cerr << rg.nextSignalY(720) << std::endl;
  //std::cerr << rg.nextSignalY(560) << std::endl;
  ////std::cerr << rg.prevSignalY(560) << std::endl;
  //std::cerr << rg.prevSignalY(400) << std::endl;

  ////cir.debug();
  //return 0;
  
  PlaceMgr placer(cir);
  if (args.count("in_place")) {
    par.parsePlace(args["in_place"].as<std::string>());
    placer.genEdgeCells();
    placer.genFillCells();
  }
  else {
    placer.solveSMT(args["threads"].as<int>(),
                    args["place_ar"].as<double>(),
                    args["place_ur"].as<double>(),
                    args["place_ur_reg_name"].as<std::vector<std::string>>(),
                    args["place_ur_reg_val"].as<std::vector<double>>(),
                    args["place_bbox"].as<std::vector<double>>(),
                    args["place_iter"].as<int>());
  }
  //cir.updateBbox();
  cir.showPlaceHpwl(false);

  cir.genRoutingBbox();
  
  cir.initSpatials();
  cir.buildSpatialPinsNObs();
  cir.buildSpatialNets();

  RouteMgr router(cir);
  RouteDrMgr dr(cir);
  if (args.count("in_route_gds")) {
    dr.init();
    par.parseRouteGds(args["in_route_gds"].as<std::string>());
    //dr.solvePS(args["threads"].as<int>());
  }
  else {
    router.genPg();
    dr.init();
    dr.solvePS(args["threads"].as<int>());
  }
  //DrPsRouter ps(dr);
  //Net& net = cir.net("mix");
  //ps.constructNetRoutables(net, false);
  //ps.ripup(net);
  //bool sc = ps.route(net, false, 12000, 2000);

  //RlUtils rl;
  //for (Int i = 0; i < 100000; ++i) {
    //Vector<Segment3d<Int>> v;
    //net.vArcs(v);
    //Vector<Segment3d<Int>> vs;
    //for (Int j = 0; j < v.size(); ++j) {
      //const auto& arc = v.at(j);
      //const auto& p0 = arc.min_corner();
      //const auto& p1 = arc.max_corner();
      //if (cir.spatialPins(p0.z()).exist(p0.to2d(), p0.to2d(), cir.pPin("xout_dcc/a"))) {
        //vs.emplace_back(arc);
      //}
      //if (cir.spatialPins(p1.z()).exist(p1.to2d(), p1.to2d(), cir.pPin("xout_dcc/a"))) {
        //vs.emplace_back(arc);
      //}
    //}
    //Vector<Vector<Segment3d<Int>>> vvSegs;
    //net.topo().vPaths(cir.pin("xmixer_3f<4>/y"), cir.pin("xmixer_4fe<4>/y"), vvSegs);
    //std::cerr << i << " " << vvSegs.size() << std::endl;
    //rl.toVisGds(cir, net, "test.gds");
    //if (vvSegs.empty()){
      //break;
    //}

    //ps.ripupSegmentNRefine(net, vs[rand() % vs.size()]);
    //if (!net.isRouted()) {
      //ps.route(net, false, 12000, 2000);
    //}

  //}

  
  PostRoute post(cir);
  post.patch();

  //for (Int i = 0; i < cir.numNets(); ++i) {
    //spdlog::info("{}: {}", cir.net(i).name(), cir.net(i).numWires());
  //}
 
  cir.showRouteWL();
  
  cir.genWsp();

  Writer wr(cir);
  if (args.count("out_place")) {
    wr.toPlace(args["out_place"].as<std::string>());
  }
  if (args.count("out_vis")) {
    wr.toVis(args["out_vis"].as<std::string>());
  }
  if (args.count("out_gds")) {
    wr.toGds(args["out_gds"].as<std::string>());
  }
  if (args.count("out_def")) {
    wr.toDef(args["out_def"].as<std::string>());
  }
  if (args.count("out_tcl")) {
    wr.toIccTcl(args["out_tcl"].as<std::string>());
  }
  if (args.count("out_gv")) {
    wr.toGv(args["out_gv"].as<std::string>());
  }
  return 0;
}


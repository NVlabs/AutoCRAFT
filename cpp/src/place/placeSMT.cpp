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

#include "placeSMT.hpp"
#include <numeric>

PROJECT_NAMESPACE_START
bool SmtPlacer::solve()
{
  
  setStep();

  Int W, H;
  Int xl, yl, xh, yh;
  if (_vBounds.empty()) {
    const Real A = calcExpectedDesignArea(_fcAreaExpand, _fcUtil);
    W = calcExpectedW(A);
    H = calcExpectedH(A);
    xl = yl = 0;
    xh = W / _stepX + 2;
    yh = H / _stepY + 1;
    _cir.bbox().set(0, 0, xh * _stepX, yh * _stepY);
    //spdlog::info("{} {} {} {} {} {}", W, H, xh, yh, _stepX, _stepY);
  }
  else {
    assert(_vBounds.size() == 4);
    const Int l = toDBUnit(_vBounds[0]);
    const Int b = toDBUnit(_vBounds[1]);
    const Int r = toDBUnit(_vBounds[2]);
    const Int t = toDBUnit(_vBounds[3]);
    xl = yl = 0;
    xh = (r - l) / _stepX;
    yh = (t - b) / _stepY;
    _cir.bbox().set(l, b, r, t);
  }

  setZ3Config();

  setZ3BvSize(xh, yh);
  spdlog::info("[SmtPlacer] Z3 bv size x: {} y: {}, hpwl: {}", _z3XBvSize, _z3YBvSize, _z3HpwlBvSize);


  // init solver 
  Z3Solver solver = (Z3Tactic(_ctx, "simplify") &
                     Z3Tactic(_ctx, "solve-eqs") &
                     Z3Tactic(_ctx, "bit-blast") &
                     //Z3Tactic(_ctx, "aig") &
                     Z3Tactic(_ctx, "sat")).mk_solver();
  setZ3SolverParams(solver);

  // init vars
  addZ3RegVarExprs();
  addZ3CellVarExprs();

  // region constraints
  addZ3CstrRegBoundaryLower(solver, xl, yl);
  addZ3CstrRegBoundaryUpper(solver, xh, yh);
  addZ3CstrRegSize(solver, true);
  addZ3CstrRegNonOverlap(solver);
  addZ3CstrRegAlign(solver);

  // cell constraints (within region)
  Int i;
  const Region* pReg;
  Cir_ForEachRegion(_cir, pReg, i) {
    addZ3CstrCellBoundaryLower(solver, *pReg);
    addZ3CstrCellBoundaryUpper(solver, *pReg);
    addZ3CstrCellNonOverlap(solver, *pReg);
    addZ3CstrCellSymmetry(solver, *pReg);
    addZ3CstrCellArray(solver, *pReg);
    addZ3CstrCellPowerAbutment(solver, *pReg);
    addZ3CstrCellEdgeDist(solver, *pReg);
  }
  addZ3CstrCellOrder(solver);
  addZ3CstrCellAlign(solver);
  addZ3CstrCellDisjoint(solver);
  addZ3CstrCellRow(solver);

  // array
  Cir_ForEachRegion(_cir, pReg, i) {
    addZ3CstrArrayArray(solver, *pReg);
    addZ3CstrArrayNonOverlap(solver, *pReg);
  }

  // io pin constraints
  //addZ3IOPinVarExprs(xl, yl, xh, yh);
  //addZ3CstrIOPinSlot(solver, xl, yl, xh, yh);
  //addZ3CstrIOPinLoc(solver, xl, yl, xh, yh);

  // routability constraints
  addZ3CstrNetSymDis(solver);
  addZ3CstrExtension(solver);
  addZ3PinDensityVarExprs(xl, yl, xh, yh);
  addZ3CstrRoutePinDensity(solver, xl, yl, xh, yh, 3, 2);
  

  // solve and eval
  Z3Model model(_ctx);

  // check sol
  bool useIO = false, useCluster = true;
  bool good = checkNUpdate(solver, model, useIO, useCluster); /* iter == 1 */
  if (!good) {
    return false;
  }
  
  // wirelength opt
  if (_maxHpwlOptIter > 1) {
    Int iter = 1;
    freezeZ3RegResult(solver, model);
    
    addZ3NetVarExprs();
    addZ3CstrNetHpwl(solver, useIO);
    addZ3CstrCellCluster(solver);

    if (good) good = wlOpt(++iter, solver, model, 0.3 * _fcHpwlOpt, useIO, useCluster); /* iter == 2 */
    //if (good) good = wlOpt(++iter, solver, model, 0.6 * _fcHpwlOpt, useIO, useCluster); 

    //solver.push();
    //freezeZ3CellResult(solver, model);
    //if (good) good = wlOpt(++iter, solver, model, 0.3 * _fcHpwlOpt, useIO, useCluster); /* iter == 3 */
    //if (good) good = wlOpt(++iter, solver, model, 0.2 * _fcHpwlOpt, useIO, useCluster); /* iter == 3 */
    //wlOpt(iter, solver, model, 0.3 * _fcHpwlOpt, useIO, useCluster); [> iter == 3 <]
    //wlOpt(iter, solver, model, 0.3 * _fcHpwlOpt, useIO, useCluster); [> iter == 3 <]
    //solver.pop();


    //freezeZ3IOPinResult(solver, model);
    if (good) good = wlOpt(++iter, solver, model, 0.8 * _fcHpwlOpt, useIO, useCluster); /* iter == 4 */
    if (good) good = wlOpt(++iter, solver, model, 0.6 * _fcHpwlOpt, useIO, useCluster); /* iter == 5 */
    if (good) good = wlOpt(++iter, solver, model, 0.5 * _fcHpwlOpt, useIO, useCluster); /* iter == 6 */
    if (good) good = wlOpt(++iter, solver, model, 0.4 * _fcHpwlOpt, useIO, useCluster); /* iter == 7 */
    if (good) good = wlOpt(++iter, solver, model, 0.4 * _fcHpwlOpt, useIO, useCluster); /* iter == 8 */
    if (good) good = wlOpt(++iter, solver, model, 0.3 * _fcHpwlOpt, useIO, useCluster); /* iter == 9 */
    if (good) good = wlOpt(++iter, solver, model, 0.2 * _fcHpwlOpt, useIO, useCluster); /* iter == 10 */
  }

  setRegResult(model);
  setCellResult(model);
  //setIOPinResult(model);


  Z3_finalize_memory();

  return true;
}

void SmtPlacer::setStep()
{
  const auto stepXY = calcGridStep();
  _stepX = stepXY.first;
  _stepY = stepXY.second;

  Int i;
  const Region* pReg;
  Cir_ForEachRegion(_cir, pReg, i) {
    assert(pReg->placeGrid().stepX() % _stepX == 0);
    assert(pReg->placeGrid().stepY() % _stepY == 0);
    _vStepXFactors[pReg->idx()] = pReg->placeGrid().stepX() / _stepX;
    _vStepYFactors[pReg->idx()] = pReg->placeGrid().stepY() / _stepY;
  }

}

bool SmtPlacer::checkNUpdate(Z3Solver& solver, Z3Model& model, const bool useIO, const bool useCluster) const
{
  const Z3CheckRes res = solver.check();
  if (res == z3::sat) {
    model = solver.get_model();
    const Int hpwl = calcZ3NetHpwl(model, useIO, useCluster);
    spdlog::info("[SmtPlacer] sat, hpwl: {}", hpwl);
    return true;
  }
  else if (res == z3::unsat) {
    Z3ExprVector core = solver.unsat_core();
    spdlog::warn("[SmtPlacer] unsat {}", core.size());
    for (size_t i = 0; i < core.size(); ++i) {
      std::cerr << core[i] << std::endl;
    }
  }
  else {
    spdlog::warn("[SmtPlacer] unknown");
  }
  return false;
}

bool SmtPlacer::wlOpt(const Int iter, Z3Solver& solver, Z3Model& model, const Real fcHpwlOpt, const bool useIO, const bool useCluster)
{
  if (iter > _maxHpwlOptIter) {
    return false;
  }
  const Int hpwl = calcZ3NetHpwl(model, useIO, useCluster);
  Vector<Int> vHpwlCandidates;
  genHpwlCandidates(hpwl, hpwl * (1. - fcHpwlOpt), 1, vHpwlCandidates);

  bool sat = false;
  for (const Int tarHpwl : vHpwlCandidates) {
    solver.push();
    spdlog::info("[SmtPlacer] Iter: {}, target hpwl: {}", iter, tarHpwl);
    solver.add(z3::ule(*_pTotHpwlExpr, _ctx.bv_val(static_cast<unsigned>(tarHpwl), _z3HpwlBvSize)));
    if (checkNUpdate(solver, model, useIO, useCluster)) {
      sat = true;
      //solver.pop();
      break;
    }
    //solver.pop();
  }
  return sat;
}

void SmtPlacer::setZ3Config()
{
  z3::set_param("auto_config", _z3AutoConfig);
  z3::set_param("model", _z3Model);
  z3::set_param("verbose", _z3Verbose);
  z3::set_param("smt.arith.solver", _z3SmtArithSolver);
  if (_z3Threads) {
    z3::set_param("parallel.enable", true);
    z3::set_param("smt.threads", _z3Threads);
    z3::set_param("sat.threads", _z3Threads);
  }
  z3::set_param("sat.restart.max", _z3SatRestartMax);
  z3::set_param("unsat_core", _z3UnsatCore);
}

void SmtPlacer::setZ3BvSize(Int xh, Int yh)
{
  _z3XBvSize = calcLog2(xh) + 2;
  _z3YBvSize = calcLog2(yh) + 2;
  _z3HpwlBvSize = calcLog2((_cir.numNets() + _cir.numPlaceClusterCstrs()) * (xh + yh)) + 2;
  //_z3HpwlBvSize = calcLog2(_cir.numNets() * (xh + yh)) + 1;
  _z3PinCntBvSize = calcLog2(_winX * _winY * 4) + 1;
}

void SmtPlacer::setZ3SolverParams(Z3Solver& solver)
{
  Z3Params p(solver.ctx());
  // p.set("priority", ctx.str_symbol("lex"));
  p.set("timeout", _z3Timeout);
  solver.set(p);
}

void SmtPlacer::addZ3RegVarExprs()
{
  spdlog::info("[SmtPlacer] Add Z3 region variables");
  Int i;
  const Region* pReg;
  Cir_ForEachRegion(_cir, pReg, i) {
    _vpRegXExprs[pReg->idx()] = std::make_unique<Z3Expr>(_ctx.bv_const((pReg->name() + "_x").c_str(), _z3XBvSize));
    _vpRegYExprs[pReg->idx()] = std::make_unique<Z3Expr>(_ctx.bv_const((pReg->name() + "_y").c_str(), _z3YBvSize));
    _vpRegWExprs[pReg->idx()] = std::make_unique<Z3Expr>(_ctx.bv_const((pReg->name() + "_w").c_str(), _z3XBvSize));
    _vpRegHExprs[pReg->idx()] = std::make_unique<Z3Expr>(_ctx.bv_const((pReg->name() + "_h").c_str(), _z3YBvSize));
  }
}

void SmtPlacer::addZ3CellVarExprs()
{
  spdlog::info("[SmtPlacer] Add Z3 cell variables");
  Int i;
  const Cell* pCell;
  Cir_ForEachCell(_cir, pCell, i) {
    _vpCellXExprs[pCell->idx()] = std::make_unique<Z3Expr>(_ctx.bv_const((pCell->name() + "_x").c_str(), _z3XBvSize));
    _vpCellYExprs[pCell->idx()] = std::make_unique<Z3Expr>(_ctx.bv_const((pCell->name() + "_y").c_str(), _z3YBvSize));
  }
}

void SmtPlacer::addZ3NetVarExprs()
{
  spdlog::info("[SmtPlacer] Add Z3 net variables");
  Int i;
  const Net* pNet;
  Cir_ForEachNet(_cir, pNet, i) {
    if (!pNet->isPower() and pNet->numPins() > 1) {
      _vpNetMinXExprs[pNet->idx()] = std::make_unique<Z3Expr>(_ctx.bv_const((pNet->name() + "_net_mnx").c_str(), _z3XBvSize));
      _vpNetMaxXExprs[pNet->idx()] = std::make_unique<Z3Expr>(_ctx.bv_const((pNet->name() + "_net_mxx").c_str(), _z3XBvSize));
      _vpNetMinYExprs[pNet->idx()] = std::make_unique<Z3Expr>(_ctx.bv_const((pNet->name() + "_net_mny").c_str(), _z3YBvSize));
      _vpNetMaxYExprs[pNet->idx()] = std::make_unique<Z3Expr>(_ctx.bv_const((pNet->name() + "_net_mxy").c_str(), _z3YBvSize));
    }
  }
}


void SmtPlacer::addZ3IOPinVarExprs(const Int xl, const Int yl, const Int xh, const Int yh) 
{
  spdlog::info("[SmtPlacer] Add Z3 IO pin variables");
  Int i;
  const Pin* pPin;
  Cir_ForEachLeftIOPin(_cir, pPin, i) {
    for (Int y = yl + 1; y < yh - 1; ++y) {
      _vvpIOLeftSlotExprs[i].emplace_back(std::make_unique<Z3Expr>(_ctx.bool_const((pPin->name() + "_ioSlot_l_" + std::to_string(y)).c_str())));  
    }
  }
  Cir_ForEachRightIOPin(_cir, pPin, i) {
    for (Int y = yl + 1; y < yh - 1; ++y) {
      _vvpIORightSlotExprs[i].emplace_back(std::make_unique<Z3Expr>(_ctx.bool_const((pPin->name() + "_ioSlot_r_" + std::to_string(y)).c_str())));  
    }
  }
  Cir_ForEachBottomIOPin(_cir, pPin, i) {
    for (Int x = xl + 1; x < xh - 1; ++x) {
      _vvpIOBottomSlotExprs[i].emplace_back(std::make_unique<Z3Expr>(_ctx.bool_const((pPin->name() + "_ioSlot_b_" + std::to_string(x)).c_str())));  
    }
  }
  Cir_ForEachTopIOPin(_cir, pPin, i) {
    for (Int x = xl + 1; x < xh - 1; ++x) {
      _vvpIOTopSlotExprs[i].emplace_back(std::make_unique<Z3Expr>(_ctx.bool_const((pPin->name() + "_ioSlot_t_" + std::to_string(x)).c_str())));  
    }
  }
  Cir_ForEachIOPin(_cir, pPin, i) {
    assert(pPin->isIO() and pPin->ioIdx() == i);
    _vpIOPinXExprs[pPin->ioIdx()] = std::make_unique<Z3Expr>(_ctx.bv_const((pPin->name() + "_ioPin_x").c_str(), _z3XBvSize));
    _vpIOPinYExprs[pPin->ioIdx()] = std::make_unique<Z3Expr>(_ctx.bv_const((pPin->name() + "_ioPin_y").c_str(), _z3YBvSize));
  }
}

void SmtPlacer::addZ3PinDensityVarExprs(const Int xl, const Int yl, const Int xh, const Int yh)
{
  spdlog::info("[SmtPlacer] Add Z3 pin density variables");

  Int i;
  const Cell* pCell;
  //Cir_ForEachCell(_cir, pCell, i) {
    //const Cell& c = *pCell;
    //const Int numX = xh - xl - 1;
    //const Int numY = yh - yl - 1;
    //_vpPinDensityExprs[c.idx()].resize(numX, numY);
    //for (Int x = xl + 1; x < xh; ++x) {
      //for (Int y = yl + 1; y < yh; ++y) {
        //const String varName = c.name() + "_pin_den_" + std::to_string(x) + "_" + std::to_string(y);
        //const Int xIdx = x - xl - 1;
        //const Int yIdx = y - yl - 1;
        //_vpPinDensityExprs[c.idx()].at(xIdx, yIdx) = std::make_unique<Z3Expr>(_ctx.bool_const(varName.c_str()));
      //}
    //}
  //}
  Int j, k;
  Cir_ForEachCell(_cir, pCell, i) {
    const Cell& c = *pCell;
    const Int numXSlots = xh - xl - 2; // [xl + 1, xh)
    const Int numYSlots = yh - yl - 2; // [yl + 1, yh)
    const Int numXWins = numXSlots - _winX + 1;
    const Int numYWins = numYSlots - _winY + 1;
    _vpPinDensityExprs[c.idx()].resize(numXWins, numYWins);
    for (j = 0; j < numXWins; ++j) {
      for (k = 0; k < numYWins; ++k) {
        const String varName = c.name() + "_pin_den_" + std::to_string(j) + "_" + std::to_string(k);
        _vpPinDensityExprs[c.idx()].at(j, k) = std::make_unique<Z3Expr>(_ctx.bool_const(varName.c_str()));
      }
    }
  }

}

void SmtPlacer::addZ3CstrRegBoundaryLower(Z3Solver& solver, const Int xl, const Int yl)
{
  spdlog::info("[SmtPlacer] Add Z3 region boundary constraints (lower)");
  
  const Z3Expr& x = _ctx.bv_val(static_cast<unsigned>(xl), _z3XBvSize);
  const Z3Expr& y = _ctx.bv_val(static_cast<unsigned>(yl), _z3YBvSize);

  Int i;
  const Region* pReg;
  Cir_ForEachRegion(_cir, pReg, i) {
    const Z3Expr& xi = *_vpRegXExprs.at(pReg->idx());
    const Z3Expr& yi = *_vpRegYExprs.at(pReg->idx());
    const Primitive& epw = pReg->edgePrimWest();
    const Primitive& eps = pReg->edgePrimSouth();
    //assert(pReg->edgePrimWestOrient() == Orient2dE::n or pReg->edgePrimWestOrient() == Orient2dE::fs);
    //assert(pReg->edgePrimSouthOrient() == Orient2dE::n or pReg->edgePrimSouthOrient() == Orient2dE::fs);
    const Int wEpw = epw.sizeX() % _stepX != 0 ? epw.sizeX() / _stepX + 1 : epw.sizeX() / _stepX;
    const Int hEps = eps.sizeY() % _stepY != 0 ? eps.sizeY() / _stepY + 1 : eps.sizeY() / _stepY;

    const Z3Expr& xSpace = _ctx.bv_val(static_cast<unsigned>(wEpw), _z3XBvSize);
    const Z3Expr& ySpace = _ctx.bv_val(static_cast<unsigned>(hEps), _z3YBvSize);
    solver.add(z3::ule(x + xSpace, xi));
    solver.add(z3::ule(y + ySpace, yi));
    solver.add(z3::bvadd_no_overflow(x, xSpace, false));
    solver.add(z3::bvadd_no_overflow(y, ySpace, false));
  }

}

void SmtPlacer::addZ3CstrRegBoundaryUpper(Z3Solver& solver, const Int xh, const Int yh)
{
  spdlog::info("[SmtPlacer] Add Z3 region boundary constraints (upper)");
  
  const Z3Expr& x = _ctx.bv_val(static_cast<unsigned>(xh), _z3XBvSize);
  const Z3Expr& y = _ctx.bv_val(static_cast<unsigned>(yh), _z3YBvSize);
  
  Int i;
  const Region* pReg;
  Cir_ForEachRegion(_cir, pReg, i) {
    const Z3Expr& xi = *_vpRegXExprs.at(pReg->idx());
    const Z3Expr& yi = *_vpRegYExprs.at(pReg->idx());
    const Z3Expr& wi = *_vpRegWExprs.at(pReg->idx());
    const Z3Expr& hi = *_vpRegHExprs.at(pReg->idx());
    const Primitive& epe = pReg->edgePrimEast();
    const Primitive& epn = pReg->edgePrimNorth();
    //assert(pReg->edgePrimEastOrient() == Orient2dE::n or pReg->edgePrimEastOrient() == Orient2dE::fs);
    //assert(pReg->edgePrimNorthOrient() == Orient2dE::n or pReg->edgePrimNorthOrient() == Orient2dE::fs);
    const Int wEpe = epe.sizeX() % _stepX != 0 ? epe.sizeX() / _stepX + 1 : epe.sizeX() / _stepX;
    const Int hEpn = epn.sizeY() % _stepY != 0 ? epn.sizeY() / _stepY + 1 : epn.sizeY() / _stepY;

    const Z3Expr& xSpace = _ctx.bv_val(static_cast<unsigned>(wEpe), _z3XBvSize);
    const Z3Expr& ySpace = _ctx.bv_val(static_cast<unsigned>(hEpn), _z3YBvSize);
    //std::cerr << xSpace.get_numeral_int() << " " << ySpace.get_numeral_int()<< std::endl;
    solver.add(z3::ule(xi + wi + xSpace, x));
    solver.add(z3::ule(yi + hi + ySpace, y));
    solver.add(z3::bvadd_no_overflow(xi, wi, false));
    solver.add(z3::bvadd_no_overflow(yi, hi, false));
    solver.add(z3::bvadd_no_overflow(xi + wi, xSpace, false));
    solver.add(z3::bvadd_no_overflow(yi + hi, ySpace, false));
  }
}

void SmtPlacer::addZ3CstrRegSize(Z3Solver& solver, const bool isEvenRow)
{
  spdlog::info("[SmtPlacer] Add Z3 region size constraints");

  Int i, j;
  const Region* pReg;
  const Cell* pCell;
  Cir_ForEachRegion(_cir, pReg, i) {
    const Z3Expr& w = *_vpRegWExprs.at(pReg->idx());
    const Z3Expr& h = *_vpRegHExprs.at(pReg->idx());
    Int area = 0;
    Reg_ForEachCell((*pReg), pCell, j) { 
      area += pCell->area();
    }
    area /= _stepX;
    area /= _stepY;
    //auto it = _mFcRegUtils.find(pReg);
    auto it = _mFcRegUtils.find(pReg->name());
    if (it != _mFcRegUtils.end()) {
      area /= it->second;
    }
    else {
      area /= _fcUtil;
    }

    Vector<Pair<Int, Int>> vApproxFactors;
    calcApproxFactors(area, isEvenRow, vApproxFactors);
    Z3ExprVector ev(_ctx);
    for (const auto& p : vApproxFactors) {
      if (calcLog2(p.first) + 1 > _z3XBvSize)
        continue;
      if (calcLog2(p.first) + 1 > _z3YBvSize)
        continue;
      if (calcLog2(p.second) + 1 > _z3XBvSize)
        continue;
      if (calcLog2(p.second) + 1 > _z3YBvSize)
        continue;
      const Z3Expr& width = _ctx.bv_val(static_cast<unsigned>(p.first), _z3XBvSize);
      const Z3Expr& height = _ctx.bv_val(static_cast<unsigned>(p.second), _z3YBvSize);
      ev.push_back(w == width && h == height);
    }
    solver.add(z3::mk_or(ev));
  }
}

void SmtPlacer::addZ3CstrRegSize(Z3Solver& solver, const Region& reg, const Int xSize, const Int ySize)
{
  spdlog::info("[SmtPlacer] Set Z3 region {} size {} {}", reg.name(), xSize, ySize);

  const Z3Expr& w = *_vpRegWExprs.at(reg.idx());
  const Z3Expr& h = *_vpRegHExprs.at(reg.idx());

  solver.add(w == _ctx.bv_val(static_cast<unsigned>(xSize), _z3XBvSize));
  solver.add(h == _ctx.bv_val(static_cast<unsigned>(ySize), _z3YBvSize));
}

void SmtPlacer::addZ3CstrRegNonOverlap(Z3Solver& solver)
{
  spdlog::info("[SmtPlacer] Add Z3 region non-overlap constraints");

  for (Int i = 0; i < _cir.numRegions() - 1; ++i) {
    const Region& ri = _cir.region(i);
    const Z3Expr& xi = *_vpRegXExprs.at(ri.idx());
    const Z3Expr& yi = *_vpRegYExprs.at(ri.idx());
    const Z3Expr& wi = *_vpRegWExprs.at(ri.idx());
    const Z3Expr& hi = *_vpRegHExprs.at(ri.idx());
    const Primitive& epei = ri.edgePrimEast();
    const Primitive& epwi = ri.edgePrimWest();
    const Primitive& epni = ri.edgePrimNorth();
    const Primitive& epsi = ri.edgePrimSouth();
    //assert(ri.edgePrimEastOrient() == Orient2dE::n or ri.edgePrimEastOrient() == Orient2dE::fs);
    //assert(ri.edgePrimWestOrient() == Orient2dE::n or ri.edgePrimWestOrient() == Orient2dE::fs);
    //assert(ri.edgePrimNorthOrient() == Orient2dE::n or ri.edgePrimNorthOrient() == Orient2dE::fs);
    //assert(ri.edgePrimSouthOrient() == Orient2dE::n or ri.edgePrimSouthOrient() == Orient2dE::fs);
    const Int wEpei = epei.sizeX() % _stepX != 0 ? epei.sizeX() / _stepX + 1 : epei.sizeX() / _stepX;
    const Int wEpwi = epwi.sizeX() % _stepX != 0 ? epwi.sizeX() / _stepX + 1 : epwi.sizeX() / _stepX;
    const Int hEpni = epni.sizeY() % _stepY != 0 ? epni.sizeY() / _stepY + 1 : epni.sizeY() / _stepY;
    const Int hEpsi = epsi.sizeY() % _stepY != 0 ? epsi.sizeY() / _stepY + 1 : epsi.sizeY() / _stepY;

    const Z3Expr& xlSpacei = _ctx.bv_val(static_cast<unsigned>(wEpwi), _z3XBvSize);
    const Z3Expr& ylSpacei = _ctx.bv_val(static_cast<unsigned>(hEpsi), _z3YBvSize);
    const Z3Expr& xhSpacei = _ctx.bv_val(static_cast<unsigned>(wEpei), _z3XBvSize);
    const Z3Expr& yhSpacei = _ctx.bv_val(static_cast<unsigned>(hEpni), _z3YBvSize);

    for (Int j = i + 1; j < _cir.numRegions(); ++j) {
      const Region& rj = _cir.region(j);
      const Z3Expr& xj = *_vpRegXExprs.at(rj.idx());
      const Z3Expr& yj = *_vpRegYExprs.at(rj.idx());
      const Z3Expr& wj = *_vpRegWExprs.at(rj.idx());
      const Z3Expr& hj = *_vpRegHExprs.at(rj.idx());
      const Primitive& epej = rj.edgePrimEast();
      const Primitive& epwj = rj.edgePrimWest();
      const Primitive& epnj = rj.edgePrimNorth();
      const Primitive& epsj = rj.edgePrimSouth();
      //assert(rj.edgePrimEastOrient() == Orient2dE::n or rj.edgePrimEastOrient() == Orient2dE::fs);
      //assert(rj.edgePrimWestOrient() == Orient2dE::n or rj.edgePrimWestOrient() == Orient2dE::fs);
      //assert(rj.edgePrimNorthOrient() == Orient2dE::n or rj.edgePrimNorthOrient() == Orient2dE::fs);
      //assert(rj.edgePrimSouthOrient() == Orient2dE::n or rj.edgePrimSouthOrient() == Orient2dE::fs);
      const Int wEpej = epej.sizeX() % _stepX != 0 ? epej.sizeX() / _stepX + 1 : epej.sizeX() / _stepX;
      const Int wEpwj = epwj.sizeX() % _stepX != 0 ? epwj.sizeX() / _stepX + 1 : epwj.sizeX() / _stepX;
      const Int hEpnj = epnj.sizeY() % _stepY != 0 ? epnj.sizeY() / _stepY + 1 : epnj.sizeY() / _stepY;
      const Int hEpsj = epsj.sizeY() % _stepY != 0 ? epsj.sizeY() / _stepY + 1 : epsj.sizeY() / _stepY;
      
      const Z3Expr& xlSpacej = _ctx.bv_val(static_cast<unsigned>(wEpwj), _z3XBvSize);
      const Z3Expr& ylSpacej = _ctx.bv_val(static_cast<unsigned>(hEpsj), _z3YBvSize);
      const Z3Expr& xhSpacej = _ctx.bv_val(static_cast<unsigned>(wEpej), _z3XBvSize);
      const Z3Expr& yhSpacej = _ctx.bv_val(static_cast<unsigned>(hEpnj), _z3YBvSize);

      solver.add(z3::ule(xi + wi + xhSpacei + xlSpacej, xj) ||
                 z3::ule(xj + wj + xhSpacej + xlSpacei, xi) ||
                 z3::ule(yi + hi + yhSpacei + ylSpacej, yj) ||
                 z3::ule(yj + hj + yhSpacej + ylSpacei, yi));
      solver.add(z3::bvadd_no_overflow(xi + wi + xhSpacei, xlSpacej, false));
      solver.add(z3::bvadd_no_overflow(xj + wj + xhSpacej, xlSpacei, false));
      solver.add(z3::bvadd_no_overflow(yi + hi + yhSpacei, ylSpacej, false));
      solver.add(z3::bvadd_no_overflow(yj + hj + yhSpacej, ylSpacei, false));
    }
  }
  // overflow handling
  for (Int i = 0; i < _cir.numRegions(); ++i) {
    const Region& ri = _cir.region(i);
    const Z3Expr& xi = *_vpRegXExprs.at(ri.idx());
    const Z3Expr& yi = *_vpRegYExprs.at(ri.idx());
    const Z3Expr& wi = *_vpRegWExprs.at(ri.idx());
    const Z3Expr& hi = *_vpRegHExprs.at(ri.idx());
    const Primitive& epei = ri.edgePrimEast();
    const Primitive& epwi = ri.edgePrimWest();
    const Primitive& epni = ri.edgePrimNorth();
    const Primitive& epsi = ri.edgePrimSouth();
    const Int wEpei = epei.sizeX() % _stepX != 0 ? epei.sizeX() / _stepX + 1 : epei.sizeX() / _stepX;
    const Int wEpwi = epwi.sizeX() % _stepX != 0 ? epwi.sizeX() / _stepX + 1 : epwi.sizeX() / _stepX;
    const Int hEpni = epni.sizeY() % _stepY != 0 ? epni.sizeY() / _stepY + 1 : epni.sizeY() / _stepY;
    const Int hEpsi = epsi.sizeY() % _stepY != 0 ? epsi.sizeY() / _stepY + 1 : epsi.sizeY() / _stepY;
    const Z3Expr& xlSpacei = _ctx.bv_val(static_cast<unsigned>(wEpwi), _z3XBvSize);
    const Z3Expr& ylSpacei = _ctx.bv_val(static_cast<unsigned>(hEpsi), _z3YBvSize);
    const Z3Expr& xhSpacei = _ctx.bv_val(static_cast<unsigned>(wEpei), _z3XBvSize);
    const Z3Expr& yhSpacei = _ctx.bv_val(static_cast<unsigned>(hEpni), _z3YBvSize);

    solver.add(z3::bvadd_no_overflow(xi + wi, xhSpacei, false));
    solver.add(z3::bvadd_no_overflow(yi + hi, yhSpacei, false));
  }
}

void SmtPlacer::addZ3CstrRegAlign(Z3Solver& solver)
{
  if (_cir.numPlaceAlignCstrs() == 0) {
    return;
  }
  
  Int i;
  const PlaceAlignCstr* pCstr;
  Vector<Int> vCstrIds;
  Cir_ForEachPlaceAlignCstr(_cir, pCstr, i) {
    if (pCstr->numRegions() > 0) {
      assert(pCstr->numCells() == 0);
      vCstrIds.emplace_back(pCstr->idx());
    }
  }

  if (vCstrIds.empty()) {
    return;
  }

  spdlog::info("[SmtPlacer] Add Z3 region alignment constraints");
  
  for (const Int idx : vCstrIds) {
    const PlaceAlignCstr& ac = _cir.placeAlignCstr(idx);
    if (ac.isHor()) {
      if (ac.isLow()) { // bottom alignment
        const Z3Expr& y = *_vpRegYExprs.at(ac.region(0).idx());
        for (Int j = 1; j < ac.numRegions(); ++j) {
          const Z3Expr& yCur = *_vpRegYExprs.at(ac.region(j).idx());
          solver.add(y == yCur);
        }
      }
      else { // top alignment
        const Region& r = ac.region(0);
        const Z3Expr& y = *_vpRegYExprs.at(r.idx());
        const Z3Expr& h = *_vpRegHExprs.at(r.idx());
        for (Int j = 1; j < ac.numRegions(); ++j) {
          const Region& rCur = ac.region(j);
          const Z3Expr& yCur = *_vpRegYExprs.at(rCur.idx());
          const Z3Expr& hCur = *_vpRegHExprs.at(rCur.idx());
          solver.add((y + h) == (yCur + hCur));
        }
      }
    }
    else {
      assert(ac.isVer());
      if (ac.isLow()) { // left alignment
        const Z3Expr& x = *_vpRegXExprs.at(ac.region(0).idx());
        for (Int j = 1; j < ac.numRegions(); ++j) {
          const Z3Expr& xCur = *_vpRegXExprs.at(ac.region(j).idx());
          solver.add(x == xCur);
        }
      }
      else { // right alignment
        const Region& r = ac.region(0);
        const Z3Expr& x = *_vpRegXExprs.at(r.idx());
        const Z3Expr& w = *_vpRegWExprs.at(r.idx());
        for (Int j = 1; j < ac.numRegions(); ++j) {
          const Region& rCur = ac.region(j);
          const Z3Expr& xCur = *_vpRegXExprs.at(rCur.idx());
          const Z3Expr& wCur = *_vpRegWExprs.at(rCur.idx());
          solver.add((x + w) == (xCur == wCur));
        }
      }
    }
  }

  
}

void SmtPlacer::addZ3CstrCellBoundaryLower(Z3Solver& solver, const Region& reg)
{
  spdlog::info("[SmtPlacer] Add Z3 cell boundary constraints (lower)");

  const Z3Expr& xl = *_vpRegXExprs.at(reg.idx());
  const Z3Expr& yl = *_vpRegYExprs.at(reg.idx());

  for (Int i = 0; i < reg.numCells(); ++i) {
    const Cell&   ci = reg.cell(i);
    const Z3Expr& xi = *_vpCellXExprs.at(ci.idx());
    const Z3Expr& yi = *_vpCellYExprs.at(ci.idx());
    solver.add(z3::ule(xl, xi));
    solver.add(z3::ule(yl, yi));
  }
}

void SmtPlacer::addZ3CstrCellBoundaryUpper(Z3Solver& solver, const Region& reg)
{
  spdlog::info("[SmtPlacer] Add Z3 cell boundary constraints (upper)");

  const Z3Expr& xl = *_vpRegXExprs.at(reg.idx());
  const Z3Expr& yl = *_vpRegYExprs.at(reg.idx());
  const Z3Expr& w  = *_vpRegWExprs.at(reg.idx());
  const Z3Expr& h  = *_vpRegHExprs.at(reg.idx());

  for (Int i = 0; i < reg.numCells(); ++i) {
    const Cell&   ci = reg.cell(i);
    const Z3Expr& xi = *_vpCellXExprs.at(ci.idx());
    const Z3Expr& yi = *_vpCellYExprs.at(ci.idx());
    const Z3Expr& wi = _ctx.bv_val(static_cast<unsigned>(ci.width() / _stepX), _z3XBvSize);
    const Z3Expr& hi = _ctx.bv_val(static_cast<unsigned>(ci.height() / _stepY), _z3YBvSize);
    solver.add(z3::bvadd_no_overflow(xi, wi, false));
    solver.add(z3::bvadd_no_overflow(yi, hi, false));
    solver.add(z3::ule(xi + wi, xl + w));
    solver.add(z3::ule(yi + hi, yl + h));
  }
}

void SmtPlacer::addZ3CstrCellNonOverlap(Z3Solver& solver, const Region& reg)
{
  spdlog::info("[SmtPlacer] Add Z3 cell non-overlap constraints");
  for (Int i = 0; i < reg.numCells() - 1; ++i) {
    const Cell&   ci = reg.cell(i);
    const Z3Expr& xi = *_vpCellXExprs.at(ci.idx());
    const Z3Expr& yi = *_vpCellYExprs.at(ci.idx());
    const Z3Expr& wi = _ctx.bv_val(static_cast<unsigned>(ci.width() / _stepX), _z3XBvSize);
    const Z3Expr& hi = _ctx.bv_val(static_cast<unsigned>(ci.height() / _stepY), _z3YBvSize);
    //solver.add(z3::bvadd_no_overflow(xi, wi, false));
    //solver.add(z3::bvadd_no_overflow(yi, hi, false));
    for (Int j = i + 1; j < reg.numCells(); ++j) {
      const Cell&   cj = reg.cell(j);
      const Z3Expr& xj = *_vpCellXExprs.at(cj.idx());
      const Z3Expr& yj = *_vpCellYExprs.at(cj.idx());
      const Z3Expr& wj = _ctx.bv_val(static_cast<unsigned>(cj.width() / _stepX), _z3XBvSize);
      const Z3Expr& hj = _ctx.bv_val(static_cast<unsigned>(cj.height() / _stepY), _z3YBvSize);
      //solver.add(z3::bvadd_no_overflow(xj, wj, false));
      //solver.add(z3::bvadd_no_overflow(yj, hj, false));
      solver.add(z3::ule(xi + wi, xj) || z3::ule(xj + wj, xi) ||
                 z3::ule(yi + hi, yj) || z3::ule(yj + hj, yi));
    }
  }
}

void SmtPlacer::addZ3CstrCellArray(Z3Solver& solver, const Region& reg)
{
  Int i;
  const PlaceArrayCstr* pCstr;
  Vector<Int> vPlaceArrayCstrIds;

  Reg_ForEachPlaceArrayCstr(reg, pCstr, i) {
    if (pCstr->numCells() > 0) {
      vPlaceArrayCstrIds.emplace_back(pCstr->idx());
    }
  }
  if (vPlaceArrayCstrIds.size() == 0) {
    return;
  }

  spdlog::info("[SmtPlacer] Add Z3 cell array constraints");
  
  for (const Int idx : vPlaceArrayCstrIds) {
    const PlaceArrayCstr& ac = _cir.placeArrayCstr(idx);
    assert(ac.numCells() > 0);

    _vpArrayMinXExprs[ac.idx()] = std::make_unique<Z3Expr>(_ctx.bv_const((ac.name() + "_mnx").c_str(), _z3XBvSize));
    _vpArrayMaxXExprs[ac.idx()] = std::make_unique<Z3Expr>(_ctx.bv_const((ac.name() + "_mxx").c_str(), _z3XBvSize));
    _vpArrayMinYExprs[ac.idx()] = std::make_unique<Z3Expr>(_ctx.bv_const((ac.name() + "_mny").c_str(), _z3YBvSize));
    _vpArrayMaxYExprs[ac.idx()] = std::make_unique<Z3Expr>(_ctx.bv_const((ac.name() + "_mxy").c_str(), _z3YBvSize));
    
    const Z3Expr& mnx = *_vpArrayMinXExprs.at(ac.idx());
    const Z3Expr& mxx = *_vpArrayMaxXExprs.at(ac.idx());
    const Z3Expr& mny = *_vpArrayMinYExprs.at(ac.idx());
    const Z3Expr& mxy = *_vpArrayMaxYExprs.at(ac.idx());

    solver.add(0 <= mnx && 0 <= mxx && mnx <= mxx);
    solver.add(0 <= mny && 0 <= mxy && mny <= mxy);

    Z3ExprVector mnxEv(_ctx);
    Z3ExprVector mxxEv(_ctx);
    Z3ExprVector mnyEv(_ctx);
    Z3ExprVector mxyEv(_ctx);
    Int area = 0;
    for (const Cell* pCell : ac.vpCells()) {
      const Z3Expr& x = *_vpCellXExprs.at(pCell->idx());
      const Z3Expr& y = *_vpCellYExprs.at(pCell->idx());
      const Int gw = pCell->width() / _stepX;
      const Int gh = pCell->height() / _stepY;
      const Z3Expr& w = _ctx.bv_val(static_cast<unsigned>(gw), _z3XBvSize);
      const Z3Expr& h = _ctx.bv_val(static_cast<unsigned>(gh), _z3YBvSize);
      mnxEv.push_back(mnx == x);
      mxxEv.push_back(mxx == x + w);
      mnyEv.push_back(mny == y);
      mxyEv.push_back(mxy == y + h);
      solver.add(z3::ule(mnx, x));
      solver.add(z3::ule(x + w, mxx));
      solver.add(z3::ule(mny, y));
      solver.add(z3::ule(y + h, mxy));
      area += gw * gh;
    }
    solver.add(z3::mk_or(mnxEv));
    solver.add(z3::mk_or(mxxEv));
    solver.add(z3::mk_or(mnyEv));
    solver.add(z3::mk_or(mxyEv));

    // array dimension
    //std::cerr << ac.name() << std::endl;
    if (ac.useAutoSize()) { // auto determine dimensions
      Vector<Pair<Int, Int>> vFactors;
      calcFactors(area, vFactors);

      Z3ExprVector arrayShapeEv(_ctx);
      for (const auto& p : vFactors) {
        if (ac.hasCol() and ac.col() != p.first) {
          continue;
        }
        if (ac.hasRow() and ac.row() != p.second) {
          continue;
        }
        const Z3Expr& xSize = _ctx.bv_val(static_cast<unsigned>(p.first), _z3XBvSize);
        const Z3Expr& ySize = _ctx.bv_val(static_cast<unsigned>(p.second), _z3YBvSize);
        arrayShapeEv.push_back((mnx + xSize == mxx) && (mny + ySize == mxy));
        //arrayShapeEv.push_back((mnx + xSize == mxx) && (mny + ySize == mxy));
        solver.add(z3::bvadd_no_overflow(mnx, xSize, false));
        solver.add(z3::bvadd_no_overflow(mny, ySize, false));
        //std::cerr << p.first << " " << p.second << std::endl;
      }
      solver.add(z3::mk_or(arrayShapeEv));
    }
    else { // fixed col row
      const Z3Expr& xSize = _ctx.bv_val(static_cast<unsigned>(ac.col()), _z3XBvSize);
      const Z3Expr& ySize = _ctx.bv_val(static_cast<unsigned>(ac.row()), _z3YBvSize);
      solver.add((mnx + xSize == mxx) && (mny + ySize == mxy));
      //std::cerr << ac.col() << " " << ac.row() << std::endl;
    }
    //std::cerr << std::endl;


    // pattern
    switch (ac.pattern()) {
      case ArrayPatternE::id: {
        Vector<const Cell*> vpPartACells;
        for (Int j = 0; j < ac.numCells(); ++j) {
          if (ac.isPartA(j)) {
            vpPartACells.emplace_back(ac.pCell(j));
          }
        }
        for (size_t j = 0; j < vpPartACells.size(); ++j) {
          const Cell* pCj = vpPartACells.at(j);
          const Z3Expr& xj = *_vpCellXExprs.at(pCj->idx());
          const Z3Expr& wj = _ctx.bv_val(static_cast<unsigned>(pCj->width() / _stepX), _z3XBvSize);
          for (size_t k = 0; k < vpPartACells.size(); ++k) {
            if (k == j)
              continue;
            const Cell* pCk = vpPartACells.at(k);
            const Z3Expr& xk = *_vpCellXExprs.at(pCk->idx());
            const Z3Expr& wk = _ctx.bv_val(static_cast<unsigned>(pCk->width() / _stepX), _z3XBvSize);
            solver.add(z3::ite(xj > xk, z3::ult(xk + wk, xj), z3::ult(xj + wj, xk)));
          }
        }
        const Z3Expr& oneY = _ctx.bv_val(static_cast<unsigned>(1), _z3YBvSize);
        solver.add(mxy - mny == oneY);
        break;
      }
      case ArrayPatternE::cc: {
        Vector<const Cell*> vpPartACells, vpPartBCells;
        for (Int j = 0; j < ac.numCells(); ++j) {
          if (ac.isPartA(j)) {
            vpPartACells.emplace_back(ac.pCell(j));
          }
          else {
            vpPartBCells.emplace_back(ac.pCell(j));
          }
        }
        assert(vpPartACells.size() == vpPartBCells.size());
        assert(vpPartACells.size() > 0);

        const Int z3BvExtSize = calcLog2(vpPartACells.size()) + 1;
        const Int a0Idx = vpPartACells.at(0)->idx();
        const Int b0Idx = vpPartBCells.at(0)->idx();
        Z3Expr sumAX = z3::zext(*_vpCellXExprs.at(a0Idx), z3BvExtSize);
        Z3Expr sumBX = z3::zext(*_vpCellXExprs.at(b0Idx), z3BvExtSize);
        Z3Expr sumAY = z3::zext(*_vpCellYExprs.at(a0Idx), z3BvExtSize);
        Z3Expr sumBY = z3::zext(*_vpCellYExprs.at(b0Idx), z3BvExtSize);
        for (size_t j = 1; j < vpPartACells.size(); ++j) {
          const Int ajIdx = vpPartACells.at(j)->idx();
          const Int bjIdx = vpPartBCells.at(j)->idx();
          const Z3Expr& xa = z3::zext(*_vpCellXExprs.at(ajIdx), z3BvExtSize);
          const Z3Expr& xb = z3::zext(*_vpCellXExprs.at(bjIdx), z3BvExtSize);
          const Z3Expr& ya = z3::zext(*_vpCellYExprs.at(ajIdx), z3BvExtSize);
          const Z3Expr& yb = z3::zext(*_vpCellYExprs.at(bjIdx), z3BvExtSize);
          solver.add(z3::bvadd_no_overflow(sumAX, xa, false));
          solver.add(z3::bvadd_no_overflow(sumBX, xb, false));
          solver.add(z3::bvadd_no_overflow(sumAY, ya, false));
          solver.add(z3::bvadd_no_overflow(sumBY, yb, false));
          sumAX = sumAX + xa;
          sumBX = sumBX + xb;
          sumAY = sumAY + ya;
          sumBY = sumBY + yb;
        }
        solver.add(sumAX == sumBX && sumAY == sumBY);
        break;
      }
      case ArrayPatternE::cs: {
        Vector<const Cell*> vpPartACells;
        for (Int j = 0; j < ac.numCells(); ++j) {
          if (ac.isPartA(j)) {
            vpPartACells.emplace_back(ac.pCell(j));
          }
        }
        
        const Int z3BvExtSize = 1;
        const Z3Expr& mnxExt = z3::zext(mnx, z3BvExtSize);
        const Z3Expr& mxxExt = z3::zext(mxx, z3BvExtSize);
        const Z3Expr& mnyExt = z3::zext(mny, z3BvExtSize);
        const Z3Expr& mxyExt = z3::zext(mxy, z3BvExtSize);
        for (size_t j = 0; j < vpPartACells.size(); ++j) {
          const Cell* pCj = vpPartACells.at(j);
          const Z3Expr& xj = z3::zext(*_vpCellXExprs.at(pCj->idx()), z3BvExtSize);
          const Z3Expr& yj = z3::zext(*_vpCellYExprs.at(pCj->idx()), z3BvExtSize);
          const Z3Expr& wj = _ctx.bv_val(static_cast<unsigned>(pCj->width() / _stepX), _z3XBvSize + z3BvExtSize);
          const Z3Expr& hj = _ctx.bv_val(static_cast<unsigned>(pCj->height() / _stepY), _z3YBvSize + z3BvExtSize);
          Z3ExprVector ev(_ctx);
          for (size_t k = 0; k < vpPartACells.size(); ++k) {
            if (k == j)
              continue;
            const Cell* pCk = vpPartACells.at(k);
            const Z3Expr& xk = z3::zext(*_vpCellXExprs.at(pCk->idx()), z3BvExtSize);
            const Z3Expr& yk = z3::zext(*_vpCellYExprs.at(pCk->idx()), z3BvExtSize);
            ev.push_back((xj + wj + xk == mnxExt + mxxExt) &&
                         (yj + hj + yk == mnyExt + mxyExt));
          }
          solver.add(z3::mk_or(ev));
        }

        break;
      }
      case ArrayPatternE::undef:
        break;
      default:
        assert(false);
    }
  }
}

void SmtPlacer::addZ3CstrCellArraySize(Z3Solver& solver, const PlaceArrayCstr& ac, const Int xSize, const Int ySize)
{
  spdlog::info("[SmtPlacer] Add Z3 array size constraints");

  assert(_vpArrayMinXExprs.at(ac.idx()));
  assert(_vpArrayMaxXExprs.at(ac.idx()));
  assert(_vpArrayMinYExprs.at(ac.idx()));
  assert(_vpArrayMaxYExprs.at(ac.idx()));

  const Z3Expr& mnx = *_vpArrayMinXExprs.at(ac.idx());
  const Z3Expr& mxx = *_vpArrayMaxXExprs.at(ac.idx());
  const Z3Expr& mny = *_vpArrayMinYExprs.at(ac.idx());
  const Z3Expr& mxy = *_vpArrayMaxYExprs.at(ac.idx());
  
  solver.add(mxx - mnx == _ctx.bv_val(static_cast<unsigned>(xSize), _z3XBvSize));
  solver.add(mxy - mny == _ctx.bv_val(static_cast<unsigned>(ySize), _z3YBvSize));
}

void SmtPlacer::addZ3CstrCellSymmetry(Z3Solver& solver, const Region& reg)
{
  if (reg.numPlaceSymCstrs() == 0) {
    return;
  }

  spdlog::info("[SmtPlacer] Add Z3 cell symmetry constraints");

  const Z3Expr& xl = *_vpRegXExprs.at(reg.idx());
  const Z3Expr& yl = *_vpRegYExprs.at(reg.idx());
  const Z3Expr& w  = *_vpRegWExprs.at(reg.idx());
  const Z3Expr& h  = *_vpRegHExprs.at(reg.idx());

  Int i;
  const PlaceSymCstr* pCstr;
  Reg_ForEachPlaceSymCstr(reg, pCstr, i) {
    const PlaceSymCstr& sc = *pCstr;
    const SymAxisE axis = sc.axis();

    // check validity
    assert(sc.numCells() > 0);
    bool isSelfSymSizeOdd = false;
    if (sc.hasSelfSym()) {
      for (Int j = 0; j < sc.numCells(); ++j) {
        if (sc.isSelfSym(j)) {
          switch (axis) {
            case SymAxisE::ver:
            case SymAxisE::either:
              isSelfSymSizeOdd = (sc.cell(j).width() / _stepX) % 2; 
              break;
            case SymAxisE::hor:
              isSelfSymSizeOdd = (sc.cell(j).height() / _stepY) % 2; 
              break;
            default: assert(false);
          }
          break;
        }
      }
      for (Int j = 0; j < sc.numCells(); ++j) {
        bool b;
        if (sc.isSelfSym(j)) {
          switch (axis) {
            case SymAxisE::ver:
            case SymAxisE::either:
              b = (sc.cell(j).width() / _stepX) % 2; 
              break;
            case SymAxisE::hor:
              b = (sc.cell(j).height() / _stepY) % 2; 
              break;
            default: assert(false);
          }
          if (isSelfSymSizeOdd != b) {
            spdlog::error("[SmtPlacer] Symmetry constraint {} unsat (self-sym cells size violation)", sc.name());
            exit(0);
          }
        }
      }
    }

    const Int z3BvSize = axis == SymAxisE::hor ? _z3YBvSize : _z3XBvSize;
    _vpSymAxisExprs[sc.idx()] = std::make_unique<Z3Expr>(_ctx.bv_const((sc.name() + "_ax").c_str(), z3BvSize));
    const Z3Expr& ax = *_vpSymAxisExprs.at(sc.idx());

    switch (axis) {
      case SymAxisE::ver:
      case SymAxisE::either:
        solver.add(z3::ule(xl, ax));
        solver.add(z3::ule(ax, xl + w));
        break;
      case SymAxisE::hor:
        solver.add(z3::ule(yl, ax));
        solver.add(z3::ule(ax, yl + h));
        break;
      default: assert(false);
    }

    for (Int j = 0; j < sc.numCells(); ++j) {
      const Cell& ca = sc.cell(j);
      const Z3Expr& xa = *_vpCellXExprs.at(ca.idx());
      const Z3Expr& ya = *_vpCellYExprs.at(ca.idx());
      const Z3Expr& wa = _ctx.bv_val(static_cast<unsigned>(ca.width() / _stepX), _z3XBvSize);
      const Z3Expr& ha = _ctx.bv_val(static_cast<unsigned>(ca.height() / _stepY), _z3XBvSize);
      const SymPartE sp = sc.symPart(j);
      if (sp == SymPartE::a) {
        const Cell& cb = sc.symCell(j);
        assert(sc.isPartB(cb));
        const Z3Expr& xb = *_vpCellXExprs.at(cb.idx());
        const Z3Expr& yb = *_vpCellYExprs.at(cb.idx());
        const Z3Expr& wb = _ctx.bv_val(static_cast<unsigned>(cb.width() / _stepX), _z3XBvSize);
        const Z3Expr& hb = _ctx.bv_val(static_cast<unsigned>(cb.height() / _stepY), _z3XBvSize);
        //solver.add(wa == wb);
        //solver.add(ha == hb);
        switch (axis) {
          case SymAxisE::ver:
          case SymAxisE::either:
            solver.add(ya == yb);
            if (sc.hasSelfSym()) {
              if (isSelfSymSizeOdd) {
                solver.add(xa + wa + xb == 2 * ax + 1);
                const Z3Expr& one = _ctx.bv_val(static_cast<unsigned>(1), _z3XBvSize);
                const Z3Expr& two = _ctx.bv_val(static_cast<unsigned>(2), _z3XBvSize);
                solver.add(z3::bvadd_no_overflow(xa, xb, false));
                solver.add(z3::bvadd_no_overflow(xa + wa, xb, false));
                solver.add(z3::bvmul_no_overflow(two, ax, false));
                solver.add(z3::bvadd_no_overflow(two * ax, one, false));
              }
              else {
                solver.add(xa + wa + xb == 2 * ax);
                const Z3Expr& two = _ctx.bv_val(static_cast<unsigned>(2), _z3XBvSize);
                solver.add(z3::bvadd_no_overflow(xa + wa, xb, false));
                solver.add(z3::bvmul_no_overflow(two, ax, false));
              }
            }
            else {
              solver.add(xa + wa + xb == 2 * ax);
              const Z3Expr& two = _ctx.bv_val(static_cast<unsigned>(2), _z3XBvSize);
              solver.add(z3::bvadd_no_overflow(xa, wa, false));
              solver.add(z3::bvadd_no_overflow(xa + wa, xb, false));
              solver.add(z3::bvmul_no_overflow(two, ax, false));
              //solver.add(2 * xa + wa + 2 * xb + wb == 4 * ax);
              //const Z3Expr& two = _ctx.bv_val(static_cast<unsigned>(2), _z3XBvSize);
              //const Z3Expr& four = _ctx.bv_val(static_cast<unsigned>(4), _z3XBvSize);
              //solver.add(z3::bvmul_no_overflow(two, xa, false));
              //solver.add(z3::bvmul_no_overflow(two, xb, false));
              //solver.add(z3::bvmul_no_overflow(four, ax, false));
              //solver.add(z3::bvadd_no_overflow(two * xa, wa, false));
              //solver.add(z3::bvadd_no_overflow(two * xa + wa, two * wb, false));
              //solver.add(z3::bvadd_no_overflow(two * xa + wa + (two * xb), wb, false));
            }
            solver.add(z3::ult(xa, xb));
            //solver.add(xb - xa != 1);
            break;
          case SymAxisE::hor:
            solver.add(xa == xb);
            if (sc.hasSelfSym()) {
              if (isSelfSymSizeOdd) {
                solver.add(ya + ha + yb == 2 * ax + 1);
                const Z3Expr& one = _ctx.bv_val(static_cast<unsigned>(1), _z3YBvSize);
                const Z3Expr& two = _ctx.bv_val(static_cast<unsigned>(2), _z3YBvSize);
                solver.add(z3::bvadd_no_overflow(ya, yb, false));
                solver.add(z3::bvadd_no_overflow(ya + hb, yb, false));
                solver.add(z3::bvmul_no_overflow(two, ax, false));
                solver.add(z3::bvmul_no_overflow(two * ax, one, false));
              }
              else {
                solver.add(ya + ha + yb == 2 * ax);
                const Z3Expr& two = _ctx.bv_val(static_cast<unsigned>(2), _z3YBvSize);
                solver.add(z3::bvadd_no_overflow(ya, yb, false));
                solver.add(z3::bvadd_no_overflow(ya + hb, yb, false));
                solver.add(z3::bvmul_no_overflow(two, ax, false));
              }
            }
            else {
              solver.add(ya + ha + yb == 2 * ax);
              const Z3Expr& two = _ctx.bv_val(static_cast<unsigned>(2), _z3XBvSize);
              solver.add(z3::bvadd_no_overflow(ya, ha, false));
              solver.add(z3::bvadd_no_overflow(ya + ha, yb, false));
              solver.add(z3::bvmul_no_overflow(two, ax, false));
              //solver.add(2 * ya + ha + 2 * yb + hb == 4 * ax);
              //const Z3Expr& two = _ctx.bv_val(static_cast<unsigned>(2), _z3YBvSize);
              //const Z3Expr& four = _ctx.bv_val(static_cast<unsigned>(4), _z3YBvSize);
              //solver.add(z3::bvmul_no_overflow(two, ya, false));
              //solver.add(z3::bvmul_no_overflow(two, yb, false));
              //solver.add(z3::bvmul_no_overflow(four, ax, false));
              //solver.add(z3::bvadd_no_overflow(two * ya, ha, false));
              //solver.add(z3::bvadd_no_overflow(two * ya + ha, two * yb, false));
              //solver.add(z3::bvadd_no_overflow(two * ya + ha + (two * yb), hb, false));
            }
            solver.add(z3::ult(ya, yb));
            solver.add((yb - ya) % 2 == 1);
            break;
          default: assert(false);
        }
      }
      else if (sp == SymPartE::self) {
        switch (axis) {
          case SymAxisE::ver:
          case SymAxisE::either:
            solver.add(xa + wa / 2 == ax);
            //solver.add(xa == ax);
            break;
          case SymAxisE::hor:
            solver.add(ya + ha / 2 == ax);
            //solver.add(ya == ax);
            break;
          default: assert(false);
        }
      }
    }
  }
  // extra proximity
  //for (Int i = 0; i < reg.numPlaceSymCstrs(); ++i) {
    //const PlaceSymCstr& sc = reg.placeSymCstr(i);
    //for (Int j = 0; j < sc.numCells(); ++j) {
      //Z3ExprVector ev(_ctx);
      //const Cell& cj = sc.cell(j);
      //const Z3Expr& xj = *_vpCellXExprs.at(cj.idx());
      //const Z3Expr& yj = *_vpCellYExprs.at(cj.idx());
      //const Int wj = cj.width() / _stepX;
      //const Int hj = cj.height() / _stepY;
      //for (Int k = 0; k < sc.numCells(); ++k) {
        //if (j == k)
          //continue;
        //const Cell& ck = sc.cell(k);
        //const Z3Expr& xk = *_vpCellXExprs.at(ck.idx());
        //const Z3Expr& yk = *_vpCellYExprs.at(ck.idx());
        //const Int wk = ck.width() / _stepX;
        //const Int hk = ck.height() / _stepY;
        //ev.push_back(((yj == yk) && ((xj + wj == xk) || (xk + wk == xj))) ||
                     //((xj == xk) && ((yj + hj == yk) || (yk + hk == yj))));
      //}
      //solver.add(z3::mk_or(ev));
    //}

  //}
}


void SmtPlacer::addZ3CstrCellCluster(Z3Solver& solver)
{
  if (_cir.numPlaceClusterCstrs() == 0) {
    return;
  }

  spdlog::info("[SmtPlacer] Add Z3 cell cluster constraints");

  Int i;
  const PlaceClusterCstr* pCstr;
  Cir_ForEachPlaceClusterCstr(_cir, pCstr, i) {
    const PlaceClusterCstr& clus = *pCstr;
    _vpClusterMinXExprs[clus.idx()] = std::make_unique<Z3Expr>(_ctx.bv_const((clus.name() + "_mnx").c_str(), _z3XBvSize));
    _vpClusterMaxXExprs[clus.idx()] = std::make_unique<Z3Expr>(_ctx.bv_const((clus.name() + "_mxx").c_str(), _z3XBvSize));
    _vpClusterMinYExprs[clus.idx()] = std::make_unique<Z3Expr>(_ctx.bv_const((clus.name() + "_mny").c_str(), _z3YBvSize));
    _vpClusterMaxYExprs[clus.idx()] = std::make_unique<Z3Expr>(_ctx.bv_const((clus.name() + "_mxy").c_str(), _z3YBvSize));

    const Z3Expr& mnx = *_vpClusterMinXExprs.at(clus.idx());
    const Z3Expr& mxx = *_vpClusterMaxXExprs.at(clus.idx());
    const Z3Expr& mny = *_vpClusterMinYExprs.at(clus.idx());
    const Z3Expr& mxy = *_vpClusterMaxYExprs.at(clus.idx());

    Z3ExprVector mnxEv(_ctx);
    Z3ExprVector mxxEv(_ctx);
    Z3ExprVector mnyEv(_ctx);
    Z3ExprVector mxyEv(_ctx);

    for (const Cell* pCell : clus.vpCells()) {
      const Z3Expr& x = *_vpCellXExprs.at(pCell->idx());
      const Z3Expr& y = *_vpCellYExprs.at(pCell->idx());
      const Z3Expr& w = _ctx.bv_val(static_cast<unsigned>(pCell->width() / _stepX), _z3XBvSize);
      const Z3Expr& h = _ctx.bv_val(static_cast<unsigned>(pCell->height() / _stepY), _z3YBvSize);
      mnxEv.push_back(mnx == x);
      mxxEv.push_back(mxx == x + w);
      mnyEv.push_back(mny == y);
      mxyEv.push_back(mxy == y + h);
      solver.add(z3::ule(mnx, x));
      solver.add(z3::ule(x + w, mxx));
      solver.add(z3::ule(mny, y));
      solver.add(z3::ule(y + h, mxy));
    }
    solver.add(z3::mk_or(mnxEv));
    solver.add(z3::mk_or(mxxEv));
    solver.add(z3::mk_or(mnyEv));
    solver.add(z3::mk_or(mxyEv));
  }
  Cir_ForEachPlaceClusterCstr(_cir, pCstr, i) {
    const PlaceClusterCstr& clus = *pCstr;
    const Z3Expr& mnx = z3::zext(*_vpClusterMinXExprs.at(clus.idx()), _z3HpwlBvSize - _z3XBvSize);
    const Z3Expr& mxx = z3::zext(*_vpClusterMaxXExprs.at(clus.idx()), _z3HpwlBvSize - _z3XBvSize);
    const Z3Expr& mny = z3::zext(*_vpClusterMinYExprs.at(clus.idx()), _z3HpwlBvSize - _z3YBvSize);
    const Z3Expr& mxy = z3::zext(*_vpClusterMaxYExprs.at(clus.idx()), _z3HpwlBvSize - _z3YBvSize);
    const Z3Expr& weight = _ctx.bv_val(static_cast<unsigned>(clus.weight()), _z3HpwlBvSize);
    if (_pTotHpwlExpr) {
      Z3Expr& totHpwlExpr = *_pTotHpwlExpr;
      totHpwlExpr = totHpwlExpr + weight * (mxx - mnx + mxy - mny);
      solver.add(z3::bvmul_no_overflow(weight, (mxx - mnx + mxy - mny), false));
      solver.add(z3::bvadd_no_overflow(totHpwlExpr, weight * (mxx - mnx + mxy - mny), false));
    }
    else {
      _pTotHpwlExpr = std::make_unique<Z3Expr>(weight * (mxx - mnx + mxy - mny));
      solver.add(z3::bvmul_no_overflow(weight, (mxx - mnx + mxy - mny), false));
    }
  }
}


void SmtPlacer::addZ3CstrCellPowerAbutment(Z3Solver& solver, const Region& reg)
{
  if (reg.numCellGroups() < 2) {
    return;
  }
  spdlog::info("[SmtPlacer] Add Z3 cell power abutment constraints");
  const Z3Expr& yl = *_vpRegYExprs.at(reg.idx());
  const Z3Expr& h  = *_vpRegHExprs.at(reg.idx());

  _vvpCellGroupYBoundExprs.resize(reg.numCellGroups());
  for (Int i = 0; i < reg.numCellGroups(); ++i) {
    auto& vpCellsYBoundExprs = _vvpCellGroupYBoundExprs.at(i);
    vpCellsYBoundExprs.emplace_back(std::make_unique<Z3Expr>(_ctx.bv_const(("g" + std::to_string(i) + "_yl").c_str(), _z3YBvSize)));
    vpCellsYBoundExprs.emplace_back(std::make_unique<Z3Expr>(_ctx.bv_const(("g" + std::to_string(i) + "_yh").c_str(), _z3YBvSize)));
  }
  for (size_t i = 1; i < _vvpCellGroupYBoundExprs.size(); ++i) {
    const auto& pre = _vvpCellGroupYBoundExprs.at(i - 1);
    const auto& cur = _vvpCellGroupYBoundExprs.at(i);
    const Z3Expr& yl0 = *pre.at(0);
    const Z3Expr& yh0 = *pre.at(1);
    const Z3Expr& yl1 = *cur.at(0);
    const Z3Expr& yh1 = *cur.at(1);
    const Z3Expr& one = _ctx.bv_val(static_cast<unsigned>(0x1), _z3YBvSize);
    const Z3Expr& zero = _ctx.bv_val(static_cast<unsigned>(0x0), _z3YBvSize);
    if (i == 1) {
      solver.add((yl0 & one) == zero && (yh0 & one) != zero);
      solver.add(z3::ule(yl, yl0));
    }
    solver.add((yl1 & one) == zero && (yh1 & one) != zero);
    solver.add(z3::ule(yl0, yh0));
    solver.add(z3::ule(yl1, yh1));
    solver.add(z3::ult(yh0, yl1));
    if (i == _vvpCellGroupYBoundExprs.size() - 1) {
      solver.add(z3::ult(yh1, yl + h));
    }
  }
  for (Int i = 0; i < reg.numCellGroups(); ++i) {
    const auto& vpCells = reg.vpCellGroup(i);
    const auto& vpCellYBoundExprs = _vvpCellGroupYBoundExprs.at(i);
    //std::cerr << "G: " << std::endl;
    for (const Cell* pCell : vpCells) {
      //std::cerr << pCell->name() << std::endl;
      const Z3Expr& y = *_vpCellYExprs.at(pCell->idx());
      const Z3Expr& yl0 = *vpCellYBoundExprs.at(0);
      const Z3Expr& yh0 = *vpCellYBoundExprs.at(1);
      solver.add(z3::ule(yl0, y) && z3::ule(y, yh0));
    }
  }
}

void SmtPlacer::addZ3CstrCellPrePlace(Z3Solver& solver)
{
  if (_cir.numPrePlaceCstrs() == 0) {
    return;
  }
  spdlog::info("[SmtPlacer] Add Z3 cell pre-place constraints");
  
  Int i;
  const PrePlaceCstr* pCstr;
  Cir_ForEachPrePlaceCstr(_cir, pCstr, i) {
    const PrePlaceCstr& pre = *pCstr;
    for (Int j = 0; j < pre.numRegions(); ++j) {
      const Region& reg = pre.region(j);
      if (pre.hasRegLoc(j)) {
        const Z3Expr& xExpr = *_vpRegXExprs.at(reg.idx());
        const Z3Expr& yExpr = *_vpRegYExprs.at(reg.idx());

        const Point<Int>& loc = pre.regLoc(j);
        const Z3Expr& x = _ctx.bv_val(static_cast<unsigned>(loc.x() / _stepX), _z3XBvSize);
        const Z3Expr& y = _ctx.bv_val(static_cast<unsigned>(loc.y() / _stepY), _z3YBvSize);
        solver.add(xExpr == x && yExpr == y);
      }
    }
    for (Int j = 0; j < pre.numCells(); ++j) {
      const Cell& c = pre.cell(j);
      if (pre.hasCellLoc(j)) {
        const Z3Expr& xExpr = *_vpCellXExprs.at(c.idx());
        const Z3Expr& yExpr = *_vpCellYExprs.at(c.idx());

        const Point<Int>& loc = pre.cellLoc(j);
        const Z3Expr& x = _ctx.bv_val(static_cast<unsigned>(loc.x() / _stepX), _z3XBvSize);
        const Z3Expr& y = _ctx.bv_val(static_cast<unsigned>(loc.y() / _stepY), _z3YBvSize);
        solver.add(xExpr == x && yExpr == y);
      }
    }
  }

}

void SmtPlacer::addZ3CstrCellEdgeDist(Z3Solver& solver, const Region& reg)
{
  if (_cir.numPlaceEdgeDistCstrs() == 0) {
    return;
  }
  spdlog::info("[SmtPlacer] Add Z3 cell edge distance constraints");

  const Z3Expr& regXExpr = *_vpRegXExprs.at(reg.idx());
  const Z3Expr& regYExpr = *_vpRegYExprs.at(reg.idx());
  const Z3Expr& regWExpr = *_vpRegWExprs.at(reg.idx());
  const Z3Expr& regHExpr = *_vpRegHExprs.at(reg.idx());

  Int i;
  const PlaceEdgeDistCstr* pCstr;
  Reg_ForEachPlaceEdgeDistCstr(_cir, pCstr, i) {
    const PlaceEdgeDistCstr& ed = *pCstr;

    if (ed.distL() > 0) {
      const Z3Expr& d = _ctx.bv_val(static_cast<unsigned>(ed.distL()), _z3XBvSize);
      for (const Cell* pCell : ed.vpCells()) {
        const Z3Expr& xExpr = *_vpCellXExprs.at(pCell->idx());
        solver.add(z3::ule(regXExpr + d, xExpr));
      }
    }
    if (ed.distR() > 0) {
      const Z3Expr& d = _ctx.bv_val(static_cast<unsigned>(ed.distR()), _z3XBvSize);
      for (const Cell* pCell : ed.vpCells()) {
        const Z3Expr& xExpr = *_vpCellXExprs.at(pCell->idx());
        const Z3Expr& wExpr = _ctx.bv_val(static_cast<unsigned>(pCell->width() / _stepX), _z3XBvSize);
        solver.add(z3::ule(xExpr + wExpr + d, regXExpr + regWExpr));
      }
    }
    if (ed.distB() > 0) {
      const Z3Expr& d = _ctx.bv_val(static_cast<unsigned>(ed.distB()), _z3YBvSize);
      for (const Cell* pCell : ed.vpCells()) {
        const Z3Expr& yExpr = *_vpCellYExprs.at(pCell->idx());
        solver.add(z3::ule(regYExpr + d, yExpr));
      }
    }
    if (ed.distT() > 0) {
      const Z3Expr& d = _ctx.bv_val(static_cast<unsigned>(ed.distT()), _z3YBvSize);
      for (const Cell* pCell : ed.vpCells()) {
        const Z3Expr& yExpr = *_vpCellYExprs.at(pCell->idx());
        const Z3Expr& hExpr = _ctx.bv_val(static_cast<unsigned>(pCell->height() / _stepY), _z3YBvSize);
        solver.add(z3::ule(yExpr + hExpr + d, regYExpr + regHExpr));
      }
    }


  }
}

void SmtPlacer::addZ3CstrCellOrder(Z3Solver& solver)
{
  if (_cir.numPlaceOrderCstrs() == 0) {
    return;
  }

  spdlog::info("[SmtPlacer] Add Z3 cell order constraints");

  Int i;
  const PlaceOrderCstr* pCstr;

  Cir_ForEachPlaceOrderCstr(_cir, pCstr, i) {
    const PlaceOrderCstr& oc = *pCstr;

    const bool isStrict = oc.isStrict();
    
    switch (oc.dir()) {
      case Direction2dE::left:
        for (Int j = 1; j < oc.numCellGroups(); ++j) {
          const Vector<Cell*>& preCg = oc.vpCells(j - 1);
          const Vector<Cell*>& curCg = oc.vpCells(j);
          for (const Cell* pre : preCg) {
            const Z3Expr& xPre = *_vpCellXExprs.at(pre->idx());
            for (const Cell* cur : curCg) {
              const Z3Expr& xCur = *_vpCellXExprs.at(cur->idx());
              if (isStrict) {
                const Z3Expr& wCur = _ctx.bv_val(static_cast<unsigned>(cur->width() / _stepX), _z3XBvSize);
                solver.add(z3::ule(xCur + wCur, xPre));
              }
              else {
                solver.add(z3::ule(xCur, xPre));
              }
            }
          }
        }
        break;
      case Direction2dE::right:
        for (Int j = 1; j < oc.numCellGroups(); ++j) {
          const Vector<Cell*>& preCg = oc.vpCells(j - 1);
          const Vector<Cell*>& curCg = oc.vpCells(j);
          for (const Cell* pre : preCg) {
            const Z3Expr& xPre = *_vpCellXExprs.at(pre->idx());
            const Z3Expr& wPre = _ctx.bv_val(static_cast<unsigned>(pre->width() / _stepX), _z3XBvSize);
            for (const Cell* cur : curCg) {
              const Z3Expr& xCur = *_vpCellXExprs.at(cur->idx());
              if (isStrict) {
                solver.add(z3::ule(xPre + wPre, xCur));
              }
              else {
                solver.add(z3::ule(xPre, xCur));
              }
            }
          }
        }
        break;
      case Direction2dE::down:
        for (Int j = 1; j < oc.numCellGroups(); ++j) {
          const Vector<Cell*>& preCg = oc.vpCells(j - 1);
          const Vector<Cell*>& curCg = oc.vpCells(j);
          for (const Cell* pre : preCg) {
            const Z3Expr& yPre = *_vpCellYExprs.at(pre->idx());
            for (const Cell* cur : curCg) {
              const Z3Expr& yCur = *_vpCellYExprs.at(cur->idx());
              if (isStrict) {
                const Z3Expr& hCur = _ctx.bv_val(static_cast<unsigned>(cur->height() / _stepY), _z3YBvSize);
                solver.add(z3::ule(yCur + hCur, yPre));
              }
              else {
                solver.add(z3::ule(yCur, yPre));
              }
            }
          }
        }
        break;
      case Direction2dE::up:
        for (Int j = 1; j < oc.numCellGroups(); ++j) {
          const Vector<Cell*>& preCg = oc.vpCells(j - 1);
          const Vector<Cell*>& curCg = oc.vpCells(j);
          for (const Cell* pre : preCg) {
            const Z3Expr& yPre = *_vpCellYExprs.at(pre->idx());
            const Z3Expr& hPre = _ctx.bv_val(static_cast<unsigned>(pre->height() / _stepY), _z3YBvSize);
            for (const Cell* cur : curCg) {
              const Z3Expr& yCur = *_vpCellYExprs.at(cur->idx());
              if (isStrict) {
                solver.add(z3::ule(yPre + hPre, yCur));
              }
              else {
                solver.add(z3::ule(yPre, yCur));
              }
            }
          }
        }
        break;
      default:
        assert(false);
    }
  }

}

void SmtPlacer::addZ3CstrCellAlign(Z3Solver& solver)
{
  if (_cir.numPlaceAlignCstrs() == 0) {
    return;
  }
  
  Int i;
  const PlaceAlignCstr* pCstr;
  Vector<Int> vCstrIds;
  Cir_ForEachPlaceAlignCstr(_cir, pCstr, i) {
    if (pCstr->numCells() > 0 or pCstr->numPlaceArrayCstrs() > 0) {
      assert(pCstr->numRegions() == 0);
      vCstrIds.emplace_back(pCstr->idx());
    }
  }

  if (vCstrIds.empty()) {
    return;
  }

  spdlog::info("[SmtPlacer] Add Z3 cell alignment constraints");

  for (const Int idx : vCstrIds) {
    const PlaceAlignCstr& ac = _cir.placeAlignCstr(idx);
    //std::cerr << ac.name() << std::endl;
    assert(ac.numCells() + ac.numPlaceArrayCstrs() > 1);

    if (ac.isHor()) {
      if (ac.isLow()) { // bottom alignment
        const Z3Expr* pYc = ac.numCells() > 0 ? _vpCellYExprs.at(ac.cell(0).idx()).get() : nullptr;
        const Z3Expr* pYa = ac.numPlaceArrayCstrs() > 0 ? _vpArrayMinYExprs.at(ac.placeArrayCstr(0).idx()).get() : nullptr;
        for (Int j = 1; j < ac.numCells(); ++j) {
          const Z3Expr& yCur = *_vpCellYExprs.at(ac.cell(j).idx());
          solver.add(*pYc == yCur);
        }
        for (Int j = 1; j < ac.numPlaceArrayCstrs(); ++j) {
          const Z3Expr& yCur = *_vpArrayMinYExprs.at(ac.placeArrayCstr(j).idx());
          solver.add(*pYa == yCur);
        }
        if (pYc and pYa) {
          solver.add(*pYc == *pYa);
        }
      }
      else { // top alignment
        const Cell* pC = ac.numCells() > 0 ? &ac.cell(0) : nullptr;
        const Z3Expr* pYc = pC ? _vpCellYExprs.at(pC->idx()).get() : nullptr;
        const Z3Expr* pYa = ac.numPlaceArrayCstrs() > 0 ? _vpArrayMaxYExprs.at(ac.placeArrayCstr(0).idx()).get() : nullptr;
        const Z3Expr& h =  pC ? _ctx.bv_val(static_cast<unsigned>(pC->height() / _stepY), _z3YBvSize) : _ctx.bv_val(static_cast<unsigned>(0), _z3YBvSize);
        for (Int j = 1; j < ac.numCells(); ++j) {
          const Cell& cCur = ac.cell(j);
          const Z3Expr& yCur = *_vpCellYExprs.at(cCur.idx());
          const Z3Expr& hCur = _ctx.bv_val(static_cast<unsigned>(cCur.height() / _stepY), _z3YBvSize);
          solver.add((*pYc + h) == (yCur + hCur));
        }
        for (Int j = 1; j < ac.numPlaceArrayCstrs(); ++j) {
          const Z3Expr& yCur = *_vpArrayMaxYExprs.at(ac.placeArrayCstr(j).idx());
          solver.add(*pYa == yCur);
        }
        if (pYc and pYa) {
          solver.add((*pYc + h) == *pYa);
        }
      }
    }
    else {
      assert(ac.isVer());
      if (ac.isLow()) { // left alignment
        const Z3Expr* pXc = ac.numCells() > 0 ? _vpCellXExprs.at(ac.cell(0).idx()).get() : nullptr;
        const Z3Expr* pXa = ac.numPlaceArrayCstrs() > 0 ? _vpArrayMinXExprs.at(ac.placeArrayCstr(0).idx()).get() : nullptr;
        for (Int j = 1; j < ac.numCells(); ++j) {
          const Z3Expr& xCur = *_vpCellXExprs.at(ac.cell(j).idx());
          solver.add(*pXc == xCur);
        }
        for (Int j = 1; j < ac.numPlaceArrayCstrs(); ++j) {
          const Z3Expr& xCur = *_vpArrayMinXExprs.at(ac.placeArrayCstr(j).idx());
          solver.add(*pXa == xCur);
        }
        if (pXc and pXa) {
          solver.add(*pXc == *pXa);
        }
      }
      else { // right alignment
        const Cell* pC = ac.numCells() > 0 ? &ac.cell(0) : nullptr;
        const Z3Expr* pXc = pC ? _vpCellXExprs.at(pC->idx()).get() : nullptr;
        const Z3Expr* pXa = ac.numPlaceArrayCstrs() > 0 ? _vpArrayMaxXExprs.at(ac.placeArrayCstr(0).idx()).get() : nullptr;
        const Z3Expr& w = pC ? _ctx.bv_val(static_cast<unsigned>(pC->width() / _stepX), _z3XBvSize) : _ctx.bv_val(static_cast<unsigned>(0), _z3XBvSize);
        for (Int j = 1; j < ac.numCells(); ++j) {
          const Cell& cCur = ac.cell(j);
          const Z3Expr& xCur = *_vpCellXExprs.at(cCur.idx());
          const Z3Expr& wCur = _ctx.bv_val(static_cast<unsigned>(cCur.width() / _stepX), _z3XBvSize);
          solver.add((*pXc + w) == (xCur + wCur));
        }
        for (Int j = 1; j < ac.numPlaceArrayCstrs(); ++j) {
          const Z3Expr& xCur = *_vpArrayMaxXExprs.at(ac.placeArrayCstr(j).idx());
          solver.add(*pXa == xCur);
        }
        if (pXc and pXa) {
          solver.add((*pXc + w) == *pXa);
        }
      }
    }
  }
}

void SmtPlacer::addZ3CstrCellDisjoint(Z3Solver& solver) 
{
  if (_cir.numPlaceDisjointCstrs() == 0) {
    return;
  }

  spdlog::info("[SmtPlacer] Add Z3 cell disjoint constraints");

  Int i;
  const PlaceDisjointCstr* pCstr;
  Cir_ForEachPlaceDisjointCstr(_cir, pCstr, i) {
    const PlaceDisjointCstr& dc = *pCstr;
    if (dc.isRow()) {
      for (Int j = 0; j < dc.numCells() - 1; ++j) {
        const Z3Expr& yj = *_vpCellYExprs.at(dc.cell(j).idx());
        for (Int k = j + 1; k < dc.numCells(); ++k) {
          const Z3Expr& yk = *_vpCellYExprs.at(dc.cell(k).idx());
          solver.add(yj != yk);
        }
      }
    }
    else {
      assert(dc.isCol());
      for (Int j = 0; j < dc.numCells() - 1; ++j) {
        const Z3Expr& xj = *_vpCellXExprs.at(dc.cell(j).idx());
        for (Int k = j + 1; k < dc.numCells(); ++k) {
          const Z3Expr& xk = *_vpCellXExprs.at(dc.cell(k).idx());
          solver.add(xj != xk);
        }
      }
    }
  }
}

void SmtPlacer::addZ3CstrCellRow(Z3Solver& solver)
{
  if (_cir.numPlaceRowCstrs() == 0) {
    return;
  }
  
  spdlog::info("[SmtPlacer] Add Z3 cell row constraints");
  
  const Z3Expr& zero = _ctx.bv_val(static_cast<unsigned>(0x0), _z3YBvSize);
  const Z3Expr& one = _ctx.bv_val(static_cast<unsigned>(0x1), _z3YBvSize);

  Int i;
  const PlaceRowCstr* pCstr;
  Cir_ForEachPlaceRowCstr(_cir, pCstr, i) {
    const PlaceRowCstr& rc = *pCstr;
    if (rc.isOdd()) {
      for (Int j = 0; j < rc.numCells(); ++j) {
        const Cell& c = rc.cell(j);
        const Z3Expr& yj = *_vpCellYExprs.at(c.idx());
        const Z3Expr& yr = *_vpRegYExprs.at(c.region().idx());
        solver.add(((yj - yr) & one) == zero);
      }
    }
    else {
      assert(rc.isEven());
      for (Int j = 0; j < rc.numCells(); ++j) {
        const Cell& c = rc.cell(j);
        const Z3Expr& yj = *_vpCellYExprs.at(c.idx());
        const Z3Expr& yr = *_vpRegYExprs.at(c.region().idx());
        solver.add(((yj - yr) & one) == one);
      }
    }
  }

}

void SmtPlacer::addZ3CstrArrayArray(Z3Solver& solver, const Region& reg)
{
  Int i;
  const PlaceArrayCstr* pCstr;
  
  Vector<Int> vPlaceArrayCstrIds;
  Reg_ForEachPlaceArrayCstr(reg, pCstr, i) {
    if (pCstr->numPlaceArrayCstrs() > 0) {
      assert(pCstr->numCells() == 0);
      vPlaceArrayCstrIds.emplace_back(pCstr->idx());
    }
  }

  if (vPlaceArrayCstrIds.size() == 0) {
    return;
  }

  spdlog::info("[SmtPlacer] Add Z3 array of arrays constraints"); 

  for (const Int idx : vPlaceArrayCstrIds) {
    const PlaceArrayCstr& ac = _cir.placeArrayCstr(idx);
    //std::cerr << ac.idx() << " " << idx << std::endl;
    assert(ac.numPlaceArrayCstrs() > 0);

    _vpArrayMinXExprs[ac.idx()] = std::make_unique<Z3Expr>(_ctx.bv_const((ac.name() + "_mnx").c_str(), _z3XBvSize));
    _vpArrayMaxXExprs[ac.idx()] = std::make_unique<Z3Expr>(_ctx.bv_const((ac.name() + "_mxx").c_str(), _z3XBvSize));
    _vpArrayMinYExprs[ac.idx()] = std::make_unique<Z3Expr>(_ctx.bv_const((ac.name() + "_mny").c_str(), _z3YBvSize));
    _vpArrayMaxYExprs[ac.idx()] = std::make_unique<Z3Expr>(_ctx.bv_const((ac.name() + "_mxy").c_str(), _z3YBvSize));

    const Z3Expr& mnx = *_vpArrayMinXExprs.at(ac.idx());
    const Z3Expr& mxx = *_vpArrayMaxXExprs.at(ac.idx());
    const Z3Expr& mny = *_vpArrayMinYExprs.at(ac.idx());
    const Z3Expr& mxy = *_vpArrayMaxYExprs.at(ac.idx());

    Z3ExprVector mnxEv(_ctx);
    Z3ExprVector mxxEv(_ctx);
    Z3ExprVector mnyEv(_ctx);
    Z3ExprVector mxyEv(_ctx);
    
    // boundary
    Int area = 0;
    for (const PlaceArrayCstr* pArray : ac.vpPlaceArrayCstrs()) {
      //std::cerr << pArray->name() << " " << pArray->idx() << std::endl;
      const Z3Expr& arrMnx = *_vpArrayMinXExprs.at(pArray->idx());
      const Z3Expr& arrMxx = *_vpArrayMaxXExprs.at(pArray->idx());
      const Z3Expr& arrMny = *_vpArrayMinYExprs.at(pArray->idx());
      const Z3Expr& arrMxy = *_vpArrayMaxYExprs.at(pArray->idx());
      mnxEv.push_back(mnx == arrMnx);
      mxxEv.push_back(mxx == arrMxx);
      mnyEv.push_back(mny == arrMny);
      mxyEv.push_back(mxy == arrMxy);
      solver.add(z3::ule(mnx, arrMnx));
      solver.add(z3::ule(arrMxx, mxx));
      solver.add(z3::ule(mny, arrMny));
      solver.add(z3::ule(arrMxy, mxy));
      area += calcArrayGridArea(*pArray);
    }
    solver.add(z3::mk_or(mnxEv));
    solver.add(z3::mk_or(mxxEv));
    solver.add(z3::mk_or(mnyEv));
    solver.add(z3::mk_or(mxyEv));

    // shape
    Vector<Pair<Int, Int>> vFactors;
    calcFactors(area, vFactors);

    //std::cerr << ac.name() << " " << area << " " << ac.size() << std::endl;
    Z3ExprVector arrayShapeEv(_ctx);
    for (const auto& p : vFactors) {
      //std::cerr << p.first << " " << p.second << std::endl;;
      const Z3Expr& xSize = _ctx.bv_val(static_cast<unsigned>(p.first), _z3XBvSize);
      const Z3Expr& ySize = _ctx.bv_val(static_cast<unsigned>(p.second), _z3YBvSize);
      arrayShapeEv.push_back((mxx - mnx == xSize) && (mxy - mny == ySize));
      //arrayShapeEv.push_back((mnx + xSize == mxx) && (mny + ySize == mxy));
      //solver.add(z3::bvadd_no_overflow(mnx, xSize, false));
      //solver.add(z3::bvadd_no_overflow(mny, ySize, false));
    }
    solver.add(z3::mk_or(arrayShapeEv));
    
    //regular subarrays shape
    for (i = 1; i < ac.numPlaceArrayCstrs(); ++i) {
      const PlaceArrayCstr& pre = ac.placeArrayCstr(i - 1);
      const PlaceArrayCstr& cur = ac.placeArrayCstr(i);
      
      const Z3Expr& preMnx = *_vpArrayMinXExprs.at(pre.idx());
      const Z3Expr& preMxx = *_vpArrayMaxXExprs.at(pre.idx());
      const Z3Expr& preMny = *_vpArrayMinYExprs.at(pre.idx());
      const Z3Expr& preMxy = *_vpArrayMaxYExprs.at(pre.idx());

      const Z3Expr& curMnx = *_vpArrayMinXExprs.at(cur.idx());
      const Z3Expr& curMxx = *_vpArrayMaxXExprs.at(cur.idx());
      const Z3Expr& curMny = *_vpArrayMinYExprs.at(cur.idx());
      const Z3Expr& curMxy = *_vpArrayMaxYExprs.at(cur.idx());

      solver.add(preMxx - preMnx == curMxx - curMnx);
      solver.add(preMxy - preMny == curMxy - curMny);
    }

    
  }
}

void SmtPlacer::addZ3CstrArrayNonOverlap(Z3Solver& solver, const Region& reg)
{
  // no other cells within the array bbox
  spdlog::info("[SmtPlacer] Add Z3 array non-overlap constraints");
  
  for (Int i = 0; i < reg.numPlaceArrayCstrs(); ++i) {
    const PlaceArrayCstr& arr = reg.placeArrayCstr(i);
    const Z3Expr& mnx = *_vpArrayMinXExprs.at(arr.idx());
    const Z3Expr& mxx = *_vpArrayMaxXExprs.at(arr.idx());
    const Z3Expr& mny = *_vpArrayMinYExprs.at(arr.idx());
    const Z3Expr& mxy = *_vpArrayMaxYExprs.at(arr.idx());
    for (Int j = 0; j < reg.numCells(); ++j) {
      const Cell& c = reg.cell(j);
      if (!arr.hasCell(c)) {
        const Z3Expr& cx = *_vpCellXExprs.at(c.idx());
        const Z3Expr& cy = *_vpCellYExprs.at(c.idx());
        const Z3Expr& cw = _ctx.bv_val(static_cast<unsigned>(c.width() / _stepX), _z3XBvSize);
        const Z3Expr& ch = _ctx.bv_val(static_cast<unsigned>(c.height() / _stepY), _z3YBvSize);
        solver.add(z3::ule(cx + cw, mnx) || z3::ule(mxx, cx) ||
                   z3::ule(cy + ch, mny) || z3::ule(mxy, cy));
      }
    }
  }
  
}

void SmtPlacer::addZ3CstrIOPinSlot(Z3Solver& solver, const Int xl, const Int yl, const Int xh, const Int yh)
{
  spdlog::info("[SmtPlacer] Add Z3 io pin slot constraints");

  Int i, j;
  const Pin* pPin; 

  const Int numLeftPins = _cir.numLeftIOPins();
  const Int numRightPins = _cir.numRightIOPins();
  const Int numBottomPins = _cir.numBottomIOPins();
  const Int numTopPins = _cir.numTopIOPins();
  const Int numXSlots = xh - xl - 2;
  const Int numYSlots = yh - yl - 2;
  assert(numXSlots > 0);
  assert(numYSlots > 0);

  // conservative (each io pin should select a slot)
  Cir_ForEachLeftIOPin(_cir, pPin, i) {
    Z3ExprVector ev(_ctx);
    for (const auto& exprPtr : _vvpIOLeftSlotExprs.at(i)) {
      ev.push_back(*exprPtr);
    }
    Vector<Int> vCoeffs(numYSlots, 1);
    solver.add(z3::pbeq(ev, vCoeffs.data(), 1));
  }
  Cir_ForEachRightIOPin(_cir, pPin, i) {
    Z3ExprVector ev(_ctx);
    for (const auto& exprPtr : _vvpIORightSlotExprs.at(i)) {
      ev.push_back(*exprPtr);
    }
    Vector<Int> vCoeffs(numYSlots, 1);
    solver.add(z3::pbeq(ev, vCoeffs.data(), 1));
  }
  Cir_ForEachBottomIOPin(_cir, pPin, i) {
    Z3ExprVector ev(_ctx);
    for (const auto& exprPtr : _vvpIOBottomSlotExprs.at(i)) {
      ev.push_back(*exprPtr);
    }
    Vector<Int> vCoeffs(numXSlots, 1);
    solver.add(z3::pbeq(ev, vCoeffs.data(), 1));
  }
  Cir_ForEachTopIOPin(_cir, pPin, i) {
    Z3ExprVector ev(_ctx);
    for (const auto& exprPtr : _vvpIOTopSlotExprs.at(i)) {
      ev.push_back(*exprPtr);
    }
    Vector<Int> vCoeffs(numXSlots, 1);
    solver.add(z3::pbeq(ev, vCoeffs.data(), 1));
  }

  // density (upperbound number of pin assigned to the same slot)
  const Int maxLeftSlotOverlaps = std::max(numLeftPins / numYSlots, _maxIOSlotOverlap);
  const Int maxRightSlotOverlaps = std::max(numRightPins / numYSlots, _maxIOSlotOverlap);
  const Int maxBottomSlotOverlaps = std::max(numBottomPins / numXSlots, _maxIOSlotOverlap);
  const Int maxTopSlotOverlaps = std::max(numTopPins / numXSlots, _maxIOSlotOverlap);

  //spdlog::info("{} {} {} {}", maxLeftSlotOverlaps, maxRightSlotOverlaps, maxBottomSlotOverlaps, maxTopSlotOverlaps);

  for (i = 0; i < numYSlots; ++i) {
    Z3ExprVector ev(_ctx);
    Cir_ForEachLeftIOPin(_cir, pPin, j) {
      const Z3Expr& expr = *_vvpIOLeftSlotExprs.at(j).at(i);
      ev.push_back(expr);
    }
    if (ev.size()) {
      Vector<Int> vCoeffs(_cir.numIOPins(), 1);
      solver.add(z3::pble(ev, vCoeffs.data(), maxLeftSlotOverlaps));
    }
  }
  for (i = 0; i < numYSlots; ++i) {
    Z3ExprVector ev(_ctx);
    Cir_ForEachRightIOPin(_cir, pPin, j) {
      const Z3Expr& expr = *_vvpIORightSlotExprs.at(j).at(i);
      ev.push_back(expr);
    }
    if (ev.size()) {
      Vector<Int> vCoeffs(_cir.numIOPins(), 1);
      solver.add(z3::pble(ev, vCoeffs.data(), maxRightSlotOverlaps));
    }
  }
  for (i = 0; i < numXSlots; ++i) {
    Z3ExprVector ev(_ctx);
    Cir_ForEachBottomIOPin(_cir, pPin, j) {
      const Z3Expr& expr = *_vvpIOBottomSlotExprs.at(j).at(i);
      ev.push_back(expr);
    }
    if (ev.size()) {
      Vector<Int> vCoeffs(_cir.numIOPins(), 1);
      solver.add(z3::pble(ev, vCoeffs.data(), maxBottomSlotOverlaps));
    }
  }
  for (i = 0; i < numXSlots; ++i) {
    Z3ExprVector ev(_ctx);
    Cir_ForEachTopIOPin(_cir, pPin, j) {
      const Z3Expr& expr = *_vvpIOTopSlotExprs.at(j).at(i);
      ev.push_back(expr);
    }
    if (ev.size()) {
      Vector<Int> vCoeffs(_cir.numIOPins(), 1);
      solver.add(z3::pble(ev, vCoeffs.data(), maxTopSlotOverlaps));
    }
  }
}

void SmtPlacer::addZ3CstrIOPinLoc(Z3Solver& solver, const Int xl, const Int yl, const Int xh, const Int yh)
{
  spdlog::info("[SmtPlacer] Add Z3 io pin location constraints");
  // left
  for (size_t i = 0; i < _vvpIOLeftSlotExprs.size(); ++i) {
    const auto& vpExprs = _vvpIOLeftSlotExprs.at(i);
    assert(static_cast<Int>(vpExprs.size()) == yh - yl - 2);

    const Int ioIdx = _cir.ioPinL(i).ioIdx();
    const Int regIdx = _cir.ioPinL(i).net().region().idx();
    
    const Z3Expr& xExpr = *_vpIOPinXExprs.at(ioIdx);
    const Z3Expr& yExpr = *_vpIOPinYExprs.at(ioIdx);

    const Z3Expr& regXExpr = *_vpRegXExprs.at(regIdx);
    const Z3Expr& regYExpr = *_vpRegYExprs.at(regIdx);
    const Z3Expr& regHExpr = *_vpRegHExprs.at(regIdx);

    for (Int y = yl + 1; y < yh - 1; ++y) {
      const Int idx = y - yl - 1;
      const Z3Expr& expr = *vpExprs.at(idx);
      solver.add(z3::implies(expr, (xExpr == regXExpr)));
      solver.add(z3::implies(expr, (yExpr == y)));
      const Z3Expr& Y = _ctx.bv_val(static_cast<unsigned>(y), _z3YBvSize);
      solver.add(z3::implies(z3::ult(Y, regYExpr), expr == _ctx.bool_val(false)));
      solver.add(z3::implies(z3::ule(regYExpr + regHExpr, Y), expr == _ctx.bool_val(false)));
    }
  }
  // right
  for (size_t i = 0; i < _vvpIORightSlotExprs.size(); ++i) {
    const auto& vpExprs = _vvpIORightSlotExprs.at(i);
    assert(static_cast<Int>(vpExprs.size()) == yh - yl - 2);

    const Int ioIdx = _cir.ioPinR(i).ioIdx();
    const Int regIdx = _cir.ioPinR(i).net().region().idx();

    const Z3Expr& xExpr = *_vpIOPinXExprs.at(ioIdx);
    const Z3Expr& yExpr = *_vpIOPinYExprs.at(ioIdx);

    const Z3Expr& regXExpr = *_vpRegXExprs.at(regIdx);
    const Z3Expr& regYExpr = *_vpRegYExprs.at(regIdx);
    const Z3Expr& regWExpr = *_vpRegWExprs.at(regIdx);
    const Z3Expr& regHExpr = *_vpRegHExprs.at(regIdx);

    for (Int y = yl + 1; y < yh - 1; ++y) {
      const Int idx = y - yl - 1;
      const Z3Expr& expr = *vpExprs.at(idx);
      solver.add(z3::implies(expr, (xExpr == (regXExpr + regWExpr)))); 
      solver.add(z3::implies(expr, (yExpr == y)));
      const Z3Expr& Y = _ctx.bv_val(static_cast<unsigned>(y), _z3YBvSize);
      solver.add(z3::implies(z3::ult(Y, regYExpr), expr == _ctx.bool_val(false)));
      solver.add(z3::implies(z3::ule(regYExpr + regHExpr, Y), expr == _ctx.bool_val(false)));
    }
  }
  // bottom
  for (size_t i = 0; i < _vvpIOBottomSlotExprs.size(); ++i) {
    const auto& vpExprs = _vvpIOBottomSlotExprs.at(i);
    assert(static_cast<Int>(vpExprs.size()) == xh - xl - 2);

    const Int ioIdx = _cir.ioPinB(i).ioIdx();
    const Int regIdx = _cir.ioPinB(i).net().region().idx();

    const Z3Expr& xExpr = *_vpIOPinXExprs.at(ioIdx);
    const Z3Expr& yExpr = *_vpIOPinYExprs.at(ioIdx);

    const Z3Expr& regXExpr = *_vpRegXExprs.at(regIdx);
    const Z3Expr& regYExpr = *_vpRegYExprs.at(regIdx);
    const Z3Expr& regWExpr = *_vpRegWExprs.at(regIdx);

    for (Int x = xl + 1; x < xh - 1; ++x) {
      const Int idx = x - xl - 1;
      const Z3Expr& expr = *vpExprs.at(idx);
      solver.add(z3::implies(expr, (xExpr == x)));
      solver.add(z3::implies(expr, (yExpr == regYExpr)));
      const Z3Expr& X = _ctx.bv_val(static_cast<unsigned>(x), _z3XBvSize);
      solver.add(z3::implies(z3::ult(X, regXExpr), expr == _ctx.bool_val(false)));
      solver.add(z3::implies(z3::ule(regXExpr + regWExpr, X), expr == _ctx.bool_val(false)));
    }
  }
  // top
  for (size_t i = 0; i < _vvpIOTopSlotExprs.size(); ++i) {
    const auto& vpExprs = _vvpIOTopSlotExprs.at(i);
    assert(static_cast<Int>(vpExprs.size()) == xh - xl - 2);

    const Int ioIdx = _cir.ioPinT(i).ioIdx();
    const Int regIdx = _cir.ioPinT(i).net().region().idx();

    const Z3Expr& xExpr = *_vpIOPinXExprs.at(ioIdx);
    const Z3Expr& yExpr = *_vpIOPinYExprs.at(ioIdx);
    
    const Z3Expr& regXExpr = *_vpRegXExprs.at(regIdx);
    const Z3Expr& regYExpr = *_vpRegYExprs.at(regIdx);
    const Z3Expr& regWExpr = *_vpRegWExprs.at(regIdx);
    const Z3Expr& regHExpr = *_vpRegHExprs.at(regIdx);

    for (Int x = xl + 1; x < xh - 1; ++x) {
      const Int idx = x - xl - 1;
      const Z3Expr& expr = *vpExprs.at(idx);
      solver.add(z3::implies(expr, (xExpr == x)));
      solver.add(z3::implies(expr, (yExpr == (regYExpr + regHExpr))));
      const Z3Expr& X = _ctx.bv_val(static_cast<unsigned>(x), _z3XBvSize);
      solver.add(z3::implies(z3::ult(X, regXExpr), expr == _ctx.bool_val(false)));
      solver.add(z3::implies(z3::ule(regXExpr + regWExpr, X), expr == _ctx.bool_val(false)));
    }
  }
}


void SmtPlacer::addZ3CstrNetHpwl(Z3Solver& solver, const bool useIO)
{
  spdlog::info("[SmtPlacer] Add Z3 net hpwl constraints");

  Vector<Int> vNetIds;

  Int i;
  const Net* pNet;
  Cir_ForEachNet(_cir, pNet, i) {
    const Net& n = *pNet;
    if (!n.isPower() and n.numPins() > 1) {
      vNetIds.emplace_back(n.idx());

      const Z3Expr& mnx = *_vpNetMinXExprs.at(n.idx());
      const Z3Expr& mxx = *_vpNetMaxXExprs.at(n.idx());
      const Z3Expr& mny = *_vpNetMinYExprs.at(n.idx());
      const Z3Expr& mxy = *_vpNetMaxYExprs.at(n.idx());
      
      Z3ExprVector mnxEv(_ctx);
      Z3ExprVector mxxEv(_ctx);
      Z3ExprVector mnyEv(_ctx);
      Z3ExprVector mxyEv(_ctx);
      for (const Pin* pPin : n.vpPins()) {
        if (pPin->pCell()) {
          const Cell& c = pPin->cell();
          const Z3Expr& x = *_vpCellXExprs.at(c.idx());
          const Z3Expr& y = *_vpCellYExprs.at(c.idx());
          const Int gw = c.width() / _stepX;
          const Int gh = c.height() / _stepY;
          const Z3Expr& w = _ctx.bv_val(static_cast<unsigned>(gw), _z3XBvSize);
          const Z3Expr& h = _ctx.bv_val(static_cast<unsigned>(gh), _z3YBvSize);
          mnxEv.push_back(mnx == x);
          mxxEv.push_back(mxx == x + w);
          mnyEv.push_back(mny == y);
          mxyEv.push_back(mxy == y + h);
          solver.add(z3::ule(mnx, x));
          solver.add(z3::ule(x + w, mxx));
          solver.add(z3::ule(mny, y));
          solver.add(z3::ule(y + h, mxy));
        }
        else if (useIO) {
          assert(n.isIO());
          const Z3Expr& x = *_vpIOPinXExprs.at(pPin->ioIdx());
          const Z3Expr& y = *_vpIOPinYExprs.at(pPin->ioIdx());
          mnxEv.push_back(mnx == x);
          mxxEv.push_back(mxx == x);
          mnyEv.push_back(mny == y);
          mxyEv.push_back(mxy == y);
          solver.add(z3::ule(mnx, x));
          solver.add(z3::ule(x, mxx));
          solver.add(z3::ule(mny, y));
          solver.add(z3::ule(y, mxy));
        }
      }
      solver.add(z3::mk_or(mnxEv));
      solver.add(z3::mk_or(mxxEv));
      solver.add(z3::mk_or(mnyEv));
      solver.add(z3::mk_or(mxyEv));
    }
  }
  for (i = 0; i < static_cast<Int>(vNetIds.size()); ++i) {
    const Int netIdx = vNetIds.at(i);
    const Z3Expr& mnx = z3::zext(*_vpNetMinXExprs.at(netIdx), _z3HpwlBvSize - _z3XBvSize);
    const Z3Expr& mxx = z3::zext(*_vpNetMaxXExprs.at(netIdx), _z3HpwlBvSize - _z3XBvSize);
    const Z3Expr& mny = z3::zext(*_vpNetMinYExprs.at(netIdx), _z3HpwlBvSize - _z3YBvSize);
    const Z3Expr& mxy = z3::zext(*_vpNetMaxYExprs.at(netIdx), _z3HpwlBvSize - _z3YBvSize);
    const Z3Expr& prio = _ctx.bv_val(static_cast<unsigned>(_cir.net(netIdx).priority()), _z3HpwlBvSize);
    if (_pTotHpwlExpr) {
      Z3Expr& totHpwlExpr = *_pTotHpwlExpr;
      totHpwlExpr = totHpwlExpr + prio * (mxx - mnx + mxy - mny);
      solver.add(z3::bvmul_no_overflow(prio, (mxx - mnx + mxy - mny), false));
      solver.add(z3::bvadd_no_overflow(totHpwlExpr, prio * (mxx - mnx + mxy - mny), false));
    }
    else {
      _pTotHpwlExpr = std::make_unique<Z3Expr>(prio * (mxx - mnx + mxy - mny));
      solver.add(z3::bvmul_no_overflow(prio, (mxx - mnx + mxy - mny), false));
    }
  }
}

void SmtPlacer::addZ3CstrNetSymDis(Z3Solver& solver)
{
  if (_cir.numRouteSymCstrs() == 0) {
    return;
  }

  spdlog::info("[SmtPlacer] Add Z3 net symmetry distance constraints");

  Int i, j, k, l;
  const RouteSymCstr* pCstr;
  const Pin* pa;
  const Pin* pb;

  const Z3Expr& oneX = _ctx.bv_val(static_cast<unsigned>(1), _z3XBvSize);
  Cir_ForEachRouteSymCstr(_cir, pCstr, i) {
    const RouteSymCstr& sc = *pCstr;
    for (j = 0; j < sc.numNets(); ++j) {
      const Net& na = sc.net(j);
      if (sc.isPartA(j)) {
        const Net& nb = sc.symNet(j);
        Net_ForEachPin(na, pa, k) {
          const Cell& ca = pa->cell();
          const Z3Expr& xa = *_vpCellXExprs.at(ca.idx());
          const Z3Expr& ya = *_vpCellYExprs.at(ca.idx());
          Net_ForEachPin(nb, pb, l) {
            const Cell& cb = pb->cell();
            if (ca.idx() != cb.idx()) {
              const Z3Expr& xb = *_vpCellXExprs.at(cb.idx());
              const Z3Expr& yb = *_vpCellYExprs.at(cb.idx());
              solver.add(z3::implies(ya == yb, 
                                     (z3::ult(xa + oneX, xb) || z3::ult(xb + oneX, xa))));
            }
          }
        }
      }
    }
  }

}

void SmtPlacer::addZ3CstrExtension(Z3Solver& solver)
{
  if (_cir.numPlaceExtCstrs() == 0) {
    return;
  }
  spdlog::info("[SmtPlacer] Add Z3 extension constraints");

  Int i;
  const PlaceExtCstr* pCstr;
  Cir_ForEachPlaceExtCstr(_cir, pCstr, i) {
    const PlaceExtCstr& cstr = *pCstr;

    switch (cstr.type()) {
      case ExtTypeE::cell: {
        const Region& reg = cstr.cell(0).region();
        const Z3Expr& extL = _ctx.bv_val(static_cast<unsigned>(cstr.leftExt().second * _vStepXFactors.at(reg.idx())), _z3XBvSize);
        const Z3Expr& extR = _ctx.bv_val(static_cast<unsigned>(cstr.rightExt().second * _vStepXFactors.at(reg.idx())), _z3XBvSize);
        const Z3Expr& extB = _ctx.bv_val(static_cast<unsigned>(cstr.bottomExt().second * _vStepYFactors.at(reg.idx())), _z3YBvSize);
        const Z3Expr& extT = _ctx.bv_val(static_cast<unsigned>(cstr.topExt().second * _vStepYFactors.at(reg.idx())), _z3YBvSize);
        for (Int j = 0; j < cstr.numCells(); ++j) {
          const Cell& cj = cstr.cell(j);
          const Z3Expr& xj = *_vpCellXExprs.at(cj.idx());
          const Z3Expr& yj = *_vpCellYExprs.at(cj.idx());
          const Z3Expr& wj = _ctx.bv_val(static_cast<unsigned>(cj.width() / _stepX), _z3XBvSize);
          const Z3Expr& hj = _ctx.bv_val(static_cast<unsigned>(cj.height() / _stepY), _z3YBvSize);
          for (Int k = 0; k < reg.numCells(); ++k) {
            const Cell& ck = reg.cell(k);
            if (ck.idx() != cj.idx()) {
              const Z3Expr& xk = *_vpCellXExprs.at(ck.idx());
              const Z3Expr& yk = *_vpCellYExprs.at(ck.idx());
              const Z3Expr& wk = _ctx.bv_val(static_cast<unsigned>(ck.width() / _stepX), _z3XBvSize);
              const Z3Expr& hk = _ctx.bv_val(static_cast<unsigned>(ck.height() / _stepY), _z3YBvSize);
              solver.add(z3::ule(xj + wj + extR, xk) || 
                         z3::ule(xk + wk + extL, xj) ||
                         z3::ule(yj + hj + extT, yk) ||
                         z3::ule(yk + hk + extB, yj));
            }
          }
        }
        break;
      }
      case ExtTypeE::region: {
        // TODO
        break;
      }
      case ExtTypeE::array: {
        const Region& reg = cstr.placeArrayCstr(0).region();
        const Z3Expr& extL = _ctx.bv_val(static_cast<unsigned>(cstr.leftExt().second * _vStepXFactors.at(reg.idx())), _z3XBvSize);
        const Z3Expr& extR = _ctx.bv_val(static_cast<unsigned>(cstr.rightExt().second * _vStepXFactors.at(reg.idx())), _z3XBvSize);
        const Z3Expr& extB = _ctx.bv_val(static_cast<unsigned>(cstr.bottomExt().second * _vStepYFactors.at(reg.idx())), _z3YBvSize);
        const Z3Expr& extT = _ctx.bv_val(static_cast<unsigned>(cstr.topExt().second * _vStepYFactors.at(reg.idx())), _z3YBvSize);
        for (Int j = 0; j < cstr.numPlaceArrayCstrs(); ++j) {
          const PlaceArrayCstr& ac = cstr.placeArrayCstr(j);
          const Z3Expr& mnx = *_vpArrayMinXExprs.at(ac.idx());
          const Z3Expr& mny = *_vpArrayMinYExprs.at(ac.idx());
          const Z3Expr& mxx = *_vpArrayMaxXExprs.at(ac.idx());
          const Z3Expr& mxy = *_vpArrayMaxYExprs.at(ac.idx());
          for (Int k = 0; k < reg.numCells(); ++k) {
            const Cell& ck = reg.cell(k);
            if (!ac.hasCell(ck)) {
              const Z3Expr& xk = *_vpCellXExprs.at(ck.idx());
              const Z3Expr& yk = *_vpCellYExprs.at(ck.idx());
              const Z3Expr& wk = _ctx.bv_val(static_cast<unsigned>(ck.width() / _stepX), _z3XBvSize);
              const Z3Expr& hk = _ctx.bv_val(static_cast<unsigned>(ck.height() / _stepY), _z3YBvSize);
              solver.add(z3::ule(mxx + extR, xk) || 
                         z3::ule(xk + wk + extL, mnx) ||
                         z3::ule(mxy + extT, yk) ||
                         z3::ule(yk + hk + extB, mny));
            }
          }

        }
        break;
      }
      default:
        assert(false);
    }
  }
}

void SmtPlacer::addZ3CstrRoutePinDensity(Z3Solver& solver, const Int xl, const Int yl, const Int xh, const Int yh, const Int wsizeX, const Int wsizeY)
{
  spdlog::info("[SmtPlacer] Add Z3 pin density constraints");

  Int i, j, k;
  const Cell* pCell;

  // set max pins for each window
  Vector<Int> vNumPins(_cir.numCells());
  Cir_ForEachCell(_cir, pCell, i) {
    vNumPins[pCell->idx()] = pCell->numSignalPins();
  }
  //for (Int x = xl + 1; x < xh; ++x) {
    //for (Int y = yl + 1; y < yh; ++y) {
      //const Int xIdx = x - xl - 1;
      //const Int yIdx = y - yl - 1;
      //Z3ExprVector ev(_ctx);
      //Cir_ForEachCell(_cir, pCell, i) {
        //ev.push_back(*_vpPinDensityExprs[pCell->idx()].at(xIdx, yIdx));
      //}
      //solver.add(z3::pble(ev, vNumPins.data(), _maxPinCnts));
    //}
  //}
  const Int numXSlots = xh - xl - 2; // [xl + 1, xh)
  const Int numYSlots = yh - yl - 2; // [yl + 1, yh)
  const Int numXWins = numXSlots - wsizeX + 1;
  const Int numYWins = numYSlots - wsizeY + 1;
  for (i = 0; i < numXWins; ++i) {
    for (j = 0; j < numYWins; ++j) {
      Z3ExprVector ev(_ctx);
      Cir_ForEachCell(_cir, pCell, k) {
        ev.push_back(*_vpPinDensityExprs[pCell->idx()].at(i, j));
      }
      solver.add(z3::pble(ev, vNumPins.data(), _maxPinCnts));
    }
  }

  // link cell loc with pin cnt on slots
  //Cir_ForEachCell(_cir, pCell, i) {
    //const Z3Expr& xExpr = *_vpCellXExprs.at(pCell->idx()); 
    //const Z3Expr& yExpr = *_vpCellYExprs.at(pCell->idx());
    //const Int w = pCell->width() / _stepX;
    //const Int h = pCell->height() / _stepY;
    //const Int extX = wsizeX - 1;
    //const Int extY = wsizeY - 1;
    //for (Int x = xl + 1; x < xh; ++x) {
      //for (Int y = yl + 1; y < yh; ++y) {
        //const Z3Expr& xVal = _ctx.bv_val(static_cast<unsigned>(x), _z3XBvSize);
        //const Z3Expr& yVal = _ctx.bv_val(static_cast<unsigned>(y), _z3YBvSize);
        
        //const Int startX = std::max(x - extX, xl + 1), endX = std::min(x + w - 1 + extX, xh - 1);
        //const Int startY = std::max(y - extY, yl + 1), endY = std::min(y + h - 1 + extY, yh - 1);
        ////for (Int xx = xl + 1; xx < xh; ++xx) {
          ////for (Int yy = yl + 1; yy < yh; ++yy) {
            ////const Int xIdx = xx - xl - 1;
            ////const Int yIdx = yy - yl - 1;
            ////const Z3Expr& p = *_vpPinDensityExprs[pCell->idx()].at(xIdx, yIdx);
            ////if (startX <= xx and xx <= endX and startY <= yy and yy <= endY) {
              ////solver.add(z3::implies(xExpr == xVal && yExpr == yVal, p == _ctx.bool_val(true)));
            ////}
            ////else {
              ////solver.add(z3::implies(xExpr == xVal && yExpr == yVal, p == _ctx.bool_val(false)));
            ////}
          ////}
        ////}
        //for (Int xx = startX; xx <= endX; ++xx) {
          //for (Int yy = startY; yy <= endY; ++yy) {
            //const Int xIdx = xx - xl - 1;
            //const Int yIdx = yy - yl - 1;
            //const Z3Expr& p = *_vpPinDensityExprs[pCell->idx()].at(xIdx, yIdx);
            //solver.add(z3::implies(xExpr == xVal && yExpr == yVal, p == _ctx.bool_val(true)));
          //}
        //}
      //}
    //}
  //}

  const Z3Expr& winX = _ctx.bv_val(static_cast<unsigned>(wsizeX), _z3XBvSize);
  const Z3Expr& winY = _ctx.bv_val(static_cast<unsigned>(wsizeY), _z3YBvSize);
  for (i = 0; i < numXWins; ++i) {
    const Z3Expr& x = _ctx.bv_val(static_cast<unsigned>(xl + 1 + i), _z3XBvSize);
    for (j = 0; j < numYWins; ++j) {
      const Z3Expr& y = _ctx.bv_val(static_cast<unsigned>(yl + 1 + j), _z3YBvSize);

      Cir_ForEachCell(_cir, pCell, k) {
        const Z3Expr& winExpr = *_vpPinDensityExprs[pCell->idx()].at(i, j);

        const Z3Expr& xExpr = *_vpCellXExprs.at(pCell->idx()); 
        const Z3Expr& yExpr = *_vpCellYExprs.at(pCell->idx());
        const Z3Expr& wExpr = _ctx.bv_val(static_cast<unsigned>(pCell->width() / _stepX), _z3XBvSize);
        const Z3Expr& hExpr = _ctx.bv_val(static_cast<unsigned>(pCell->height() / _stepY), _z3YBvSize);

        const Z3Expr& overlapExpr = !(z3::ule(x + winX, xExpr) ||
                                      z3::ule(xExpr + wExpr, x) ||
                                      z3::ule(y + winY, yExpr) ||
                                      z3::ule(yExpr + hExpr, y));
        solver.add(z3::implies(overlapExpr, winExpr == _ctx.bool_val(true)));
      }
    }
  }
  // overflow handling
  for (i = 0; i < numXWins; ++i) {
    const Z3Expr& x = _ctx.bv_val(static_cast<unsigned>(xl + 1 + i), _z3XBvSize);
    solver.add(z3::bvadd_no_overflow(x, winX, false));
  }
  for (i = 0; i < numYWins; ++i) {
    const Z3Expr& y = _ctx.bv_val(static_cast<unsigned>(yl + 1 + i), _z3YBvSize);
    solver.add(z3::bvadd_no_overflow(y, winY, false));
  }

}

void SmtPlacer::freezeZ3RegSizeResult(Z3Solver& solver, const Z3Model& model) const
{
  Int i;
  const Region* pReg;
  Cir_ForEachRegion(_cir, pReg, i) {
    freezeZ3RegSizeResult(solver, model, *pReg);
  }
}

void SmtPlacer::freezeZ3RegSizeResult(Z3Solver& solver, const Z3Model& model, const Region& reg) const
{
  const Z3Expr& wExpr = *_vpRegWExprs.at(reg.idx());
  const Z3Expr& hExpr = *_vpRegHExprs.at(reg.idx());
  solver.add(wExpr == model.eval(wExpr));
  solver.add(hExpr == model.eval(hExpr));
}

void SmtPlacer::freezeZ3RegResult(Z3Solver& solver, const Z3Model& model) const
{
  Int i;
  const Region* pReg;
  Cir_ForEachRegion(_cir, pReg, i) {
    freezeZ3RegResult(solver, model, *pReg);
  }
}

void SmtPlacer::freezeZ3RegResult(Z3Solver& solver, const Z3Model& model, const Region& reg) const
{
  const Z3Expr& xExpr = *_vpRegXExprs.at(reg.idx());
  const Z3Expr& yExpr = *_vpRegYExprs.at(reg.idx());
  const Z3Expr& wExpr = *_vpRegWExprs.at(reg.idx());
  const Z3Expr& hExpr = *_vpRegHExprs.at(reg.idx());
  solver.add(xExpr == model.eval(xExpr));
  solver.add(yExpr == model.eval(yExpr));
  solver.add(wExpr == model.eval(wExpr));
  solver.add(hExpr == model.eval(hExpr));
}

void SmtPlacer::freezeZ3CellResult(Z3Solver& solver, const Z3Model& model) const
{
  Int i;
  const Cell* pCell;
  Cir_ForEachCell(_cir, pCell, i) {
    freezeZ3CellResult(solver, model, *pCell);
  }
}

void SmtPlacer::freezeZ3CellResult(Z3Solver& solver, const Z3Model& model, const Region& reg) const
{
  Int i;
  const Cell* pCell;
  Reg_ForEachCell(_cir, pCell, i) {
    freezeZ3CellResult(solver, model, *pCell);
  }
}

void SmtPlacer::freezeZ3CellResult(Z3Solver& solver, const Z3Model& model, const Cell& cell) const
{
  const Z3Expr& xExpr = *_vpCellXExprs.at(cell.idx());
  const Z3Expr& yExpr = *_vpCellYExprs.at(cell.idx());
  solver.add(xExpr == model.eval(xExpr));
  solver.add(yExpr == model.eval(yExpr));
}

void SmtPlacer::freezeZ3IOPinResult(Z3Solver& solver, const Z3Model& model) const 
{
  Int i;
  const Pin* pPin;
  Cir_ForEachIOPin(_cir, pPin, i) {
    const Z3Expr& xExpr = *_vpIOPinXExprs.at(pPin->ioIdx());
    const Z3Expr& yExpr = *_vpIOPinYExprs.at(pPin->ioIdx());
    solver.add(xExpr == model.eval(xExpr));
    solver.add(yExpr == model.eval(yExpr));
  }
}

void SmtPlacer::genHpwlCandidates(const Int curHpwl, const Int tarHpwl, const Int num, Vector<Int>& vHpwlCandidates) const
{
  vHpwlCandidates.clear();
  if (num == 1) {
    vHpwlCandidates.emplace_back(tarHpwl);
  }
  else {
    const Int step = (curHpwl - tarHpwl) / num;
    for (Int i = 0; i < num; ++i) {
      vHpwlCandidates.emplace_back(tarHpwl + i * step);
    }
  }
}

Real SmtPlacer::calcExpectedDesignArea(const Real fcAreaExpand, const Real fcUtil) const
{
  Real area = 0;
  Int i;
  const Cell* pCell;
  Cir_ForEachCell(_cir, pCell, i) {
    area += pCell->area();
  }
  area *= fcAreaExpand;
  area /= fcUtil;
  return area;
}

Int SmtPlacer::calcExpectedW(const Real area) const
{
  return std::lround(std::sqrt(area * _ar));
}

Int SmtPlacer::calcExpectedH(const Real area) const
{
  return std::lround(std::sqrt(area / _ar));
}

Pair<Int, Int> SmtPlacer::calcGridStep() const
{
  Int i;
  const Region* pReg;

  Vector<Int> vStepXs, vStepYs;
  Cir_ForEachRegion(_cir, pReg, i) {
    vStepXs.emplace_back(pReg->placeGrid().stepX());
    vStepYs.emplace_back(pReg->placeGrid().stepY());
  }
  Int resX = vStepXs.at(0), resY = vStepYs.at(0);
  for (i = 1; i < static_cast<Int>(vStepXs.size()); ++i) {
    resX = std::gcd(vStepXs.at(i), resX);
    if (resX == 1) {
      break;
    }
  }
  for (i = 1; i < static_cast<Int>(vStepYs.size()); ++i) {
    resY = std::gcd(vStepYs.at(i), resY);
    if (resY == 1) {
      break;
    }
  }
  return std::make_pair(resX, resY);
}

void SmtPlacer::calcFactors(const Int n, Vector<Pair<Int, Int>>& v) const
{
  v.clear();
  const Int sq = static_cast<Int>(std::sqrt(n));
  for (Int i = 1; i <= sq; ++i) {
    if (n % i == 0) {
      const Int j = n / i;
      v.emplace_back(i, j);
      if (j != i)  {
        v.emplace_back(j, i);
      }
    }
  }
}

void SmtPlacer::calcApproxFactors(const Int n, const bool isEvenRow, Vector<Pair<Int, Int>>& v) const
{
  v.clear();
  const Int sq = static_cast<Int>(std::sqrt(n));
  for (Int i = 1; i <= sq; ++i) {
    if (n % i == 0) {
      const Int j = n / i;
      if (!isEvenRow or j % 2 == 0) {
        v.emplace_back(i, j);
      }
      if (j != i) {
        if (!isEvenRow or i % 2 == 0) {
          v.emplace_back(j, i);
        }
      }
    }
    else {
      const Int j = n / i + 1;
      if (!isEvenRow or j % 2 == 0) {
        v.emplace_back(i, j);
      }
      if (j != i) {
        if (!isEvenRow or i % 2 == 0) {
          v.emplace_back(j, i);
        }
      }
    }
  }
}

Int SmtPlacer::calcLog2(Int x) const
{
  Int ret = 0;
  while ((x >>= 1) > 0) {
    ++ret;
  }
  return ret;
}

Int SmtPlacer::calcZ3NetHpwl(const Z3Model& model, const bool useIO, const bool useCluster) const
{
  Int i, hpwl = 0;
  const Net* pNet;
  Cir_ForEachNet(_cir, pNet, i) {
    if (!pNet->isPower() and pNet->numPins() > 1) {
      Int mnx = MAX_INT, mxx = MIN_INT;
      Int mny = MAX_INT, mxy = MIN_INT;
      for (const Pin* pPin : pNet->vpPins()) {
        if (pPin->pCell()) {
          const Cell& c = pPin->cell();
          const Z3Expr& x = *_vpCellXExprs.at(c.idx());
          const Z3Expr& y = *_vpCellYExprs.at(c.idx());
          const Int w = c.width() / _stepX;
          const Int h = c.height() / _stepY;
          mnx = std::min(mnx, model.eval(x).get_numeral_int());
          mxx = std::max(mxx, model.eval(x).get_numeral_int() + w);
          mny = std::min(mny, model.eval(y).get_numeral_int());
          mxy = std::max(mxy, model.eval(y).get_numeral_int() + h);
        }
        else if (useIO) {
          const Z3Expr& x = *_vpIOPinXExprs.at(pPin->ioIdx());
          const Z3Expr& y = *_vpIOPinYExprs.at(pPin->ioIdx());
          mnx = std::min(mnx, model.eval(x).get_numeral_int());
          mxx = std::max(mxx, model.eval(x).get_numeral_int());
          mny = std::min(mny, model.eval(y).get_numeral_int());
          mxy = std::max(mxy, model.eval(y).get_numeral_int());
        }
      }
      assert(mnx < mxx);
      assert(mny < mxy);
      const Int prio = pNet->priority();
      hpwl += prio * (mxx - mnx + mxy - mny);
    }
  }
  if (useCluster) {
    const PlaceClusterCstr* pCstr;
    Cir_ForEachPlaceClusterCstr(_cir, pCstr, i) {
      Int mnx = MAX_INT, mxx = MIN_INT;
      Int mny = MAX_INT, mxy = MIN_INT;
      const PlaceClusterCstr& clus = *pCstr;
      for (const Cell* pCell : clus.vpCells()) {
        const Cell& c = *pCell;
        const Z3Expr& x = *_vpCellXExprs.at(c.idx());
        const Z3Expr& y = *_vpCellYExprs.at(c.idx());
        const Int w = c.width() / _stepX;
        const Int h = c.height() / _stepY;
        mnx = std::min(mnx, model.eval(x).get_numeral_int());
        mxx = std::max(mxx, model.eval(x).get_numeral_int() + w);
        mny = std::min(mny, model.eval(y).get_numeral_int());
        mxy = std::max(mxy, model.eval(y).get_numeral_int() + h);
      }
      assert(mnx < mxx);
      assert(mny < mxy);
      const Int weight = clus.weight();
      hpwl += weight * (mxx - mnx + mxy - mny);
    }
  }
  return hpwl;
}

Int SmtPlacer::calcArrayGridArea(const PlaceArrayCstr& ac) const
{
  Int area = 0;
  if (ac.numCells()) {
    assert(ac.numPlaceArrayCstrs() == 0);
    for (const Cell* pCell : ac.vpCells()) {
      const Int gw = pCell->width() / _stepX;
      const Int gh = pCell->height() / _stepY;
      area += gw * gh;
    }
  }
  else {
    assert(ac.numPlaceArrayCstrs() > 0);
    for (const PlaceArrayCstr* p : ac.vpPlaceArrayCstrs()) {
      area += calcArrayGridArea(*p);
    }
  }
  return area;
}

Int SmtPlacer::toDBUnit(const Real v) const
{
  assert(_cir.physRes() != 0);
  return std::lround(v / _cir.physRes());
}

void SmtPlacer::setRegResult(const Z3Model& model) 
{
  Int i;
  Region* pReg;

  Cir_ForEachRegion(_cir, pReg, i) {
    Region& reg = *pReg;
    const Int xl = model.eval(*_vpRegXExprs.at(reg.idx())).get_numeral_int() * _stepX;
    const Int yl = model.eval(*_vpRegYExprs.at(reg.idx())).get_numeral_int() * _stepY;
    const Int xh = xl + model.eval(*_vpRegWExprs.at(reg.idx())).get_numeral_int() * _stepX;
    const Int yh = yl + model.eval(*_vpRegHExprs.at(reg.idx())).get_numeral_int() * _stepY;
    reg.setBboxInner(xl, yl, xh, yh);
  }
}

void SmtPlacer::setCellResult(const Z3Model& model)
{
  Int i, j;
  Region* pReg;
  Cell* pCell;

  Cir_ForEachRegion(_cir, pReg, i) {
    Reg_ForEachCell((*pReg), pCell, j) {
      Cell& c = *pCell;
      const Int gx = model.eval(*_vpCellXExprs.at(c.idx())).get_numeral_int();
      const Int gy = model.eval(*_vpCellYExprs.at(c.idx())).get_numeral_int();
      //spdlog::info("{} {} {}", c.name(), gx, gy);
      const Int x = gx * _stepX;
      const Int y = gy * _stepY;
      c.setLoc(x, y, 
               (x - pReg->bboxInner().xl()) / pReg->placeGrid().stepX(), 
               (y - pReg->bboxInner().yl()) / pReg->placeGrid().stepY());

    }
  }
}

void SmtPlacer::setIOPinResult(const Z3Model& model)
{
  Int i;
  Pin* pPin;

  Cir_ForEachIOPin(_cir, pPin, i) {
    const Int gx = model.eval(*_vpIOPinXExprs.at(pPin->ioIdx())).get_numeral_int();
    const Int gy = model.eval(*_vpIOPinYExprs.at(pPin->ioIdx())).get_numeral_int();
    const Int x = gx * _stepX;
    const Int y = gy * _stepY;
    //spdlog::info("{} {} {}", pPin->name(), x, y);

    Box<Int> b;
    switch (_cir.ioPinLoc(i)) {
      case IOPinLocE::left:
        b.set(x - _stepX, y, x, y + _stepY);
        break;
      case IOPinLocE::right:
        b.set(x, y, x + _stepX, y + _stepY);
        break;
      case IOPinLocE::bottom:
        b.set(x, y - _stepY, x + _stepX, y);
        break;
      case IOPinLocE::top:
        b.set(x, y, x + _stepX, y + _stepY);
        break;
      default:
        assert(false);
    }
    _cir.setIOPinBbox(i, b);
  }

}

PROJECT_NAMESPACE_END

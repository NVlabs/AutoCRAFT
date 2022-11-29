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

#pragma once

#include <z3++.h>

#include "db/dbCir.hpp"

PROJECT_NAMESPACE_START

class SmtPlacer {
  using Z3Context    = z3::context;
  using Z3Params     = z3::params;
  using Z3Solver     = z3::solver;
  using Z3Expr       = z3::expr;
  using Z3ExprVector = z3::expr_vector;
  using Z3Tactic     = z3::tactic;
  using Z3Model      = z3::model;
  using Z3CheckRes   = z3::check_result;

  enum Z3SmtArithSolver {
    Z3_bellman_ford = 1, // diff logic (sparse)
    Z3_simplex,          // simplex based
    Z3_floyd_warshell,   // diff logic (dense)
    Z3_utvpi,            // unit two variable per inequality
    Z3_inf_lra,          // infinity lra
    Z3_lra               // default
  };

public:
  SmtPlacer(CirDB& cir,
            const Real ar = 1,
            const Real fcUtil = 0.7,
            //const FlatHashMap<const Region*, Real>& mFcRegUtils = FlatHashMap<const Region*, Real>(),
            const Umap<String, Real>& mFcRegUtils = Umap<String, Real>(),
            const Vector<Real>& vBounds = Vector<Real>(),
            const Int numThreads = 1,
            const Int iter = 1) 
      : _cir(cir),
        _ar(ar),
        _fcUtil(fcUtil),
        _mFcRegUtils(mFcRegUtils),
        _vBounds(vBounds),
        _z3Threads(numThreads),
        _maxHpwlOptIter(iter),
        _vStepXFactors(_cir.numRegions()),
        _vStepYFactors(_cir.numRegions()),
        _vpRegXExprs(_cir.numRegions()),
        _vpRegYExprs(_cir.numRegions()),
        _vpRegWExprs(_cir.numRegions()),
        _vpRegHExprs(_cir.numRegions()),
        _vpCellXExprs(_cir.numCells()),
        _vpCellYExprs(_cir.numCells()),
        _vpSymAxisExprs(_cir.numPlaceSymCstrs()),
        _vpArrayMinXExprs(_cir.numPlaceArrayCstrs()),
        _vpArrayMaxXExprs(_cir.numPlaceArrayCstrs()),
        _vpArrayMinYExprs(_cir.numPlaceArrayCstrs()),
        _vpArrayMaxYExprs(_cir.numPlaceArrayCstrs()),
        _vpClusterMinXExprs(_cir.numPlaceClusterCstrs()),
        _vpClusterMaxXExprs(_cir.numPlaceClusterCstrs()),
        _vpClusterMinYExprs(_cir.numPlaceClusterCstrs()),
        _vpClusterMaxYExprs(_cir.numPlaceClusterCstrs()),
        _vpNetMinXExprs(_cir.numNets()),
        _vpNetMaxXExprs(_cir.numNets()),
        _vpNetMinYExprs(_cir.numNets()),
        _vpNetMaxYExprs(_cir.numNets()),
        _vpNetHpwlExprs(_cir.numNets()),
        _vvpIOLeftSlotExprs(_cir.numLeftIOPins()),
        _vvpIORightSlotExprs(_cir.numRightIOPins()),
        _vvpIOBottomSlotExprs(_cir.numBottomIOPins()),
        _vvpIOTopSlotExprs(_cir.numTopIOPins()),
        _vpIOPinXExprs(_cir.numIOPins()),
        _vpIOPinYExprs(_cir.numIOPins()),
        _vpPinDensityExprs(_cir.numCells())
  {
  }
  ~SmtPlacer() {}

  bool solve();

  void setAR(const Real ar)           { _ar = ar; }
  void setFcInfl(const Real f)        { _fcInfl = f; }
  void setFcShrink(const Real f)      { _fcShrink = f; }
  void setFcHpwlOpt(const Real f)     { _fcHpwlOpt = f; }
  void setFcUtil(const Real f)        { _fcUtil = f; }
  void setFcAreaExpand(const Real f)  { _fcAreaExpand = f; }

private:
  CirDB& _cir;

  Real                                    _ar;    // aspect ratio (W/H)
  Real                                    _fcInfl        = 1.2;  // inflation factor
  Real                                    _fcShrink      = 0.95; // shrink factor
  Real                                    _fcHpwlOpt     = 0.1;
  Real                                    _fcUtil        = 0.7;
  Real                                    _fcAreaExpand  = 2; // reserve spaces for edge cells 
  //const FlatHashMap<const Region*, Real>& _mFcRegUtils;
  const Umap<String, Real>&               _mFcRegUtils;
  const Vector<Real>&                     _vBounds;

  const bool  _z3AutoConfig         = true;
  const bool  _z3Model              = true;
  const bool  _z3UnsatCore          = true;
  const Uint  _z3Timeout            = 0;
  const Int   _z3Verbose            = 1;
  const Int   _z3Threads            ; // not neccessary faster
  const Int   _z3SmtArithSolver     = Z3_simplex;
  const Int   _z3SatRestartMax      = 400000;
  Int         _z3XBvSize;
  Int         _z3YBvSize;
  Int         _z3HpwlBvSize;
  Int         _z3PinCntBvSize;

  Int         _stepX = 0;
  Int         _stepY = 0;
  Int         _maxHpwlOptIter;
  Vector<Int> _vStepXFactors;
  Vector<Int> _vStepYFactors;

  Z3Context                  _ctx;
  Vector<UniquePtr<Z3Expr>>  _vpRegXExprs; // lower left x of each region (inner)
  Vector<UniquePtr<Z3Expr>>  _vpRegYExprs; // lower left y of each region (inner)
  Vector<UniquePtr<Z3Expr>>  _vpRegWExprs; // width of each region (inner)
  Vector<UniquePtr<Z3Expr>>  _vpRegHExprs; // height of each region (inner)
  Vector<UniquePtr<Z3Expr>>  _vpCellXExprs; // lower left x of cells
  Vector<UniquePtr<Z3Expr>>  _vpCellYExprs; // lower left y of cells 
  Vector<UniquePtr<Z3Expr>>  _vpSymAxisExprs;
  Vector<UniquePtr<Z3Expr>>  _vpArrayMinXExprs;
  Vector<UniquePtr<Z3Expr>>  _vpArrayMaxXExprs;
  Vector<UniquePtr<Z3Expr>>  _vpArrayMinYExprs;
  Vector<UniquePtr<Z3Expr>>  _vpArrayMaxYExprs;
  Vector<UniquePtr<Z3Expr>>  _vpClusterMinXExprs;
  Vector<UniquePtr<Z3Expr>>  _vpClusterMaxXExprs;
  Vector<UniquePtr<Z3Expr>>  _vpClusterMinYExprs;
  Vector<UniquePtr<Z3Expr>>  _vpClusterMaxYExprs;
  Vector<UniquePtr<Z3Expr>>  _vpNetMinXExprs;
  Vector<UniquePtr<Z3Expr>>  _vpNetMaxXExprs;
  Vector<UniquePtr<Z3Expr>>  _vpNetMinYExprs;
  Vector<UniquePtr<Z3Expr>>  _vpNetMaxYExprs;
  Vector<UniquePtr<Z3Expr>>  _vpNetHpwlExprs;
  UniquePtr<Z3Expr>          _pTotHpwlExpr; // not var

  // io pins
  Vector<Vector<UniquePtr<Z3Expr>>> _vvpIOLeftSlotExprs; // bool const
  Vector<Vector<UniquePtr<Z3Expr>>> _vvpIORightSlotExprs; // bool const
  Vector<Vector<UniquePtr<Z3Expr>>> _vvpIOBottomSlotExprs; // bool const
  Vector<Vector<UniquePtr<Z3Expr>>> _vvpIOTopSlotExprs; // bool const
  Vector<UniquePtr<Z3Expr>>         _vpIOPinXExprs;
  Vector<UniquePtr<Z3Expr>>         _vpIOPinYExprs;
  //const Int _maxIOSlotOverlap = 2;
  const Int _maxIOSlotOverlap = 3;
  
  // for different power domains in the same region
  Vector<Vector<UniquePtr<Z3Expr>>>   _vvpCellGroupYBoundExprs;

  // routability
  const Int _winX = 3;
  const Int _winY = 2;
  const Int _maxPinCnts = 16;
  Vector<Array2d<UniquePtr<Z3Expr>>> _vpPinDensityExprs; // bool const, size = numCells * xh * yh

  // init
  void setStep();
  void setZ3Config();
  void setZ3BvSize(Int xh, Int yh); // xh, yh: design right/top boundaries
  void setZ3SolverParams(Z3Solver& solver);

  // variables
  void addZ3RegVarExprs();
  void addZ3CellVarExprs();
  void addZ3NetVarExprs();
  void addZ3IOPinVarExprs(const Int xl, const Int yl, const Int xh, const Int yh);
  void addZ3PinDensityVarExprs(const Int xl, const Int yl, const Int xh, const Int yh);
  // region constraints
  void addZ3CstrRegBoundaryLower(Z3Solver& solver, const Int xl, const Int yl);
  void addZ3CstrRegBoundaryUpper(Z3Solver& solver, const Int xh, const Int yh);
  void addZ3CstrRegSize(Z3Solver& solver, const bool isEvenRow); // auto, inner size
  void addZ3CstrRegSize(Z3Solver& solver, const Region& reg, const Int xSize, const Int ySize); // inner size
  void addZ3CstrRegNonOverlap(Z3Solver& solver);
  void addZ3CstrRegAlign(Z3Solver& solver);
  // cell constraints
  void addZ3CstrCellBoundaryLower(Z3Solver& solver, const Region& reg);
  void addZ3CstrCellBoundaryUpper(Z3Solver& solver, const Region& reg);
  void addZ3CstrCellNonOverlap(Z3Solver& solver, const Region& reg);
  void addZ3CstrCellArray(Z3Solver& solver, const Region& reg);
  void addZ3CstrCellArraySize(Z3Solver& solver, const PlaceArrayCstr& ac, const Int xSize, const Int ySize);
  void addZ3CstrCellSymmetry(Z3Solver& solver, const Region& reg);
  void addZ3CstrCellCluster(Z3Solver& solver);
  void addZ3CstrCellPowerAbutment(Z3Solver& solver, const Region& reg);
  void addZ3CstrCellPrePlace(Z3Solver& solver);
  void addZ3CstrCellEdgeDist(Z3Solver& solver, const Region& reg);
  void addZ3CstrCellOrder(Z3Solver& solver);
  void addZ3CstrCellAlign(Z3Solver& solver);
  void addZ3CstrCellDisjoint(Z3Solver& solver);
  void addZ3CstrCellRow(Z3Solver& solver);
  // array constraints
  void addZ3CstrArrayArray(Z3Solver& solver, const Region& reg);
  void addZ3CstrArrayNonOverlap(Z3Solver& solver, const Region& reg);
  // io pin constraints
  void addZ3CstrIOPinSlot(Z3Solver& solver, const Int xl, const Int yl, const Int xh, const Int yh);
  void addZ3CstrIOPinLoc(Z3Solver& solver, const Int xl, const Int yl, const Int xh, const Int yh);
  // net constraints
  void addZ3CstrNetHpwl(Z3Solver& solver, const bool useIO);
  void addZ3CstrNetSymDis(Z3Solver& solver);
  // routability constraints
  void addZ3CstrExtension(Z3Solver& solver);
  void addZ3CstrRoutePinDensity(Z3Solver& solver, const Int xl, const Int yl, const Int xh, const Int yh, const Int wsizeX, const Int wsizeY);

  // incremental solving
  void freezeZ3RegSizeResult(Z3Solver& solver, const Z3Model& model) const;
  void freezeZ3RegSizeResult(Z3Solver& solver, const Z3Model& model, const Region& reg) const;
  void freezeZ3RegResult(Z3Solver& solver, const Z3Model& model) const;
  void freezeZ3RegResult(Z3Solver& solver, const Z3Model& model, const Region& reg) const;
  void freezeZ3CellResult(Z3Solver& solver, const Z3Model& model) const;
  void freezeZ3CellResult(Z3Solver& solver, const Z3Model& model, const Region& reg) const;
  void freezeZ3CellResult(Z3Solver& solver, const Z3Model& model, const Cell& cell) const;
  void freezeZ3IOPinResult(Z3Solver& solver, const Z3Model& model) const;
  
  bool checkNUpdate(Z3Solver& solver, Z3Model& model, const bool useIO, const bool useCluster) const;
  bool wlOpt(const Int iter, Z3Solver& solver, Z3Model& model, const Real fcHpwlOpt, const bool useIO, const bool useCluster);


  void genHpwlCandidates(const Int curHpwl, const Int tarHpwl, const Int num, Vector<Int>& vHpwlCandidates) const;

  // helper funcs
  Real           calcExpectedDesignArea(const Real fcAreaExpand, const Real fcUtil) const; // gridded area
  Int            calcExpectedW(const Real area) const;
  Int            calcExpectedH(const Real area) const;
  Pair<Int, Int> calcGridStep() const;
  void           calcFactors(const Int n, Vector<Pair<Int, Int>>& v) const;
  void           calcApproxFactors(const Int n, const bool isEvenRow, Vector<Pair<Int, Int>>& v) const;
  Int            calcLog2(Int x) const;
  Int            calcZ3NetHpwl(const Z3Model& model, const bool useIO, const bool useCluster) const;
  Int            calcArrayGridArea(const PlaceArrayCstr& ac) const;
  Int            toDBUnit(const Real v) const;

  // save
  void setRegResult(const Z3Model& model);
  void setCellResult(const Z3Model& model);
  void setIOPinResult(const Z3Model& model);

};

PROJECT_NAMESPACE_END

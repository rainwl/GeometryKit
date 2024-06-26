// This file is part of libigl, a simple c++ geometry processing library.
// 
// Copyright (C) 2024 Alec Jacobson <alecjacobson@gmail.com>
// 
// This Source Code Form is subject to the terms of the Mozilla Public License 
// v. 2.0. If a copy of the MPL was not distributed with this file, You can 
// obtain one at http://mozilla.org/MPL/2.0/
#include "find_intersections.h"
#include "find_self_intersections.h"

template <
  typename DerivedV,
  typename DerivedF,
  typename DerivedIF,
  typename DerivedCP>
IGL_INLINE bool igl::predicates::find_self_intersections(
  const Eigen::MatrixBase<DerivedV> & V,
  const Eigen::MatrixBase<DerivedF> & F,
  const bool first_only,
  Eigen::PlainObjectBase<DerivedIF> & IF,
  Eigen::PlainObjectBase<DerivedCP> & CP)
{
  // This is really just a wrapper around fast_find_intersections which will
  // internally detect that V,F are the second set
  return find_intersections( V,F,V,F,first_only,IF,CP);
}

template <
  typename DerivedV,
  typename DerivedF,
  typename DerivedIF,
  typename DerivedCP,
  typename DerivedEV,
  typename DerivedEE,
  typename DerivedEI>
bool igl::predicates::find_self_intersections(
  const Eigen::MatrixBase<DerivedV> & V,
  const Eigen::MatrixBase<DerivedF> & F,
  Eigen::PlainObjectBase<DerivedIF> & IF,
  Eigen::PlainObjectBase<DerivedCP> & CP,
  Eigen::PlainObjectBase<DerivedEV> & EV,
  Eigen::PlainObjectBase<DerivedEE> & EE,
  Eigen::PlainObjectBase<DerivedEI> & EI)
{
  return find_intersections( V,F,V,F,IF,CP,EV,EE,EI);
}

#ifdef IGL_STATIC_LIBRARY
// Explicit template instantiation
// generated by autoexplicit.sh
template bool igl::predicates::find_self_intersections<Eigen::Matrix<double, -1, -1, 0, -1, -1>, Eigen::Matrix<int, -1, -1, 0, -1, -1>, Eigen::Matrix<int, -1, -1, 0, -1, -1>, Eigen::Array<bool, -1, 1, 0, -1, 1>, Eigen::Matrix<double, -1, -1, 0, -1, -1>, Eigen::Matrix<int, -1, -1, 0, -1, -1>, Eigen::Matrix<int, -1, 1, 0, -1, 1> >(Eigen::MatrixBase<Eigen::Matrix<double, -1, -1, 0, -1, -1> > const&, Eigen::MatrixBase<Eigen::Matrix<int, -1, -1, 0, -1, -1> > const&, Eigen::PlainObjectBase<Eigen::Matrix<int, -1, -1, 0, -1, -1> >&, Eigen::PlainObjectBase<Eigen::Array<bool, -1, 1, 0, -1, 1> >&, Eigen::PlainObjectBase<Eigen::Matrix<double, -1, -1, 0, -1, -1> >&, Eigen::PlainObjectBase<Eigen::Matrix<int, -1, -1, 0, -1, -1> >&, Eigen::PlainObjectBase<Eigen::Matrix<int, -1, 1, 0, -1, 1> >&);
// generated by autoexplicit.sh
template bool igl::predicates::find_self_intersections<Eigen::Matrix<double, -1, -1, 0, -1, -1>, Eigen::Matrix<int, -1, -1, 0, -1, -1>, Eigen::Matrix<int, -1, -1, 0, -1, -1>, Eigen::Array<bool, -1, 1, 0, -1, 1> >(Eigen::MatrixBase<Eigen::Matrix<double, -1, -1, 0, -1, -1> > const&, Eigen::MatrixBase<Eigen::Matrix<int, -1, -1, 0, -1, -1> > const&, bool, Eigen::PlainObjectBase<Eigen::Matrix<int, -1, -1, 0, -1, -1> >&, Eigen::PlainObjectBase<Eigen::Array<bool, -1, 1, 0, -1, 1> >&);
#endif

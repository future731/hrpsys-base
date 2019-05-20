// -*- C++ -*-
/*!
 * @file  eiquadprogWrapper.cpp
 * @brief for eiquadprog
 * @date  $Date$
 *
 * $Id$
 */

#ifdef USE_EIQUADPROG_IN_SEQUENCER
#include "qpWrapper.h"
#include "eiquadprog.h"

double solve_strict_qp(
        const boost::shared_ptr<std::ofstream>& ofs_bsp_debug,
        // const hrp::dvector& initial_state,
        const hrp::dvector& state_min_vector,
        const hrp::dvector& state_max_vector,
        const hrp::dmatrix& eval_weight_matrix,
        const hrp::dvector& eval_coeff_vector,
        const hrp::dmatrix& equality_matrix,
        const hrp::dvector& equality_coeff_vector,
        const hrp::dmatrix& inequality_matrix,
        const hrp::dvector& inequality_min_vector,
        const hrp::dvector& inequality_max_vector,
        hrp::dvector& result_vector
        )
{
  // eiquadprog
  hrp::dmatrix G = eval_weight_matrix;
  hrp::dvector g0 = eval_coeff_vector;
  hrp::dmatrix CE = equality_matrix;
  hrp::dvector ce0 = -equality_coeff_vector; // sign inversion is needed; in eus interface, equality-coeff signature is inverted when passing it to c++ eiquagprog source
  int c = state_min_vector.size();
  hrp::dmatrix CI = hrp::dmatrix::Zero(2 * (c - 1) + 2 * c, c);
  CI.block(0, 0, c, c) = hrp::dmatrix::Identity(c, c);
  CI.block(c, 0, c, c) = -hrp::dmatrix::Identity(c, c);
  CI.block(c * 2, 0, c - 1, c) = inequality_matrix;
  CI.block(c * 2 + c - 1, 0, c - 1, c) = -inequality_matrix;
  hrp::dvector ci0 = hrp::dvector::Zero(2 * (c - 1) + 2 * c);
  ci0.segment(0, c) = -state_min_vector;
  ci0.segment(c, c) = state_max_vector;
  ci0.segment(c * 2, c - 1) = -inequality_min_vector;
  ci0.segment(c * 2 + c - 1, c - 1) = inequality_max_vector;
  double opt = Eigen::solve_quadprog(G, g0, CE.transpose(), ce0, CI.transpose(), ci0, result_vector);
  return opt;
}
#endif // USE_EIQUADPROG_IN_SEQUENCER

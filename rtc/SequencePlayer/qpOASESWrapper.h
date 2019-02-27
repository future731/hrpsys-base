// -*- C++ -*-
/*!
 * @file  qpOASESWrapper.h
 * @brief for qpOASES
 * @date  $Date$
 *
 * $Id$
 */
#ifndef QPOASESWRAPPER_H
#define QPOASESWRAPPER_H

#include <fstream>
#include <boost/shared_ptr.hpp>
#include <qpOASES.hpp>
#include <Eigen/Core>
#include <hrpUtil/EigenTypes.h>

/*
 * minimize 1/2 x^T H x + x^T g
 * subject to lb <= x <= ub
 *            lba <= Ax <= ubA
 */

void setUpQPParam(qpOASES::real_t* raw_mat, const hrp::dmatrix& hrp_dmat);

void setUpQPParam(qpOASES::real_t* raw_arr, const hrp::dvector& hrp_dvec);

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
        );

double solve_mild_qp(
        const boost::shared_ptr<std::ofstream>& ofs_bsp_debug,
        // const hrp::dvector& initial_state,
        const hrp::dvector& state_min_vector,
        const hrp::dvector& state_max_vector,
        const hrp::dmatrix& eval_weight_matrix,
        const hrp::dvector& eval_coeff_vector,
        const hrp::dmatrix& equality_matrix,
        const hrp::dvector& equality_coeff_vector,
        const hrp::dmatrix& equality_weight_matrix,
        const hrp::dmatrix& inequality_matrix,
        const hrp::dvector& inequality_min_vector,
        const hrp::dvector& inequality_max_vector,
        hrp::dvector& result_vector
        );

#endif

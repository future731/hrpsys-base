// -*- C++ -*-
/*!
 * @file  qpWrapper.h
 * @brief qp library selector
 * @date  $Date$
 *
 * $Id$
 */

#ifndef QPWRAPPER_H
#define QPWRAPPER_H
#include <boost/shared_ptr.hpp>
#include <hrpUtil/EigenTypes.h>
#include <fstream>
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

/*
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
*/
#endif

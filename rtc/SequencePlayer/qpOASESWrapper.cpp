#include "qpOASESWrapper.h"
#include <iostream>

void setUpQPParam(qpOASES::real_t* raw_mat, const hrp::dmatrix& hrp_dmat)
{
    for (int i = 0; i < hrp_dmat.rows(); i++) {
        for (int j = 0; j < hrp_dmat.cols(); j++) {
            raw_mat[i * hrp_dmat.cols() + j] = hrp_dmat(i, j);
        }
    }
}

void setUpQPParam(qpOASES::real_t* raw_arr, const hrp::dvector& hrp_dvec)
{
    for (int i = 0; i < hrp_dvec.size(); i++) {
        raw_arr[i] = hrp_dvec[i];
    }
}

double solve_strict_qp(
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
  int x_size = eval_weight_matrix.rows();
  int eq_size = equality_coeff_vector.size();
  int ieq_size = inequality_min_vector.size();
  using namespace qpOASES;
  boost::shared_ptr<real_t> H_shptr(new real_t[x_size * x_size]);
  real_t* H = H_shptr.get();
  setUpQPParam(H, eval_weight_matrix);

  boost::shared_ptr<real_t> g_shptr(new real_t[x_size]);
  real_t* g = g_shptr.get();
  setUpQPParam(g, eval_coeff_vector);

  boost::shared_ptr<real_t> A_shptr(new real_t[(eq_size + ieq_size) * x_size]);
  Eigen::MatrixXd A_Eigen(eq_size + ieq_size, x_size);
  A_Eigen << equality_matrix, inequality_matrix;
  real_t* A = A_shptr.get();
  setUpQPParam(A, A_Eigen);

  boost::shared_ptr<real_t> lb_shptr(new real_t[x_size]);
  real_t* lb = lb_shptr.get();
  setUpQPParam(lb, state_min_vector);

  boost::shared_ptr<real_t> ub_shptr(new real_t[x_size]);
  real_t* ub = ub_shptr.get();
  setUpQPParam(ub, state_max_vector);

  boost::shared_ptr<real_t> lbAx_shptr(new real_t[eq_size + ieq_size]);
  Eigen::VectorXd lbAx_Eigen(eq_size + ieq_size);
  lbAx_Eigen << equality_coeff_vector, inequality_min_vector;
  real_t* lbAx = lbAx_shptr.get();
  setUpQPParam(lbAx, lbAx_Eigen);

  boost::shared_ptr<real_t> ubAx_shptr(new real_t[eq_size + ieq_size]);
  Eigen::VectorXd ubAx_Eigen(eq_size + ieq_size);
  ubAx_Eigen << equality_coeff_vector, inequality_max_vector;
  real_t* ubAx = ubAx_shptr.get();
  setUpQPParam(ubAx, ubAx_Eigen);

  QProblem qp(x_size, eq_size + ieq_size);
  Options options;
  options.printLevel = PL_NONE;
  qp.setOptions(options);
  sparse_int_t nWSR = 100;
  real_t max_cputime_sec[1] = { 1e-4 };
  returnValue qp_return_value = qp.init(H, g, A, lb, ub, lbAx, ubAx, nWSR, max_cputime_sec);
  if (qp_return_value == SUCCESSFUL_RETURN) {
    std::cout << "qp succeeded" << std::endl;
  } else if (qp_return_value == RET_INIT_FAILED) {
    std::cout << "qp init failed" << std::endl;
    return std::numeric_limits<double>::infinity();
  } else if (qp_return_value == RET_INIT_FAILED_CHOLESKY) {
    std::cout << "qp init failed cholesky" << std::endl;
    return std::numeric_limits<double>::infinity();
  } else if (qp_return_value == RET_INIT_FAILED_TQ) {
    std::cout << "qp init failed tq" << std::endl;
    return std::numeric_limits<double>::infinity();
  } else if (qp_return_value == RET_INIT_FAILED_HOTSTART) {
    std::cout << "qp init failed hotstart" << std::endl;
    return std::numeric_limits<double>::infinity();
  } else if (qp_return_value == RET_INIT_FAILED_UNBOUNDEDNESS) {
    std::cout << "qp init failed unboundness" << std::endl;
    return std::numeric_limits<double>::infinity();
  } else if (qp_return_value == RET_MAX_NWSR_REACHED){
    std::cout << "qp max nwsr reached" << std::endl;
    return std::numeric_limits<double>::infinity();
  } else if (qp_return_value == RET_INVALID_ARGUMENTS) {
    std::cout << "qp init invalid arguments" << std::endl;
    return std::numeric_limits<double>::infinity();
  } else {
    std::cout << "unknown qp status: " << qp_return_value << std::endl;
    return std::numeric_limits<double>::infinity();
  }
  boost::shared_ptr<real_t> x_opt_shptr(new real_t[x_size]);
  real_t* x_opt = x_opt_shptr.get();
  returnValue primal_solution_value = qp.getPrimalSolution(x_opt);
  if (primal_solution_value == SUCCESSFUL_RETURN) {
      std::cout << "qp primal solution succeeded" << std::endl;
  } else if (primal_solution_value == RET_QP_NOT_SOLVED) {
      std::cout << "qp not solved" << std::endl;
  } else {
    std::cout << "unknown qp primal solution status: " << primal_solution_value << std::endl;
  }
  for (int i = 0; i < result_vector.size(); i++) {
      result_vector[i] = x_opt[i];
  }
  return qp.getObjVal();
}

// equality condition is eased
double solve_mild_qp(
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
        )
{
  int x_size = eval_weight_matrix.rows();
  int eq_size = equality_coeff_vector.size();
  int ieq_size = inequality_min_vector.size();
  using namespace qpOASES;
  boost::shared_ptr<real_t> A_shptr(new real_t[eq_size * x_size]);
  Eigen::MatrixXd A_Eigen(eq_size, x_size);
  A_Eigen << equality_matrix;
  real_t* A = A_shptr.get();
  setUpQPParam(A, A_Eigen);

  boost::shared_ptr<real_t> H_shptr(new real_t[x_size * x_size]);
  real_t* H = H_shptr.get();
  Eigen::MatrixXd H_Eigen = eval_weight_matrix + A_Eigen.transpose() * equality_weight_matrix * A_Eigen;
  setUpQPParam(H, H_Eigen);

  boost::shared_ptr<real_t> g_shptr(new real_t[x_size]);
  real_t* g = g_shptr.get();
  Eigen::VectorXd g_Eigen = eval_coeff_vector - 2 * A_Eigen.transpose() * equality_weight_matrix.transpose() * equality_coeff_vector;
  setUpQPParam(g, g_Eigen);

  boost::shared_ptr<real_t> lb_shptr(new real_t[x_size]);
  real_t* lb = lb_shptr.get();
  setUpQPParam(lb, state_min_vector);

  boost::shared_ptr<real_t> ub_shptr(new real_t[x_size]);
  real_t* ub = ub_shptr.get();
  setUpQPParam(ub, state_max_vector);

  boost::shared_ptr<real_t> B_shptr(new real_t[ieq_size * x_size]);
  Eigen::MatrixXd B_Eigen(ieq_size, x_size);
  B_Eigen << inequality_matrix;
  real_t* B = B_shptr.get();
  setUpQPParam(B, B_Eigen);

  boost::shared_ptr<real_t> lbBx_shptr(new real_t[ieq_size]);
  Eigen::VectorXd lbBx_Eigen(ieq_size);
  lbBx_Eigen << inequality_min_vector;
  real_t* lbBx = lbBx_shptr.get();
  setUpQPParam(lbBx, lbBx_Eigen);

  boost::shared_ptr<real_t> ubBx_shptr(new real_t[ieq_size]);
  Eigen::VectorXd ubBx_Eigen(ieq_size);
  ubBx_Eigen << inequality_max_vector;
  real_t* ubBx = ubBx_shptr.get();
  setUpQPParam(ubBx, ubBx_Eigen);

  QProblem qp(x_size, ieq_size);
  Options options;
  options.printLevel = PL_NONE;
  qp.setOptions(options);
  sparse_int_t nWSR = 100000000;
  real_t max_cputime_sec[1] = { 1 };
  returnValue qp_return_value = qp.init(H, g, B, lb, ub, lbBx, ubBx, nWSR, max_cputime_sec);
  if (qp_return_value == SUCCESSFUL_RETURN) {
    std::cout << "qp succeeded" << std::endl;
  } else if (qp_return_value == RET_INIT_FAILED) {
    std::cout << "qp init failed" << std::endl;
    return std::numeric_limits<double>::infinity();
  } else if (qp_return_value == RET_INIT_FAILED_CHOLESKY) {
    std::cout << "qp init failed cholesky" << std::endl;
    return std::numeric_limits<double>::infinity();
  } else if (qp_return_value == RET_INIT_FAILED_TQ) {
    std::cout << "qp init failed tq" << std::endl;
    return std::numeric_limits<double>::infinity();
  } else if (qp_return_value == RET_INIT_FAILED_HOTSTART) {
    std::cout << "qp init failed hotstart" << std::endl;
    return std::numeric_limits<double>::infinity();
  } else if (qp_return_value == RET_INIT_FAILED_UNBOUNDEDNESS) {
    std::cout << "qp init failed unboundness" << std::endl;
    return std::numeric_limits<double>::infinity();
  } else if (qp_return_value == RET_MAX_NWSR_REACHED){
    std::cout << "qp max nwsr reached" << std::endl;
    return std::numeric_limits<double>::infinity();
  } else if (qp_return_value == RET_INVALID_ARGUMENTS) {
    std::cout << "qp init invalid arguments" << std::endl;
    return std::numeric_limits<double>::infinity();
  } else {
    std::cout << "unknown qp status: " << qp_return_value << std::endl;
    return std::numeric_limits<double>::infinity();
  }
  boost::shared_ptr<real_t> x_opt_shptr(new real_t[x_size]);
  real_t* x_opt = x_opt_shptr.get();
  returnValue primal_solution_value = qp.getPrimalSolution(x_opt);
  if (primal_solution_value == SUCCESSFUL_RETURN) {
      std::cout << "qp primal solution succeeded" << std::endl;
  } else if (primal_solution_value == RET_QP_NOT_SOLVED) {
      std::cout << "qp not solved" << std::endl;
  } else {
    std::cout << "unknown qp primal solution status: " << primal_solution_value << std::endl;
  }
  for (int i = 0; i < result_vector.size(); i++) {
      result_vector[i] = x_opt[i];
  }
  return qp.getObjVal();
}

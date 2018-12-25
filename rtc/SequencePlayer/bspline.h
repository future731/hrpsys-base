#pragma once

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Dense>
#include <algorithm>
#include <boost/shared_ptr.hpp>
#include <cmath>
#include <iostream>
#include <vector>

#include <hrpUtil/EigenTypes.h>

namespace hrp
{

// {{{ pseudoInverse
template <typename t_matrix>
t_matrix pseudoInverse(const t_matrix& m, const double& tolerance = 1.e-6)
{
  using namespace Eigen;
  typedef JacobiSVD<t_matrix> TSVD;
  unsigned int svd_opt(ComputeThinU | ComputeThinV);
  if (m.RowsAtCompileTime != Dynamic || m.ColsAtCompileTime != Dynamic)
    svd_opt = ComputeFullU | ComputeFullV;
  TSVD svd(m, svd_opt);
  const typename TSVD::SingularValuesType& sigma(svd.singularValues());
  typename TSVD::SingularValuesType sigma_inv(sigma.size());
  for (long i = 0; i < sigma.size(); ++i)
  {
    if (sigma(i) > tolerance)
      sigma_inv(i) = 1.0 / sigma(i);
    else
      sigma_inv(i) = 0.0;
  }
  return svd.matrixV() * sigma_inv.asDiagonal() * svd.matrixU().transpose();
}
// }}}

}  // namespace hrp

namespace BSpline
{
// {{{ calcTVector
hrp::dmatrix calcTVector(const double& t, const bool& is_toplevel, const hrp::dvector& buf, const int& n = 2, const int& depth = 0, const double& t_nth_power = 1);
// }}}

// {{{ calcBSplineCoeffVector
hrp::dvector calcBSplineCoeffVector(
    const double& t,
    const int& recursive_order,
    const int& recursive_cnt,
    const int& id = 0,
    const int& id_max = 8,
    const double& t_min = 0.0,
    const double& t_max = 1.0);
// }}}

// {{{ calcMatrixLinearEquationCoeffMatrix
// A[p0,...,pM-1]B = {a_ik*p_kl*b_lj}ixj => C[P0;...;pM-1]
hrp::dmatrix calcMatrixLinearEquationCoeffMatrix(
    const hrp::dmatrix& left_matrix, const hrp::dmatrix& right_matrix,
    const int& col, const int& row, const std::vector<int>& col_list);
// }}}

// {{{ solveMatrixLinearEquation
// A[p0,...,pM-1]B = C[P0;...;pM-1] = D => return P
hrp::dmatrix solveMatrixLinearEquation(
    const hrp::dmatrix& left_matrix, const hrp::dmatrix& right_matrix,
    const int& col, const int& row, const hrp::dmatrix& answer_matrix);
// }}}

// {{{ calcIntegralxTxMatrix
hrp::dmatrix calcIntegralxTxMatrix(const double& t, const bool& is_toplevel, const hrp::dmatrix& buf, const int& n, const int& depth, const double& t_power_n);
// }}}

// {{{ calcDeltaCoeffMatrixForTimeVector
hrp::dmatrix calcDeltaCoeffMatrixForTimeVector(const int& n);
// }}}

class BSplineElement
{
public:
  // {{{ Constructor
  explicit BSplineElement(const int& recursive_order = 4, const int& recursive_cnt = 4,
                          const int& id = 0, const int& id_max = 8,
                          const double& t_min = 0.0, const double& t_max = 1.0)
    : recursive_order_(recursive_order), recursive_cnt_(recursive_cnt), id_(id), id_max_(id_max), t_min_(t_min), t_max_(t_max)
  {
    t_step_ = (t_max_ - t_min_) * 1.0 / (id_max_ - recursive_order_);
    double t_cnt = t_min_ + t_step_ / 2.0;
    coeff_matrix_ = hrp::dmatrix::Zero(0, recursive_cnt_ + 1);
    while (t_cnt <= t_max_)
    {
      // updating coeff matrix
      hrp::dmatrix updated_coeff_matrix = hrp::dmatrix::Zero(coeff_matrix_.rows() + 1, coeff_matrix_.cols());
      for (int i = 0; i < coeff_matrix_.rows() + 1; i++)
      {
        if (i == coeff_matrix_.rows())
        {
          updated_coeff_matrix.row(i) = calcBSplineCoeffVector(t_cnt, recursive_order_, recursive_cnt_, id, id_max, t_min, t_max);
        }
        else
        {
          updated_coeff_matrix.row(i) = coeff_matrix_.row(i);
        }
      }
      coeff_matrix_ = updated_coeff_matrix;
      t_cnt += t_step_;
    }
  }
  // }}}

  // {{{ coeffVector
  hrp::dvector coeffVector(const double& t)
  {
    int id = std::floor((t - t_min_) / t_step_);
    if (id < 0 or coeff_matrix_.rows() <= id)
    {
      std::cerr << "Error in function " << __FUNCTION__ << std::endl;
      std::cerr << "id: " << id << ", coeff_matrix_: " << std::endl;
      std::cerr << coeff_matrix_ << std::endl;
    }
    id = std::max(0, std::min(static_cast<int>(coeff_matrix_.rows()) - 1, id));
    return coeff_matrix_.row(id);
  }
  // }}}

  // {{{ calc
  double calc(const double& t)
  {
    hrp::dvector k = this->coeffVector(t);
    hrp::dvector v = calcTVector(t, true, hrp::dvector::Zero(0), recursive_order_, 0, 1);
    int n = k.size();
    int m = v.size();
    double ret = 0;
    for (int i = 0; i < n; i++)
    {
      ret += k[n - i - 1] * v[m - i - 1];
    }
    return ret;
  }
  // }}}

  // {{{ coeffMatrix getter
  const hrp::dmatrix& coeffMatrix() const
  {
    return coeff_matrix_;
  }
  // }}}

private:
  hrp::dmatrix coeff_matrix_;
  int recursive_order_;
  int recursive_cnt_;
  int id_;
  int id_max_;
  double t_min_;
  double t_max_;
  double t_step_;
};

// using enum because enum class is available only after C++11
enum DerivativeMode
{
  FORWARD,
  BACKWARD,
  AVERAGE,
};

struct DiscreteDeltaMatrix
{
  DerivativeMode mode;
  hrp::dmatrix matrix;
};

bool compFunc(double a, double b);

class BSpline
{
public:
  // {{{ Constructor
  explicit BSpline(const int& recursive_order = 4,
                   const int& recursive_cnt = 4,
                   const int& id = 8, const int& id_max = 8,
                   const double& t_min = 0.0, const double& t_max = 1.0)
    : recursive_order_(recursive_order), recursive_cnt_(recursive_cnt), id_(id), id_max_(id_max), t_min_(t_min), t_max_(t_max), delta_matrix_(hrp::dmatrix::Zero(0, 0)), is_delta_bspline_initialized_(false)
  {
    t_step_ = (t_max_ - t_min_) / (id_max_ - recursive_order_);
    bspline_elements_.clear();
    for (int i = 0; i < id_; i++)
    {
      bspline_elements_.push_back(BSplineElement(recursive_order_, recursive_cnt_, i, id_max_, t_min_, t_max_));
      discrete_delta_matrix_.clear();
    }
  }
  // }}}

  // {{{ calcIntegralObjectiveCoeffMatrix
  hrp::dmatrix calcIntegralObjectiveCoeffMatrix(const int& n)
  {
    hrp::dmatrix d = calcDeltaCoeffMatrixForTimeVector(recursive_order_);
    hrp::dmatrix d_power_n = d;
    hrp::dmatrix ret = hrp::dmatrix::Zero(id_, id_);
    hrp::dmatrix ttte_ttts = hrp::dmatrix::Zero(recursive_order_ + 1, recursive_order_ + 1);
    for (int i = 0; i < n - 1; i++)
    {
      d_power_n *= d;
    }
    double ts = t_min_;
    double te = t_min_ + t_step_;
    while ((ts + te) / 2.0 <= t_max_)
    {
      hrp::dmatrix m_j = this->calcCoeffMatrixForTimeVector((ts + te) / 2.0);
      ttte_ttts = calcIntegralxTxMatrix(te, true, hrp::dmatrix::Zero(recursive_order_ + 1, recursive_order_ + 1), recursive_order_, 0, te) - calcIntegralxTxMatrix(ts, true, hrp::dmatrix::Zero(recursive_order_ + 1, recursive_order_ + 1), recursive_order_, 0, ts) - ttte_ttts;
      ret = ret + (m_j * d_power_n * ttte_ttts) * (m_j * d_power_n).transpose();
      ts += t_step_;
      te += t_step_;
    }
    return ret;
  }
  // }}}

  // {{{ calcCoeffMatrixForTimeVector calc M(t) of p(t) = g^T b(t) = g^T M(t)t
  hrp::dmatrix calcCoeffMatrixForTimeVector(const double& t)
  {
    hrp::dmatrix ret = hrp::dmatrix::Zero(id_, recursive_order_ + 1);
    for (int i = 0; i < id_; i++)
    {
      ret.row(i) = bspline_elements_.at(i).coeffVector(t);
    }
    return ret;
  }
  // }}}

  // {{{ calcCoeffVector
  hrp::dvector calcCoeffVector(const double& t)
  {
    hrp::dvector ret = hrp::dvector::Zero(id_);
    for (int i = 0; i < id_; i++)
    {
      ret[i] = bspline_elements_.at(i).calc(t);
    }
    return ret;
  }
  // }}}

  // {{{ calc
  double calc(const double& t, const hrp::dvector& gain)
  {
    return gain.dot(this->calcCoeffVector(t));
  }
  // }}}

  // {{{ calcGainVectorCoeffMatrixFromViaTList
  hrp::dmatrix calcGainVectorCoeffMatrixFromViaTList(const std::vector<double>& via_t_list)
  {
    hrp::dmatrix ret = hrp::dmatrix::Zero(via_t_list.size(), id_max_);
    for (size_t i = 0; i < via_t_list.size(); i++)
    {
      ret.row(i) = this->calcCoeffVector(via_t_list.at(i));
    }
  }
  // }}}

  // {{{ calcGainVectorFromViaFList
  hrp::dvector calcGainVectorFromViaFList(const std::vector<double>& via_f_list, const std::vector<double>& via_t_list)
  {
    std::vector<double> via_f_list_buf = via_f_list;
    // using &vector[0] because vector.data() is available only after C++11
    return hrp::pseudoInverse(this->calcGainVectorCoeffMatrixFromViaTList(via_t_list)) * Eigen::Map<hrp::dvector>(&via_f_list_buf[0], via_f_list.size());
  }
  // }}}


  // {{{ discreteDeltaMatrix
  hrp::dmatrix discreteDeltaMatrix(const int& n)
  {
    if (n == 0)
    {
      return hrp::dmatrix::Identity(id_max_, id_max_);
    }
    else if (n == 1)
    {
      this->discreteDeltaMatrixImpl(AVERAGE);
    }
    else /* if (n > 1) */
    {
      this->discreteDeltaMatrixImpl(AVERAGE) * this->discreteDeltaMatrix(n - 1);
    }
  }
  // }}}

  // {{{ calcDeltaMatrix
  hrp::dmatrix calcDeltaMatrix(const int& n)
  {
    if (not is_delta_bspline_initialized_)
    {
      delta_bspline_.reset(new BSpline(recursive_order_, recursive_cnt_ - 1, id_, id_max_, t_min_, t_max_));
      is_delta_bspline_initialized_ = true;
    }
    if (n <= 0)
    {
      return hrp::dmatrix::Identity(id_max_, id_max_);
    }
    else if (n == 1)
    {
      return this->calcDeltaMatrixImpl();
    }
    else
    {
      return this->calcDeltaMatrixImpl() * delta_bspline_->calcDeltaMatrix(n - 1);
    }
  }
  // }}}

  // {{{ calcDeltaMatrixForKeepRecursiveOrder
  // bspline-set = A(t)[t^n-1;t^n-2;...;t^0], A(t)=MxN
  // bspline = pT[A(0),...,A(M-1)]
  // must be (N+1)(M-N) < M^2 => M=N+1 or M>N(N+1)
  hrp::dmatrix calcDeltaMatrixForKeepRecursiveOrder(const int& n)
  {
    std::vector<hrp::dmatrix> coeff_matrices;
    coeff_matrices.clear();
    for (size_t i = 0; i < bspline_elements_.size(); i++)
    {
      coeff_matrices.push_back(bspline_elements_.at(i).coeffMatrix());
    }
    hrp::dmatrix d_n = hrp::dmatrix::Zero(recursive_order_ + 1, recursive_order_ + 1);
    int row = recursive_order_;
    int col = recursive_order_ - n;
    int order = recursive_order_ - n;
    for (int id = 0; id < recursive_order_ + 1 - n; id++)
    {
      d_n(row, col) = 1;
      order = id + n;
      for (int i = 0; i < n; i++)
      {
        d_n(row, col) *= order;
        order--;
      }
      row--;
      col--;
    }
    hrp::dmatrix a = hrp::dmatrix::Zero(id_max_, (recursive_order_ + 1) * id_max_);
    for (size_t i = 0; i < coeff_matrices.size(); i++)
    {
      hrp::dmatrix tmp = coeff_matrices.at(i);
      tmp.transposeInPlace();  // distructive transpose
#warning size does not match, so fill the rest with 0s
      int size = tmp.cols() * tmp.rows();
      a.row(i).block(0, 0, 1, size) = Eigen::Map<hrp::dvector>(tmp.data(), size).transpose();
    }
    hrp::dmatrix ad = hrp::dmatrix::Zero(id_max_, (recursive_order_ + 1) * id_max_);
    for (size_t i = 0; i < coeff_matrices.size(); i++)
    {
      hrp::dmatrix tmp = coeff_matrices.at(i) * d_n;
      tmp.transposeInPlace();  // distructive transpose
#warning size does not match, so fill the rest with 0s
      int size = tmp.cols() * tmp.rows();
      ad.row(i).block(0, 0, 1, size) = Eigen::Map<hrp::dvector>(tmp.data(), size).transpose();
    }
    hrp::dmatrix ret = solveMatrixLinearEquation(
        hrp::dmatrix::Identity(id_max_, id_max_), a, id_max_, id_max_, ad);
    return ret;
  }
  // }}}

  // {{{ calcDiscreteDeltaCoeffVector
  hrp::dvector calcDiscreteDeltaCoeffVector(const double& t, const int& n)
  {
    return this->discreteDeltaMatrix(n) * this->calcCoeffVector(t);
  }
  // }}}

  // {{{ calcDeltaCoeffVector
  hrp::dvector calcDeltaCoeffVector(const double& t, const int& n)
  {
    if (not is_delta_bspline_initialized_)
    {
      delta_bspline_.reset(new BSpline(recursive_order_, recursive_cnt_ - 1, id_, id_max_, t_min_, t_max_));
      is_delta_bspline_initialized_ = true;
    }
    if (n == 1)
    {
      // is 1 really true?
      return this->calcDeltaMatrix(1) * delta_bspline_->calcCoeffVector(t);
    }
    else
    {
      // is 1 really true?
      return this->calcDeltaMatrix(1) * delta_bspline_->calcDeltaCoeffVector(t, n - 1);
    }
  }
  // }}}

  // {{{ calcDelta
  double calcDelta(const double& t, const hrp::dvector& gain, const int& n = 1, const bool& is_discrete = false)
  {
    if (n <= 0)
    {
      return this->calc(t, gain);
    }
    else
    {
      if (n <= 0)
      {
#warning gain.dot(this->calc(t, gain)) in bspline.l?
        // return gain.dot(this->calc(t, gain)); // gain.dot(this->calc(t, gain)) in bspline.l?
        std::cerr << "Error" << std::endl;
        return 0.0;  // this must be wrong
      }
      else if (is_discrete)
      {
        return gain.dot(this->calcDiscreteDeltaCoeffVector(t, n));
      }
      else
      {
        return gain.dot(this->calcDeltaCoeffVector(t, n));
      }
    }
  }
  // }}}

  // {{{ varifySumOfBSplineVector
  // for debug
  void varifySumOfBSplineVector(const hrp::dvector& gain)
  {
    std::vector<double> bufs;
    bufs.clear();
    double t_buf = t_min_;
    while (t_buf <= t_max_)
    {
      bufs.push_back(this->calc(t_buf, gain));
      t_buf += t_step_;
    }
    double average = 0.0;
    for (size_t i = 0; i < bufs.size(); i++) {
        average += bufs.at(i);
    }
    average /= bufs.size();
    std::sort(bufs.begin(), bufs.end(), compFunc);
    if (bufs.size() <= 0)
    {
      std::cerr << "bufs.size() <= 0" << std::endl;
      return;
    }
    double max_diff = bufs.at(0);
    double var = 0.0;
    for (size_t i = 0; i < bufs.size(); i++) {
        var += (bufs.at(i) - average) * (bufs.at(i)  - average);
    }
    var /= bufs.size();
    double stddev = std::sqrt(var);
    std::cerr << "average: " << average << " max_diff: " << max_diff << " stddev: " << stddev << std::endl;
  }
  // }}}

  // {{{ varifyInterpolePoint
  void varifyInterpolePoint(const std::vector<double>& f_list, const std::vector<double>& t_list, const hrp::dvector& gain, const double& tol = 0.1)
  {
    if (f_list.size() != t_list.size())
    {
      std::cerr << "f_list and t_list sizes do not match" << std::endl;
      return;
    }
    double t_buf = t_min_;
    std::vector<double> bufs;
    bufs.clear();
    while (t_buf <= t_max_)
    {
      bufs.push_back(this->calc(t_buf, gain));
      t_buf += t_step_;
    }
    for (size_t i = 0; i < f_list.size(); i++)
    {
      std::cerr << this->calc(t_list.at(i), gain) << " vs " << f_list.at(i) << std::endl;
    }
  }
  // }}}

  // {{{ varifyCalcDelta
  void varifyCalcDelta(const hrp::dvector& gain, const double& split_cnt = 1000.0)
  {
    double t_step = (t_max_ - t_min_) / split_cnt;
    double t_buf = t_min_;
    double f = this->calc(t_buf, gain);
    double prev_f;
    bool ok = true;
    while (t_buf + t_step <= t_max_)
    {
      t_buf += t_step;
      prev_f = f;
      f = this->calc(t_buf, gain);
      double delta = this->calcDelta(t_buf, gain);
      double diff = (f - prev_f) / t_step;
      std::cerr << "t: " << t_buf << " delta: " << delta << " diff: " << diff
                << " delta-diff: " << delta - diff << std::endl;
      if (std::abs(delta - diff) >= 100.0 / split_cnt)
      {
        ok = false;
      }
    }
    std::cerr << (ok ? "ok" : "ng") << std::endl;
  }
  // }}}

  // {{{ recursiveOrder getter
  const int& recursiveOrder() const
  {
    return recursive_order_;
  }
  // }}}

  // {{{ bsplineElements getter
  const std::vector<BSplineElement>& bsplineElements() const
  {
    return bspline_elements_;
  }
  // }}}

private:
  // {{{ discreteDeltaMatrixImpl
  hrp::dmatrix discreteDeltaMatrixImpl(const DerivativeMode& mode)
  {
    bool found = false;
    for (std::vector<DiscreteDeltaMatrix>::iterator itr = discrete_delta_matrix_.begin(); itr != discrete_delta_matrix_.end(); itr++) {
        if ((*itr).mode == mode) {
            return (*itr).matrix;
        }
    }

    DiscreteDeltaMatrix mode_and_dmat;
    mode_and_dmat.mode = mode;
    if (mode == FORWARD)
    {
      hrp::dmatrix ret = hrp::dmatrix::Identity(id_max_, id_max_);
      for (int i = 0; i < id_max_ - 1; i++)
      {
        ret(i, i + 1) = -1;
      }
      ret = ((1.0 / t_step_ - 1.0) * ret).transpose();
      mode_and_dmat.matrix = ret;
      discrete_delta_matrix_.push_back(mode_and_dmat);
      return ret;
    }
    else if (mode == BACKWARD)
    {
      hrp::dmatrix ret = hrp::dmatrix::Identity(id_max_, id_max_);
      for (int i = 0; i < id_max_ - 1; i++)
      {
        ret(i + 1, i) = -1;
      }
      ret = ((1.0 / t_step_ + 1.0) * ret).transpose();
      mode_and_dmat.matrix = ret;
      discrete_delta_matrix_.push_back(mode_and_dmat);
      return ret;
    }
    else if (mode == AVERAGE)
    {
      hrp::dmatrix forward = this->discreteDeltaMatrixImpl(FORWARD);
      hrp::dmatrix backward = this->discreteDeltaMatrixImpl(BACKWARD);
      hrp::dmatrix ret = 0.5 * (forward + backward);
      ret(0, 0) = -1.0 / t_step_;
      ret(0, 1) = 1.0 / t_step_;
      ret(ret.rows() - 1, ret.cols() - 2) = -1.0 / t_step_;
      ret(ret.rows() - 1, ret.cols() - 1) = 1.0 / t_step_;
      ret = ret.transpose();
      discrete_delta_matrix_.push_back(mode_and_dmat);
      return ret;
    }
  }
  // }}}

  // {{{ calcDeltaMatrixImpl
  hrp::dmatrix calcDeltaMatrixImpl()
  {
    if (delta_matrix_.cols() == 0 && delta_matrix_.rows() == 0)
    {
      delta_matrix_ = 1.0 / t_step_ * hrp::dmatrix::Identity(id_max_, id_max_);
      for (int i = 0; i < id_max_ - 1; i++)
      {
        delta_matrix_(i, i + 1) = -1.0 / t_step_;
      }
    }
    return delta_matrix_;
  }
  // }}}
  int recursive_order_;
  int recursive_cnt_;
  int id_;
  int id_max_;
  double t_min_;
  double t_max_;
  double t_step_;
  std::vector<BSplineElement> bspline_elements_;
  hrp::dmatrix delta_matrix_;
  std::vector<DiscreteDeltaMatrix> discrete_delta_matrix_;
  bool is_delta_bspline_initialized_;
  boost::shared_ptr<BSpline> delta_bspline_;
};

}  // namespace BSpline

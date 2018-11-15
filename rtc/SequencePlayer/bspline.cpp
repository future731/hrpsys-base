#include "bspline.h"

namespace BSpline
{
// {{{ calcTVector
hrp::dmatrix calcTVector(const double& t, const bool& is_toplevel, const hrp::dvector& buf, const int& n, const int& depth, const double& t_nth_power)
{
  hrp::dvector ret = buf;
  if (is_toplevel)
  {
    ret = hrp::dvector::Zero(n + 1);
  }
  if (depth > n)
  {
    return ret;
  }
  else
  {
    ret[ret.size() - 1 - depth] = t_nth_power;
    return calcTVector(t, false, ret, n, depth + 1, t_nth_power * t);
  }
}
// }}}

// {{{ calcBSplineCoeffVector
hrp::dvector calcBSplineCoeffVector(
    const double& t,
    const int& recursive_order,
    const int& recursive_cnt,
    const int& id,
    const int& id_max,
    const double& t_min,
    const double& t_max)
{
  double h = (t_max - t_min) / (id_max - recursive_order);
  double tj = h * id + (id_max * t_min - recursive_order * t_max) / (id_max - recursive_order);
  double tj_1 = tj + h;
  double tj_n_1 = tj_1 + h * recursive_cnt;
  if (t < tj or tj_n_1 <= t)
  {
    hrp::dvector ret = hrp::dvector::Zero(recursive_cnt + 1);
    return ret;
  }
  else if (recursive_cnt == 0)
  {
    hrp::dvector ret = hrp::dvector::Ones(1);
    return ret;
  }
  else
  {
    hrp::dvector v1 = calcBSplineCoeffVector(t, recursive_order, recursive_cnt - 1, id, id_max, t_min, t_max);
    hrp::dvector v2 = calcBSplineCoeffVector(t, recursive_order, recursive_cnt - 1, id + 1, id_max, t_min, t_max);
    hrp::dvector ret(v1.size() + 1);
    for (int i = 0; i < ret.size(); i++)
    {
      if (i == 0)
      {
        ret[i] = v1[i] - v2[i] + 0.0;
      }
      else if (i == ret.size() - 1)
      {
        ret[i] = 0.0 + tj_n_1 * v2[i - 1] - tj * v1[i - 1];
      }
      else
      {
        ret[i] = v1[i] - v2[i] + tj_n_1 * v2[i - 1] - tj * v1[i - 1];
      }
    }
    ret /= (recursive_cnt * h);
    return ret;
  }
}
// }}}

// {{{ calcMatrixLinearEquationCoeffMatrix
// A[p0,...,pM-1]B = {a_ik*p_kl*b_lj}ixj => C[P0;...;pM-1]
hrp::dmatrix calcMatrixLinearEquationCoeffMatrix(
    const hrp::dmatrix& left_matrix, const hrp::dmatrix& right_matrix,
    const int& col, const int& row, const std::vector<int>& col_list)
{
  hrp::dmatrix c = hrp::dmatrix::Zero(right_matrix.cols() * left_matrix.rows(), col * row);
  int x0 = 0;
  int y0 = 0;
  int cnt = 0;
  for (int i = 0; i < right_matrix.cols(); i++)
  {
    if (std::find(col_list.begin(), col_list.end(), i) != col_list.end())
    {
      for (int x = 0; x < left_matrix.rows(); x++)
      {
        for (int j = 0; j < col; j++)
        {
          for (int y = 0; y < left_matrix.cols(); y++)
          {
            cnt++;
            c(x0 + x, y0 + y + left_matrix.cols() * j) = left_matrix(x, y) * right_matrix(j, i);
          }
        }
      }
      x0 += left_matrix.rows();
    }
  }
  return c;
}
// }}}

// {{{ solveMatrixLinearEquation
// A[p0,...,pM-1]B = C[P0;...;pM-1] = D => return P
hrp::dmatrix solveMatrixLinearEquation(
    const hrp::dmatrix& left_matrix, const hrp::dmatrix& right_matrix,
    const int& col, const int& row, const hrp::dmatrix& answer_matrix)
{
  // Eigen stores values with column basis, so answer_matrix is transposed.
  hrp::dmatrix answer_matrix_copy = answer_matrix;
  hrp::dvector d_vector = Eigen::Map<hrp::dvector>(answer_matrix_copy.data(), answer_matrix_copy.cols() * answer_matrix_copy.rows());
  std::vector<int> col_list;
  col_list.clear();
  for (int i = 0; i < right_matrix.cols(); i++)
  {
    col_list.push_back(i);
  }
  hrp::dmatrix c = calcMatrixLinearEquationCoeffMatrix(left_matrix, right_matrix, col, row, col_list);
  hrp::dvector res = hrp::pseudoInverse(c) * d_vector;
  hrp::dmatrix ret = hrp::dmatrix::Zero(row, col);
  for (int i = 0; i < ret.rows(); i++)
  {
    for (int j = 0; j < ret.cols(); j++)
    {
      ret(i, j) = res[i + j * ret.rows()];
    }
  }
  return ret;
}
// }}}

// {{{ calcIntegralxTxMatrix
hrp::dmatrix calcIntegralxTxMatrix(const double& t, const bool& is_toplevel, const hrp::dmatrix& buf, const int& n, const int& depth, const double& t_power_n)
{
  hrp::dmatrix ret = buf;
  if (is_toplevel)
  {
    ret = hrp::dmatrix::Zero(n + 1, n + 1);
  }
  if (depth > 2 * n)
  {
    return ret;
  }
  else
  {
    double val = t_power_n / (depth + 1);
    if (depth > n)
    {
      int tmp = n - (depth - n);  // ???
      for (int i = 0; i < tmp + 1; i++)
      {
        ret(i, tmp - i) = val;
      }
    }
    else
    {
      int tmp = depth;
      for (int i = 0; i < tmp + 1; i++)
      {
        ret(n - i, n - tmp + i) = val;
      }
    }
    return calcIntegralxTxMatrix(t, false, ret, n, depth + 1, t_power_n * t);
  }
}
// }}}

// {{{ calcDeltaCoeffMatrixForTimeVector
hrp::dmatrix calcDeltaCoeffMatrixForTimeVector(const int& n)
{
  hrp::dmatrix ret = hrp::dmatrix::Zero(n + 1, n + 1);
  int cnt = n;
  for (int i = 0; i < n; i++)
  {
    ret(i, i + 1) = cnt;
    cnt--;
  }
  ret(0, 0) = 0.0;  // delta
  return ret;
}
// }}}

// {{{ compFunc
bool compFunc(double a, double b)
{
    return std::abs(a - 1.0) > std::abs(b - 1.0);
}
// }}}

}  // namespace BSpline

#ifndef ONLINE_TRAJECTORY_MODIFICATOR
#define ONLINE_TRAJECTORY_MODIFICATOR

#include <hrpModel/Link.h>
#include <hrpModel/JointPath.h>
#include "../ImpedanceController/JointPathEx.h"
#include "bspline.h"

#include "qpWrapper.h"

class OnlineTrajectoryModificatorMT
{
public:
  OnlineTrajectoryModificatorMT(
      const hrp::BodyPtr& robot,
      const std::vector<int>& rarm_indices,
      const double& dt,
      const boost::shared_ptr<std::ofstream>& ofs_bsp_debug,
      const std::string& instance_name,
      const hrp::Vector3& p_ground_to_racket,
      const hrp::Matrix33& R_ground_to_racket,
      const hrp::Vector3& p_rarm_to_racket,
      const hrp::Matrix33& R_rarm_to_racket,
      const hrp::dvector& p,
      const int& online_modified_min_id,
      const int& online_modified_max_id_1,
      const hrp::dvector& hit_pose,
      const std::vector<BSpline::BSpline>& bsplines,
      const double& tCurrent,
      const double& tHit,
      const int& id_max)
    : m_rarm_indices(rarm_indices),
      m_dt(dt),
      m_ofs_bsp_debug(ofs_bsp_debug),
      m_instance_name(instance_name),
      m_p_ground_to_racket(p_ground_to_racket),
      m_R_ground_to_racket(R_ground_to_racket),
      m_p_rarm_to_racket(p_rarm_to_racket),
      m_R_rarm_to_racket(R_rarm_to_racket),
      m_p(p),
      m_online_modified_min_id(online_modified_min_id),
      m_online_modified_max_id_1(online_modified_max_id_1),
      m_hit_pose(hit_pose),
      m_bsplines(bsplines),
      m_tCurrent(tCurrent),
      m_tHit(tHit),
      m_id_max(id_max)
  {
    m_robot_copy = hrp::BodyPtr(new hrp::Body(*robot));
    for (int i = 0; i < m_robot_copy->numJoints(); i++) {
      if (m_robot_copy->joint(i)->q != robot->joint(i)->q) {
        *m_ofs_bsp_debug << "joint q differs" << std::endl;
      }
      if (m_robot_copy->joint(i)->llimit != robot->joint(i)->llimit) {
        *m_ofs_bsp_debug << "joint llimit differs" << std::endl;
      }
      if (m_robot_copy->joint(i)->ulimit != robot->joint(i)->ulimit) {
        *m_ofs_bsp_debug << "joint ulimit differs" << std::endl;
      }
      if (m_robot_copy->joint(i)->lvlimit != robot->joint(i)->lvlimit) {
        *m_ofs_bsp_debug << "joint lvlimit differs" << std::endl;
      }
      if (m_robot_copy->joint(i)->uvlimit != robot->joint(i)->uvlimit) {
        *m_ofs_bsp_debug << "joint uvlimit differs" << std::endl;
      }
    }
    if (m_robot_copy->rootLink()->p != robot->rootLink()->p) {
      *m_ofs_bsp_debug << "rootlink p differs" << std::endl;
    }
    if (m_robot_copy->rootLink()->R != robot->rootLink()->R) {
      *m_ofs_bsp_debug << "rootlink R differs" << std::endl;
    }
  }

  hrp::dvector calc()
  {
    struct timeval time_s, time_e;
    gettimeofday(&time_s, NULL);
    m_robot_copy->calcForwardKinematics();
    std::vector<Link*> online_modified_jlist
      = std::vector<Link*>(m_robot_copy->joints().begin() + m_rarm_indices.at(0),
          m_robot_copy->joints().begin() + m_rarm_indices.at(m_rarm_indices.size() - 1) + 1);
    int k = online_modified_jlist.size(); // joint size
    int c = m_online_modified_max_id_1 - m_online_modified_min_id; // bspline control point time index
    std::string base_parent_name = m_robot_copy->joint(m_rarm_indices.at(0))->parent->name;
    std::string target_name = m_robot_copy->joint(m_rarm_indices.at(m_rarm_indices.size() - 1))->name;
    hrp::JointPathExPtr manip = hrp::JointPathExPtr(new hrp::JointPathEx(m_robot_copy, m_robot_copy->link(base_parent_name), m_robot_copy->link(target_name), m_dt, true, m_instance_name));
    double ik_error_pos = 1.0e-4;
    double ik_error_rot = 1.57 / 8.0;
    manip->setMaxIKError(ik_error_pos, ik_error_rot);
    short ik_iteration = 300;
    manip->setMaxIKIteration(ik_iteration);
    *m_ofs_bsp_debug << "calced tennis racket p before: " << (m_robot_copy->link(target_name)->R * m_p_rarm_to_racket + m_robot_copy->link(target_name)->p).eval().transpose() << std::endl;
    *m_ofs_bsp_debug << "calced tennis racket R before: " << std::endl;
    *m_ofs_bsp_debug << m_robot_copy->link(target_name)->R * m_R_rarm_to_racket << std::endl;

    struct timeval time_ik_s, time_ik_e;
    gettimeofday(&time_ik_s, NULL);
    bool ik_succeeded = manip->calcInverseKinematics2(m_p_ground_to_racket, m_R_ground_to_racket, m_p_rarm_to_racket, m_R_rarm_to_racket);
    gettimeofday(&time_ik_e, NULL);
    *m_ofs_bsp_debug << "calced tennis racket p after: " << (m_robot_copy->link(target_name)->R * m_p_rarm_to_racket + m_robot_copy->link(target_name)->p).eval().transpose() << std::endl;
    *m_ofs_bsp_debug << "calced tennis racket R after: " << std::endl;
    *m_ofs_bsp_debug << m_robot_copy->link(target_name)->R * m_R_rarm_to_racket << std::endl;
    *m_ofs_bsp_debug << "elapsed time ik: " << time_ik_e.tv_sec - time_ik_s.tv_sec + (time_ik_e.tv_usec - time_ik_s.tv_usec)*1.0e-6 << "[s]" << std::endl;
    if (!ik_succeeded) {
      *m_ofs_bsp_debug << "[OnlineTrajectoryModificator] ik failed" << std::endl;
      return hrp::dvector::Zero(m_p.size());
    } else {
      *m_ofs_bsp_debug << "[OnlineTrajectoryModificator] ik succeeded" << std::endl;
    }
    hrp::dvector dq = hrp::dvector::Zero(k);
    for (int i = 0; i < k; i++) {
      dq[i] = m_robot_copy->joint(m_rarm_indices.at(0) + i)->q - m_hit_pose[i];
    }
    hrp::dvector angle_min_limit(k);
    hrp::dvector angle_max_limit(k);
    for (int i = 0; i < k; i++) {
        angle_min_limit[i] = online_modified_jlist.at(i)->llimit;
        angle_max_limit[i] = online_modified_jlist.at(i)->ulimit;
    }
    hrp::dvector velocity_min_limit(k);
    hrp::dvector velocity_max_limit(k);
    for (int i = 0; i < k; i++) {
        velocity_min_limit[i] = online_modified_jlist.at(i)->lvlimit;
        velocity_max_limit[i] = online_modified_jlist.at(i)->uvlimit;
    }
    *m_ofs_bsp_debug << "dq is: " << dq.transpose() << std::endl;
    // hrp::dvector initial_state = hrp::dvector::Zero(c);
    hrp::dmatrix equality_matrix = hrp::dmatrix::Zero(3, c);
    equality_matrix.row(0) = m_bsplines.at(0).calcCoeffVector(m_tCurrent).segment(m_online_modified_min_id, c); // 右腕の現在関節角
    equality_matrix.row(1) = m_bsplines.at(0).calcDeltaCoeffVector(m_tCurrent, 1).segment(m_online_modified_min_id, c); // 右腕の現在関節速度
    equality_matrix.row(2) = m_bsplines.at(0).calcCoeffVector(m_tHit).segment(m_online_modified_min_id, c); // 右腕の打球時関節角
    // this may have some bug
    hrp::dmatrix delta_matrix = m_bsplines.at(0).calcDeltaMatrix(1).block(0, 1, c, c - 1);
    hrp::dmatrix inequality_matrix = delta_matrix.transpose();
    hrp::dmatrix eval_weight_matrix = hrp::dmatrix::Zero(c, c);
    for (int i = 0; i < c; i++) {
      eval_weight_matrix(i, i) = 1 + i * 0.1;
    }
    hrp::dvector eval_coeff_vector = hrp::dvector::Zero(c);
    hrp::dvector dp_modified = hrp::dvector::Zero(k * c);
    for (int j_k_id = 0; j_k_id < k; j_k_id++) { // foreach joints
      int id = m_id_max * (m_rarm_indices.at(0) + j_k_id); // m_p joint id start index
      hrp::dvector offset_ps = m_p.segment(id + m_online_modified_min_id, c);
      hrp::dvector state_min_vector = hrp::dvector::Zero(c);
      hrp::dvector state_max_vector = hrp::dvector::Zero(c);
      for (int i = 0; i <c; i++) {
        state_min_vector[i] = online_modified_jlist.at(j_k_id)->llimit;
      }
      state_min_vector -= offset_ps;
      for (int i = 0; i <c; i++) {
        state_max_vector[i] = online_modified_jlist.at(j_k_id)->ulimit;
      }
      state_max_vector -= offset_ps;
      hrp::dvector equality_coeff_vector = hrp::dvector::Zero(3);
      equality_coeff_vector[2] = dq[j_k_id];

      // angular velocity of joint j_k_id of each bspline control point
      hrp::dvector offset_vels = delta_matrix.transpose() * m_p.segment(id + m_online_modified_min_id, c);
      hrp::dvector inequality_min_vector = hrp::dvector::Zero(c - 1);
      for (int i = 0; i < c - 1; i++) {
          inequality_min_vector[i] = online_modified_jlist.at(j_k_id)->lvlimit;
      }
      inequality_min_vector -= offset_vels;
      hrp::dvector inequality_max_vector = hrp::dvector::Zero(c - 1);
      for (int i = 0; i < c - 1; i++) {
          inequality_max_vector[i] = online_modified_jlist.at(j_k_id)->uvlimit;
      }
      inequality_max_vector -= offset_vels;
      hrp::dvector tmp_dp_modified = hrp::dvector::Zero(c);

      // equal condition is equal
      double opt = solve_strict_qp(m_ofs_bsp_debug, state_min_vector, state_max_vector,
              eval_weight_matrix, eval_coeff_vector,
              equality_matrix, equality_coeff_vector,
              inequality_matrix, inequality_min_vector, inequality_max_vector,
              tmp_dp_modified);
      if (std::isfinite(opt)) {
          dp_modified.segment(j_k_id * c, c) = tmp_dp_modified;
      } else {
        // {{{ debug print
        *m_ofs_bsp_debug << "qp result is inf or nan" << std::endl;
        *m_ofs_bsp_debug << "result is " << tmp_dp_modified.transpose() << std::endl;
        *m_ofs_bsp_debug << "now angles as time: " << offset_ps.transpose() << std::endl;
        *m_ofs_bsp_debug
            << " min angle: " << angle_min_limit[j_k_id]
            << " hit angle: " << m_hit_pose[j_k_id]
            << " hit angle+dq: " << m_hit_pose[j_k_id] + dq[j_k_id]
            << " max angle: " << angle_max_limit[j_k_id]
            << std::endl;
        *m_ofs_bsp_debug << "now velocities as time: " << offset_vels.transpose() << std::endl;
        *m_ofs_bsp_debug
            << " min velocity: " << velocity_min_limit[j_k_id]
            << " hit velocity: " << offset_vels[j_k_id]
            << " max velocity: " << velocity_max_limit[j_k_id]
            << std::endl;
        *m_ofs_bsp_debug << "state_min_vector" << std::endl;
        for (int i = 0; i < state_min_vector.size(); i++) {
            *m_ofs_bsp_debug << state_min_vector[i];
            if (i != state_min_vector.size() - 1) {
                *m_ofs_bsp_debug << ", ";
            }
        }
        *m_ofs_bsp_debug << ";" << std::endl;

        *m_ofs_bsp_debug << "state_max_vector" << std::endl;
        for (int i = 0; i < state_max_vector.size(); i++) {
            *m_ofs_bsp_debug << state_max_vector[i];
            if (i != state_max_vector.size() - 1) {
                *m_ofs_bsp_debug << ", ";
            }
        }
        *m_ofs_bsp_debug << ";" << std::endl;

        *m_ofs_bsp_debug << "eval_weight_matrix" << std::endl;
        for (int i = 0; i < eval_weight_matrix.rows(); i++) {
            for (int j = 0; j < eval_weight_matrix.cols(); j++) {
                *m_ofs_bsp_debug << eval_weight_matrix(i, j);
                if (i != eval_weight_matrix.rows() - 1 or j != eval_weight_matrix.cols() - 1) {
                     *m_ofs_bsp_debug << ",";
                }
            }
            if (i != eval_weight_matrix.rows() - 1) {
                *m_ofs_bsp_debug << std::endl;
            }
        }
        *m_ofs_bsp_debug << ";" << std::endl;

        *m_ofs_bsp_debug << "eval_coeff_vector" << std::endl;
        for (int i = 0; i < eval_coeff_vector.size(); i++) {
            *m_ofs_bsp_debug << eval_coeff_vector[i];
            if (i != eval_coeff_vector.size() - 1) {
                *m_ofs_bsp_debug << ", ";
            }
        }
        *m_ofs_bsp_debug << ";" << std::endl;

        *m_ofs_bsp_debug << "equality_matrix" << std::endl;
        for (int i = 0; i < equality_matrix.rows(); i++) {
            for (int j = 0; j < equality_matrix.cols(); j++) {
                *m_ofs_bsp_debug << equality_matrix(i, j);
                if (i != equality_matrix.rows() - 1 or j != equality_matrix.cols() - 1) {
                     *m_ofs_bsp_debug << ",";
                }
            }
            if (i != equality_matrix.rows() - 1) {
                *m_ofs_bsp_debug << std::endl;
            }
        }
        *m_ofs_bsp_debug << ";" << std::endl;

        *m_ofs_bsp_debug << "equality_coeff_vector" << std::endl;
        for (int i = 0; i < equality_coeff_vector.size(); i++) {
            *m_ofs_bsp_debug << equality_coeff_vector[i];
            if (i != equality_coeff_vector.size() - 1) {
                *m_ofs_bsp_debug << ", ";
            }
        }
        *m_ofs_bsp_debug << ";" << std::endl;

        *m_ofs_bsp_debug << "inequality_matrix" << std::endl;
        for (int i = 0; i < inequality_matrix.rows(); i++) {
            for (int j = 0; j < inequality_matrix.cols(); j++) {
                *m_ofs_bsp_debug << inequality_matrix(i, j);
                if (i != inequality_matrix.rows() - 1 or j != inequality_matrix.cols() - 1) {
                     *m_ofs_bsp_debug << ",";
                }
            }
            if (i != inequality_matrix.rows() - 1) {
                *m_ofs_bsp_debug << std::endl;
            }
        }
        *m_ofs_bsp_debug << ";" << std::endl;

        *m_ofs_bsp_debug << "inequality_min_vector" << std::endl;
        for (int i = 0; i < inequality_min_vector.size(); i++) {
            *m_ofs_bsp_debug << inequality_min_vector[i];
            if (i != inequality_min_vector.size() - 1) {
                *m_ofs_bsp_debug << ", ";
            }
        }
        *m_ofs_bsp_debug << ";" << std::endl;

        *m_ofs_bsp_debug << "inequality_max_vector" << std::endl;
        for (int i = 0; i < inequality_max_vector.size(); i++) {
            *m_ofs_bsp_debug << inequality_max_vector[i];
            if (i != inequality_max_vector.size() - 1) {
                *m_ofs_bsp_debug << ", ";
            }
        }
        *m_ofs_bsp_debug << ";" << std::endl;

        // }}}
        return hrp::dvector::Zero(m_p.size());
      }
    }
    *m_ofs_bsp_debug << "qp succeeded" << std::endl;
    hrp::dvector dp = hrp::dvector::Zero(m_p.size());
    for (size_t i = 0; i < k; i++) {
      dp.segment((i + m_rarm_indices.at(0)) * m_id_max, c) = dp_modified.segment(i * c, c);
    }
    gettimeofday(&time_e, NULL);
    *m_ofs_bsp_debug << "elapsed time: " << time_e.tv_sec - time_s.tv_sec + (time_e.tv_usec - time_s.tv_usec)*1.0e-6 << "[s]" << std::endl;
    return dp;
  }

private:
  hrp::BodyPtr m_robot_copy;
  pthread_t m_modification_thread;
  pthread_mutex_t m_mutex;
  std::vector<int> m_rarm_indices;
  double m_dt;
  boost::shared_ptr<std::ofstream> m_ofs_bsp_debug;
  std::string m_instance_name;
  hrp::Vector3 m_p_ground_to_racket;
  hrp::Matrix33 m_R_ground_to_racket;
  hrp::Vector3 m_p_rarm_to_racket;
  hrp::Matrix33 m_R_rarm_to_racket;
  hrp::dvector m_p;
  int m_online_modified_min_id;
  int m_online_modified_max_id_1;
  hrp::dvector m_hit_pose;
  std::vector<BSpline::BSpline> m_bsplines;
  double m_tCurrent;
  double m_tHit;
  int m_id_max;
};

#endif

#ifndef ONLINE_TRAJECTORY_MODIFICATOR
#define ONLINE_TRAJECTORY_MODIFICATOR

#include <hrpModel/Link.h>
#include <hrpModel/JointPath.h>
#include "../ImpedanceController/JointPathEx.h"
#include "bspline.h"
#include <sys/time.h>

#ifdef USE_QPOASES_IN_SEQUENCER
#include "qpOASESWrapper.h"
#else
#include "eiquadprog.h"
#endif

#include "SequencePlayerService_impl.h"

namespace hrp
{

class OnlineTrajectoryModificatorMT
{
public:
  OnlineTrajectoryModificatorMT(
      const hrp::BodyPtr& robot,
      const std::vector<int>& rarm_indices,
      const double& dt,
      const boost::shared_ptr<std::ofstream>& ofs_bsp_debug,
      const std::string& instance_name,
      const hrp::Vector3& p_rarm_to_racket,
      const hrp::Matrix33& R_rarm_to_racket,
      const hrp::Vector3& p_rarm_to_end_effector,
      const hrp::Matrix33& R_rarm_to_end_effector,
      const hrp::Vector3& p_end_effector_to_racket,
      const hrp::Matrix33& R_end_effector_to_racket,
      const OpenHRP::TimedPosAndVelWithCovariance& hitTarget,
      const hrp::dvector& p,
      const std::vector<BSpline::BSpline>& bsplines,
      const double& tCurrent,
      const double& tHit,
      const bool& isChoreonoid,
      const int& bsplines_length,
      const hrp::dvector& rootlink_6dof_offset,
      const int& id_max)
    : m_rarm_indices(rarm_indices),
      m_dt(dt),
      m_ofs_bsp_debug(ofs_bsp_debug),
      m_instance_name(instance_name),
      m_p_rarm_to_racket(p_rarm_to_racket),
      m_R_rarm_to_racket(R_rarm_to_racket),
      m_p_rarm_to_end_effector(p_rarm_to_end_effector),
      m_R_rarm_to_end_effector(R_rarm_to_end_effector),
      m_p_end_effector_to_racket(p_end_effector_to_racket),
      m_R_end_effector_to_racket(R_end_effector_to_racket),
      m_hitTarget(hitTarget),
      m_target(hrp::dvector::Zero(6)),
      m_last_target(hrp::dvector::Zero(6)),
      m_p(p),
      m_bsplines(bsplines),
      m_tCurrent(tCurrent),
      m_tHit(tHit),
      m_isChoreonoid(isChoreonoid),
      m_bsplines_length(bsplines_length),
      m_rootlink_6dof_offset(rootlink_6dof_offset),
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
    const double epsilon = 2.22507e-308;
    BSpline::BSpline bspline = m_bsplines.at(0);
    hrp::dvector coeff_vector_current = bspline.calcCoeffVector(m_tCurrent);
    hrp::dvector coeff_vector_hit = bspline.calcCoeffVector(m_tHit);
    int online_modified_min_id = 0;
    for (int i = 0; i < m_id_max; i++) {
        if (std::abs(coeff_vector_current[i]) > epsilon) {
            online_modified_min_id = i;
            break;
        }
    }
    int online_modified_max_id_1 = m_id_max;
    for (int i = 0; i < m_id_max; i++) {
        if (std::abs(coeff_vector_hit[m_id_max - i]) > epsilon) {
            online_modified_max_id_1 = m_id_max - i;
            break;
        }
    }

    // online_modified_max_id_1は実際のonline_modified_max_idより1大きい
    int c = online_modified_max_id_1 - online_modified_min_id;  // ここは+1する必要がない
    // 腕だけik
    // online_modified_links ikを解く*limb*のlink(jointを持っている)のリスト
    // RARM_LINK0, 1, 2, 3, 4, 5, 6, 7
#warning m_ofs_bsp_debug will be deleted
    std::vector<Link*> online_modified_jlist
        = std::vector<Link*>(m_robot_copy->joints().begin() + m_rarm_indices.at(0),
                             m_robot_copy->joints().begin() + m_rarm_indices.at(m_rarm_indices.size() - 1) + 1);
    int k = online_modified_jlist.size();
    // hit_pose_full: m_tHitでの関節角度+rootlink6自由度の計算, fix-leg-to-coordsなどをして現在姿勢を計算(IKの初期値の姿勢)
    hrp::dvector hit_pose_full = hrp::dvector::Zero(m_bsplines_length);
    // full body joint without THK
    for (size_t i = 0; i < m_robot_copy->numJoints() - (m_isChoreonoid ? 4 : 0); i++) {
        hrp::dvector ps(m_id_max);
        for (int j = 0; j < m_id_max; j++) {
            ps[j] = m_p[m_id_max * i + j];
        }
        // joint数33, bsplinesのサイズdが39(virtualjoint6dofがあるため)
        hit_pose_full[i] = m_bsplines.at(i).calc(m_tHit, ps);
    }
#warning THKハンド
    // rootlink 3dof rot of 6dof; euslisp virtual joint align is rpy
    for (int i = 0; i < 3; i++) {
        hrp::dvector ps(m_id_max);
        for (int j = 0; j < m_id_max; j++) {
            ps[j] = m_p[m_id_max * (i + m_robot_copy->numJoints() - (m_isChoreonoid ? 4 : 0) + 3) + j];
        }
        hit_pose_full[i + m_robot_copy->numJoints() - (m_isChoreonoid ? 4 : 0) + 3] = m_bsplines.at(i + m_robot_copy->numJoints() - (m_isChoreonoid ? 4 : 0) + 3).calc(m_tHit, ps);
    }
    // rootlink 3dof pos of 6dof
    for (int i = 0; i < 3; i++) {
        hrp::dvector ps(m_id_max);
        for (int j = 0; j < m_id_max; j++) {
            ps[j] = m_p[m_id_max * (i + m_robot_copy->numJoints() - (m_isChoreonoid ? 4 : 0)) + j];
        }
        hit_pose_full[i + m_robot_copy->numJoints() - (m_isChoreonoid ? 4 : 0)] = m_bsplines.at(i + m_robot_copy->numJoints() - (m_isChoreonoid ? 4 : 0)).calc(m_tHit, ps);
    }
    *m_ofs_bsp_debug << "expected hit virtualjoint rpy: " << hit_pose_full.segment(m_robot_copy->numJoints() - (m_isChoreonoid ? 4 : 0) + 3, 3).transpose() << std::endl;
    *m_ofs_bsp_debug << "expected hit virtualjoint p: " << hit_pose_full.segment(m_robot_copy->numJoints() - (m_isChoreonoid ? 4 : 0), 3).transpose() << std::endl;
    hit_pose_full.segment(m_robot_copy->numJoints() - (m_isChoreonoid ? 4 : 0) + 3, 3)
        = hrp::rpyFromRot(
                hrp::rotFromRpy(
                    hit_pose_full[m_robot_copy->numJoints() - (m_isChoreonoid ? 4 : 0) + 3],
                    hit_pose_full[m_robot_copy->numJoints() - (m_isChoreonoid ? 4 : 0) + 4],
                    hit_pose_full[m_robot_copy->numJoints() - (m_isChoreonoid ? 4 : 0) + 5])
                * hrp::rotFromRpy(
                    m_rootlink_6dof_offset[3],
                    m_rootlink_6dof_offset[4],
                    m_rootlink_6dof_offset[5])
                        );
    hit_pose_full.segment(m_robot_copy->numJoints() - (m_isChoreonoid ? 4 : 0), 3)
        += hrp::rotFromRpy(
                hit_pose_full[m_robot_copy->numJoints() - (m_isChoreonoid ? 4 : 0) + 3],
                hit_pose_full[m_robot_copy->numJoints() - (m_isChoreonoid ? 4 : 0) + 4],
                hit_pose_full[m_robot_copy->numJoints() - (m_isChoreonoid ? 4 : 0) + 5]) * m_rootlink_6dof_offset.segment(0, 3);

    // for debug
    *m_ofs_bsp_debug << "expected hit rootlink p: " << hit_pose_full.segment(m_robot_copy->numJoints() - (m_isChoreonoid ? 4 : 0), 3).transpose() << std::endl;
    *m_ofs_bsp_debug << "expected hit rootlink rpy: " << hit_pose_full.segment(m_robot_copy->numJoints() - (m_isChoreonoid ? 4 : 0) + 3, 3).transpose() << std::endl;

    // hit_pose: dq計算用 hit_pose_fullのうち補正する関節のみのangle-vector
    hrp::dvector hit_pose = hit_pose_full.segment(m_rarm_indices.at(0), k);

    for (int i=0; i<m_robot_copy->numJoints() - (m_isChoreonoid ? 4 : 0); i++){
        m_robot_copy->joint(i)->q = hit_pose_full[i];
    }

    for (int i = 0; i < 3; i++) {
        m_robot_copy->rootLink()->p[i] = hit_pose_full[m_robot_copy->numJoints() - (m_isChoreonoid ? 4 : 0) + i];
    }

    m_robot_copy->rootLink()->R = hrp::rotFromRpy(
            hit_pose_full[m_robot_copy->numJoints()-(m_isChoreonoid ? 4 : 0)+3],
            hit_pose_full[m_robot_copy->numJoints()-(m_isChoreonoid ? 4 : 0)+4],
            hit_pose_full[m_robot_copy->numJoints()-(m_isChoreonoid ? 4 : 0)+5]);

    *m_ofs_bsp_debug << "hit angle-vector: " << hit_pose_full.segment(0, m_robot_copy->numJoints() - (m_isChoreonoid ? 4 : 0)).transpose() << std::endl;
    *m_ofs_bsp_debug << "hit pose full (rootlink's x y z r p y): " << hit_pose_full.segment(m_robot_copy->numJoints() - (m_isChoreonoid ? 4 : 0), 6).transpose() << std::endl;
    *m_ofs_bsp_debug << "rootlink p: " << m_robot_copy->rootLink()->p.transpose() << std::endl;
    *m_ofs_bsp_debug << "rootlink R: " << m_robot_copy->rootLink()->R << std::endl;
    /*
    */
    // 後でdqを求めるため，ikを解く必要があり，その準備として今FKを解いておく
    m_robot_copy->calcForwardKinematics();
    // ラケットの位置を求める

    std::string base_parent_name = m_robot_copy->joint(m_rarm_indices.at(0))->parent->name;
    std::string target_name = m_robot_copy->joint(m_rarm_indices.at(m_rarm_indices.size()-1))->name;
    // rarm: rarmの最後のjoint, end_effector: rarmのend_effector, racket: ラケット打面中心
    hrp::Matrix33 R_ground_to_rarm_expected = m_robot_copy->link(target_name)->R;
    hrp::Vector3 p_ground_to_rarm_expected = m_robot_copy->link(target_name)->p;
    hrp::Matrix33 R_ground_to_end_effector_expected = R_ground_to_rarm_expected * m_R_rarm_to_end_effector;
    hrp::Vector3 p_ground_to_end_effector_expected = R_ground_to_rarm_expected * m_p_rarm_to_end_effector + p_ground_to_rarm_expected;
    hrp::Matrix33 R_ground_to_racket_expected = R_ground_to_end_effector_expected * m_R_end_effector_to_racket;
    hrp::Vector3 p_ground_to_racket_expected = R_ground_to_end_effector_expected * m_p_end_effector_to_racket + p_ground_to_end_effector_expected;

    double x = m_hitTarget.point.x;
    double y = m_hitTarget.point.y;
    double z = m_hitTarget.point.z;
    double vx = m_hitTarget.velocity.vx;
    double vy = m_hitTarget.velocity.vy;
    double vz = m_hitTarget.velocity.vz;
    if (not m_hit_plane_initialized) {
      // somewhat px and py does not follow with bspline param sometimes
      m_px = p_ground_to_racket_expected[0];
      m_py = p_ground_to_racket_expected[1];
      m_hit_plane_initialized = true;
    }
    OpenHRP::dSequence var = m_hitTarget.pos_and_vel_covariance;
    double k_hit = (vy * x - vx * y) / (vy * m_px - vx * m_py);
    double hit_pos_x = k_hit * m_px;
    double hit_pos_y = k_hit * m_py;
    // call service delay
    double ttc_margin = 0.4;
    double ttc = (hit_pos_y - y) / vy - ttc_margin;
    *m_ofs_bsp_debug << "x: " << x << " vx: " << vx <<  " y: " << y << " vy: " << vy << " z: " << z << " vz: " << vz << std::endl;
    // *m_ofs_bsp_debug << "expected ee pos: " << p_ground_to_end_effector_expected.transpose() << std::endl;
    *m_ofs_bsp_debug << "expected ground_to_racket pos: " << p_ground_to_racket_expected.transpose() << std::endl;
    // *m_ofs_bsp_debug << "expected ground_to_racket rotation: " << std::endl;
    // *m_ofs_bsp_debug << R_ground_to_racket_expected << std::endl;
    *m_ofs_bsp_debug << "expected ground_to_racket rpy: " << rpyFromRot(R_ground_to_racket_expected).transpose() << std::endl;
    *m_ofs_bsp_debug << "k hit is: " << k_hit << std::endl;
    *m_ofs_bsp_debug << "ttc is: " << ttc << std::endl;
    double var_trace = 0.0;
    for (int i = 0; i < 6; i++) {
        var_trace += var[i * 6 + i];
    }

    m_target[0] = x + vx * ttc;
    m_target[1] = y + vy * ttc;
    m_target[2] = z + vz * ttc - 9.8 / 2 * ttc * ttc;
    //m_target[0] = 1.0644;
    //m_target[1] = 0.385105;
    //m_target[2] = 0.603515 + 0.0;
    m_target[3] = -0.95983;
    m_target[4] = -1.14178;
    m_target[5] = -0.502124;
    *m_ofs_bsp_debug << "target: " << m_target[0] << " " << m_target[1] << " " << m_target[2] << " " << m_target[3] << " " << m_target[4] << " " << m_target[5] << std::endl;

    if (var_trace > 2.0 or ttc > m_tHit or ttc < 0.0) {
        *m_ofs_bsp_debug << "target is not valid" << std::endl;
        gettimeofday(&time_e, NULL);
        *m_ofs_bsp_debug << "onlineTrajectoryModification elapsed time: " << time_e.tv_sec - time_s.tv_sec + (time_e.tv_usec - time_s.tv_usec)*1.0e-6 << "[s]" << std::endl;
        return hrp::dvector::Zero(m_p.size());
    }
    if (var_trace < 0.6) {
        *m_ofs_bsp_debug << "target is converged." << std::endl;
        gettimeofday(&time_e, NULL);
        *m_ofs_bsp_debug << "onlineTrajectoryModification elapsed time: " << time_e.tv_sec - time_s.tv_sec + (time_e.tv_usec - time_s.tv_usec)*1.0e-6 << "[s]" << std::endl;
        return hrp::dvector::Zero(m_p.size());
    }

    if (m_is_first_valid_target) {
        m_is_first_valid_target = false;
        m_last_target = m_target;
    }
    /*
#warning P control
    double pos_p_gain = 0.2;
    m_target = (m_last_target - m_target) * pos_p_gain + m_target;
    *m_ofs_bsp_debug << "target P: " << m_target[0] << " " << m_target[1] << " " << m_target[2] << " " << m_target[3] << " " << m_target[4] << " " << m_target[5] << std::endl;
    */

    // dq
    // ラケット先端がm_target(6次元)にある想定
    // ラケットの姿勢を元にendcoordsを手先に設定
    // 右手先端から見たラケットの打点
    // irteusgl$ (send (send *robot* :rarm :end-coords) :transformation (send *sweet-spot* :copy-worldcoords) :local)
    // #<coordinates #X10141278  332.34単位注意 0.0 332.34単位注意 / 2.186 -0.524 2.186>
    //
    // ラケットの打点から見た右手先端
    // irteusgl$ (send (send *sweet-spot* :copy-worldcoords) :transformation (send *robot* :rarm :end-coords) :local)
    // #<coordinates #X1071a898  1.137e-13 -470.0単位注意 -3.411e-13 / 2.186 -0.524 2.186>
    // その上でIKを解く
    // 関節角の差分を返す

    // m_targetはラケット先端
    // gripperは無視
    // end_effectorはハンド
    // rarmはarmの先端リンク
    // m_targetを元にrarmの位置姿勢を計算
    // onInitializeに移行できるものはするとよい
    // target(*sweet-spot*の位置)から計算されたground座標系におけるrarmの位置姿勢
    hrp::Vector3 p_ground_to_racket = m_target.segment(0, 3);
    hrp::Vector3 rpy_ground_to_racket = m_target.segment(3, 3);
    hrp::Matrix33 R_ground_to_racket = rotFromRpy(rpy_ground_to_racket[0], rpy_ground_to_racket[1], rpy_ground_to_racket[2]);
    /*
    hrp::Vector3 p_ground_to_end_effector = R_ground_to_racket * p_racket_to_end_effector + p_ground_to_racket;
    hrp::Matrix33 R_ground_to_end_effector = R_ground_to_racket * R_racket_to_end_effector;
    hrp::Vector3 p_ground_to_rarm = R_ground_to_racket * m_p_racket_to_rarm + p_ground_to_racket;
    hrp::Matrix33 R_ground_to_rarm = R_ground_to_racket * m_R_racket_to_rarm;
    */
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
    bool ik_succeeded = manip->calcInverseKinematics2(p_ground_to_racket, R_ground_to_racket, m_p_rarm_to_racket, m_R_rarm_to_racket);
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
      dq[i] = m_robot_copy->joint(m_rarm_indices.at(0) + i)->q - hit_pose[i];
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
    equality_matrix.row(0) = m_bsplines.at(0).calcCoeffVector(m_tCurrent).segment(online_modified_min_id, c); // 右腕の現在関節角
    equality_matrix.row(1) = m_bsplines.at(0).calcDeltaCoeffVector(m_tCurrent, 1).segment(online_modified_min_id, c); // 右腕の現在関節速度
    equality_matrix.row(2) = m_bsplines.at(0).calcCoeffVector(m_tHit).segment(online_modified_min_id, c); // 右腕の打球時関節角
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
      hrp::dvector offset_ps = m_p.segment(id + online_modified_min_id, c);
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
      hrp::dvector offset_vels = delta_matrix.transpose() * m_p.segment(id + online_modified_min_id, c);
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

#ifndef USE_QPOASES_IN_SEQUENCER
#warning ここconst参照的なものにできないか確かめる
      hrp::dmatrix G = eval_weight_matrix;
      hrp::dvector g0 = eval_coeff_vector;
      hrp::dmatrix CE = equality_matrix;
      hrp::dvector ce0 = -equality_coeff_vector; // sign inversion is needed; in eus interface, equality-coeff signature is inverted when passing it to c++ eiquagprog source
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
      double opt = Eigen::solve_quadprog(G, g0, CE.transpose(), ce0, CI.transpose(), ci0, tmp_dp_modified);
      // *m_ofs_bsp_debug << "opt: " << opt << std::endl;
#else
      double opt = solve_strict_qp(m_ofs_bsp_debug, state_min_vector, state_max_vector,
              eval_weight_matrix, eval_coeff_vector,
              equality_matrix, equality_coeff_vector,
              inequality_matrix, inequality_min_vector, inequality_max_vector,
              tmp_dp_modified);
      // *m_ofs_bsp_debug << "equality_coeff_vector" << std::endl;
      // *m_ofs_bsp_debug << equality_coeff_vector.transpose() << std::endl;
      // *m_ofs_bsp_debug << "equality_coeff_vector calced" << std::endl;
      // *m_ofs_bsp_debug << (equality_matrix * tmp_dp_modified).eval().transpose() << std::endl;

      // hrp::dmatrix equality_weight_matrix = hrp::dmatrix::Zero(c, c);
      // equality_weight_matrix(0, 0) = 1.0; // current position equality
      // equality_weight_matrix(1, 1) = 0.5; // current speed equality
      // equality_weight_matrix(2, 2) = 0.4; // target position equality
      // double opt = solve_mild_qp(m_ofs_bsp_debug, state_min_vector, state_max_vector,
      //         eval_weight_matrix, eval_coeff_vector,
      //         equality_matrix, equality_coeff_vector, equality_weight_matrix,
      //         inequality_matrix, inequality_min_vector, inequality_max_vector,
      //         tmp_dp_modified);
#endif
      if (std::isfinite(opt)) {
          dp_modified.segment(j_k_id * c, c) = tmp_dp_modified;
      } else {
        // {{{ debug print
        *m_ofs_bsp_debug << "qp result is inf or nan" << std::endl;
        *m_ofs_bsp_debug << "result is " << tmp_dp_modified.transpose() << std::endl;
        *m_ofs_bsp_debug << "now angles as time: " << offset_ps.transpose() << std::endl;
        *m_ofs_bsp_debug
            << " min angle: " << angle_min_limit[j_k_id]
            << " hit angle: " << hit_pose[j_k_id]
            << " hit angle+dq: " << hit_pose[j_k_id] + dq[j_k_id]
            << " max angle: " << angle_max_limit[j_k_id]
            << std::endl;
        *m_ofs_bsp_debug << "now velocities as time: " << offset_vels.transpose() << std::endl;
        *m_ofs_bsp_debug
            << " min velocity: " << velocity_min_limit[j_k_id]
            << " hit velocity: " << offset_vels[j_k_id]
            << " max velocity: " << velocity_max_limit[j_k_id]
            << std::endl;
#ifdef USE_QPOASES_IN_SEQUENCER
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

#else
        *m_ofs_bsp_debug << "G" << std::endl;
        for (int i = 0; i < G.rows(); i++) {
            for (int j = 0; j < G.cols(); j++) {
                *m_ofs_bsp_debug << G(i, j) << ",";
            }
            *m_ofs_bsp_debug << std::endl;
        }
        *m_ofs_bsp_debug << "g0" << std::endl;
        for (int i = 0; i < g0.size(); i++) {
            *m_ofs_bsp_debug << g0[i] << ",";
        }
        *m_ofs_bsp_debug << std::endl;
        *m_ofs_bsp_debug << "CE" << std::endl;
        for (int i = 0; i < CE.rows(); i++) {
            for (int j = 0; j < CE.cols(); j++) {
                *m_ofs_bsp_debug << CE(i, j) << ",";
            }
            *m_ofs_bsp_debug << std::endl;
        }
        *m_ofs_bsp_debug << "ce0" << std::endl;
        for (int i = 0; i < ce0.size(); i++) {
            *m_ofs_bsp_debug << ce0[i] << ",";
        }
        *m_ofs_bsp_debug << std::endl;
        *m_ofs_bsp_debug << "CI" << std::endl;
        for (int i = 0; i < CI.rows(); i++) {
            for (int j = 0; j < CI.cols(); j++) {
                *m_ofs_bsp_debug << CI(i, j) << ",";
            }
            *m_ofs_bsp_debug << std::endl;
        }
        *m_ofs_bsp_debug << std::endl;
        *m_ofs_bsp_debug << "ci0" << std::endl;
        for (int i = 0; i < ci0.size(); i++) {
            *m_ofs_bsp_debug << ci0[i] << ",";
        }
        *m_ofs_bsp_debug << std::endl;
#endif
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
  hrp::Vector3 m_p_rarm_to_racket;
  hrp::Matrix33 m_R_rarm_to_racket;
  hrp::Vector3 m_p_rarm_to_end_effector;
  hrp::Matrix33 m_R_rarm_to_end_effector;
  hrp::Vector3 m_p_end_effector_to_racket;
  hrp::Matrix33 m_R_end_effector_to_racket;
  OpenHRP::TimedPosAndVelWithCovariance m_hitTarget;
  hrp::dvector m_p;
  static bool m_is_first_valid_target; // thread safe when c++11
  hrp::dvector m_target;
  hrp::dvector m_last_target;
  static double m_px; // thread safe when c++11
  static double m_py; // thread safe when c++11
  static bool m_hit_plane_initialized; // thread safe when c++11
  std::vector<BSpline::BSpline> m_bsplines;
  double m_tCurrent;
  double m_tHit;
  bool m_isChoreonoid;
  int m_bsplines_length;
  hrp::dvector m_rootlink_6dof_offset;
  int m_id_max;
};

} // namespace hrp

#endif

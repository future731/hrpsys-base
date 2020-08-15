// -*- C++ -*-
// clang-format off
/*!
 * @file  SequencePlayer.cpp
 * @brief sequence player component
 * $Date$
 *
 * $Id$
 */

#include <rtm/CorbaNaming.h>
#include <hrpModel/Link.h>
#include <hrpModel/ModelLoaderUtil.h>
#include "SequencePlayer.h"
#include "hrpsys/util/VectorConvert.h"
#include <hrpModel/JointPath.h>
#include <hrpUtil/MatrixSolvers.h>
#include "../ImpedanceController/JointPathEx.h"
#include "hrpsys/idl/SequencePlayerService.hh"
#include <sys/time.h>
#include <boost/algorithm/string.hpp>

typedef coil::Guard<coil::Mutex> Guard;

// Module specification
// <rtc-template block="module_spec">
static const char* sequenceplayer_spec[] =
    {
        "implementation_id", "SequencePlayer",
        "type_name",         "SequencePlayer",
        "description",       "sequence player component",
        "version",           HRPSYS_PACKAGE_VERSION,
        "vendor",            "AIST",
        "category",          "example",
        "activity_type",     "DataFlowComponent",
        "max_instance",      "10",
        "language",          "C++",
        "lang_type",         "compile",
        // Configuration variables
        "conf.default.debugLevel", "0",
        "conf.default.fixedLink", "",

        ""
    };
// </rtc-template>

SequencePlayer::SequencePlayer(RTC::Manager* manager)
    : RTC::DataFlowComponentBase(manager),
      // <rtc-template block="initializer">
      m_qInitIn("qInit", m_qInit),
      m_basePosInitIn("basePosInit", m_basePosInit),
      m_baseRpyInitIn("baseRpyInit", m_baseRpyInit),
      m_zmpRefInitIn("zmpRefInit", m_zmpRefInit),
      m_hitTargetIn("hitTargetIn", m_hitTarget),
      m_qRefOut("qRef", m_qRef),
      m_tqRefOut("tqRef", m_tqRef),
      m_zmpRefOut("zmpRef", m_zmpRef),
      m_accRefOut("accRef", m_accRef),
      m_basePosOut("basePos", m_basePos),
      m_baseRpyOut("baseRpy", m_baseRpy),
      m_optionalDataOut("optionalData", m_optionalData),
      m_SequencePlayerServicePort("SequencePlayerService"),
      // </rtc-template>
      m_robot(hrp::BodyPtr()),
      m_debugLevel(0),
      m_error_pos(0.0001),
      m_error_rot(0.001),
      m_iteration(50),
      m_rootlink_6dof_offset(hrp::dvector::Zero(6)),
      m_onlineModifyStarted(false),
      m_isChoreonoid(false),
      m_id_max(0),
      m_tMin(0.0),
      m_tMax(std::numeric_limits<double>::max()),
      m_tCurrent(0.0),
      m_tHit(0.0),
      m_bsplines(std::vector<BSpline::BSpline>()),
      m_p(hrp::dvector::Zero(0)),
      m_last_target(hrp::dvector::Zero(6)),
      m_qp_ready(false),
      m_qp_last_ready(false)
{
    sem_init(&m_waitSem, 0, 0);
    m_service0.player(this);
    m_clearFlag = false;
    m_waitFlag = false;
}

SequencePlayer::~SequencePlayer()
{
}


RTC::ReturnCode_t SequencePlayer::onInitialize()
{
    std::cout << "SequencePlayer::onInitialize()" << std::endl;
    // Registration: InPort/OutPort/Service
    // <rtc-template block="registration">
    // Set InPort buffers
    addInPort("qInit", m_qInitIn);
    addInPort("basePosInit", m_basePosInitIn);
    addInPort("baseRpyInit", m_baseRpyInitIn);
    addInPort("zmpRefInit", m_zmpRefInitIn);
    addInPort("hitTargetIn", m_hitTargetIn);

    // Set OutPort buffer
    addOutPort("qRef", m_qRefOut);
    addOutPort("tqRef", m_tqRefOut);
    addOutPort("zmpRef", m_zmpRefOut);
    addOutPort("accRef", m_accRefOut);
    addOutPort("basePos", m_basePosOut);
    addOutPort("baseRpy", m_baseRpyOut);
    addOutPort("optionalData", m_optionalDataOut);

    // Set service provider to Ports
    m_SequencePlayerServicePort.registerProvider("service0", "SequencePlayerService", m_service0);

    // Set service consumers to Ports

    // Set CORBA Service Ports
    addPort(m_SequencePlayerServicePort);

    // </rtc-template>
    // <rtc-template block="bind_config">
    // Bind variables and configuration variable

    bindParameter("debugLevel", m_debugLevel, "0");
    bindParameter("fixedLink", m_fixedLink, "");
    // </rtc-template>

    RTC::Properties& prop = getProperties();
    coil::stringTo(dt, prop["dt"].c_str());

    m_robot = hrp::BodyPtr(new Body());

    RTC::Manager& rtcManager = RTC::Manager::instance();
    std::string nameServer = rtcManager.getConfig()["corba.nameservers"];
    int comPos = nameServer.find(",");
    if (comPos < 0){
        comPos = nameServer.length();
    }
    nameServer = nameServer.substr(0, comPos);
    RTC::CorbaNaming naming(rtcManager.getORB(), nameServer.c_str());
    if (!loadBodyFromModelLoader(m_robot, prop["model"].c_str(),
                                 CosNaming::NamingContext::_duplicate(naming.getRootContext())
                                 )){
        std::cerr << "failed to load model[" << prop["model"] << "]"
                  << std::endl;
    }

    coil::vstring end_effectors_str = coil::split(prop["end_effectors"], ",");
    size_t prop_num = 10;
    if (end_effectors_str.size() > 0) {
      size_t num = end_effectors_str.size()/prop_num;
      for (size_t i = 0; i < num; i++) {
        std::string ee_name, ee_target, ee_base;
        coil::stringTo(ee_name, end_effectors_str[i*prop_num].c_str());
        coil::stringTo(ee_target, end_effectors_str[i*prop_num+1].c_str());
        coil::stringTo(ee_base, end_effectors_str[i*prop_num+2].c_str());
        boost::algorithm::to_upper(ee_name);
        if (ee_name == "RARM") {
          for (size_t j = 0; j < 3; j++) {
            coil::stringTo(m_p_rarm_to_end_effector(j), end_effectors_str[i*prop_num+3+j].c_str());
          }
        }
        double tmpv[4];
        for (int j = 0; j < 4; j++ ) {
          coil::stringTo(tmpv[j], end_effectors_str[i*prop_num+6+j].c_str());
        }
        m_R_rarm_to_end_effector = Eigen::AngleAxis<double>(tmpv[3], hrp::Vector3(tmpv[0], tmpv[1], tmpv[2])).toRotationMatrix(); // rotation in VRML is represented by axis + angle
      }
    }
    hrp::Vector3 p_racket_to_end_effector;
    p_racket_to_end_effector[0] = 0.0;
    p_racket_to_end_effector[1] = -0.470;
    p_racket_to_end_effector[2] = 0.0;
    hrp::Vector3 rpy_racket_to_end_effector;
    rpy_racket_to_end_effector[0] = 2.186;
    rpy_racket_to_end_effector[1] = -0.524;
    rpy_racket_to_end_effector[2] = 2.186;
    hrp::Matrix33 R_racket_to_end_effector = rotFromRpy(rpy_racket_to_end_effector[0], rpy_racket_to_end_effector[1], rpy_racket_to_end_effector[2]);
    m_R_end_effector_to_racket = R_racket_to_end_effector.transpose();
    m_p_end_effector_to_racket = m_R_end_effector_to_racket * -p_racket_to_end_effector;
    m_R_end_effector_to_rarm = m_R_rarm_to_end_effector.transpose();
    m_p_end_effector_to_rarm = m_R_end_effector_to_rarm * -m_p_rarm_to_end_effector;
    m_p_rarm_to_racket = m_R_rarm_to_end_effector * m_p_end_effector_to_racket + m_p_rarm_to_end_effector;
    m_R_rarm_to_racket = m_R_rarm_to_end_effector * m_R_end_effector_to_racket;
    m_R_racket_to_rarm = m_R_rarm_to_racket.transpose();
    m_p_racket_to_rarm = m_R_racket_to_rarm * -m_p_rarm_to_racket;


    unsigned int dof = m_robot->numJoints();

    // dof + rootlink 6 dof - thk hand 2 * 2
    m_bsplines_length = dof + 6 - (m_isChoreonoid ? 4 : 0);
    /*
        * eustf roseus_c_util 1.irteusgl$ (require "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon-interface.l")
        * ;; extending gcstack 0x5834080[32738] --> 0x64ce790[65476] top=7ec4
        * :|PACKAGE://HRPSYS_ROS_BRIDGE_TUTORIALS/EUSLISP/JAXON-INTERFACE.L|
        * 2.irteusgl$ (jaxon)
        * #<jaxon-robot #X72d4f98 JAXON  0.0 0.0 0.0 / 0.0 0.0 0.0>
        * 3.irteusgl$ (setq *robot* *jaxon*)
        * #<jaxon-robot #X72d4f98 JAXON  0.0 0.0 0.0 / 0.0 0.0 0.0>
        * 4. irteusgl$ (send (car (send *robot* :links)) :worldcoords)
        * #<coordinates #X5517540  0.0 0.0 1032.5 / 0.0 0.0 0.0>
        * 5. irteusgl$ (with-append-root-joint (link-list-with-robot-6dof *robot* (list (cdr (send *robot* :links))) :joint-class 6dof-joint)
        * (send (send (car (send *robot* :links)) :joint) :joint-angle)))
        * #f(0.0 0.0 0.0 0.0 0.0 0.0)
        */

#warning this is offset from euslisp
    m_rootlink_6dof_offset << 0.0 / 1000.0,
        0.0 / 1000.0,
        1032.5 / 1000.0,
        0.0,
        0.0,
        0.0;
    // Setting up debug logger
    time_t timer;
    struct tm *local;
    timer = time(NULL);
    local = localtime(&timer);
    int year = local->tm_year + 1900;
    int month = local->tm_mon + 1;
    int day = local->tm_mday;
    int hour = local->tm_hour;
    int min = local->tm_min;
    int sec = local->tm_sec;
    // bool tmp = local->tm_isdst; // is summer time
    std::ostringstream oss_date;
    oss_date << std::setfill('0')
        << std::setw(4) << year << "-"
        << std::setw(2) << month << "-"
        << std::setw(2) << day << "-"
        << std::setw(2) << hour << "-"
        << std::setw(2) << min << "-"
        << std::setw(2) << sec
        << std::setfill(' ');
    std::ifstream ifs_date("/userdir/timestamp.txt");
    std::string timestamp_str_in;
    if (ifs_date) {
        ifs_date >> timestamp_str_in;
    } else {
        timestamp_str_in = "";
    }
    std::string fname_debug = "/userdir/logs/" + timestamp_str_in + "hrpsys_bsp_"
        + oss_date.str() + ".log";
    m_ofs_bsp_debug = boost::shared_ptr<std::ofstream>(new std::ofstream(fname_debug.c_str()));
    *m_ofs_bsp_debug << "###### bsp log start: " << oss_date.str() << std::endl;

    // Setting for wrench data ports (real + virtual)
    std::vector<std::string> fsensor_names;
    //   find names for real force sensors
    unsigned int npforce = m_robot->numSensors(hrp::Sensor::FORCE);
    for (unsigned int i=0; i<npforce; i++){
      fsensor_names.push_back(m_robot->sensor(hrp::Sensor::FORCE, i)->name);
    }
    //   find names for virtual force sensors
    coil::vstring virtual_force_sensor = coil::split(prop["virtual_force_sensor"], ",");
    unsigned int nvforce = virtual_force_sensor.size()/10;
    for (unsigned int i=0; i<nvforce; i++){
      fsensor_names.push_back(virtual_force_sensor[i*10+0]);
    }
    //   add ports for all force sensors
    unsigned int nforce  = npforce + nvforce;
    m_wrenches.resize(nforce);
    m_wrenchesOut.resize(nforce);
    for (unsigned int i=0; i<nforce; i++){
      m_wrenchesOut[i] = new OutPort<TimedDoubleSeq>(std::string(fsensor_names[i]+"Ref").c_str(), m_wrenches[i]);
      m_wrenches[i].data.length(6);
      registerOutPort(std::string(fsensor_names[i]+"Ref").c_str(), *m_wrenchesOut[i]);
    }

    if (prop.hasKey("seq_optional_data_dim")) {
      coil::stringTo(optional_data_dim, prop["seq_optional_data_dim"].c_str());
    } else {
      optional_data_dim = 1;
    }

    m_seq = new seqplay(dof, dt, nforce, optional_data_dim);

    m_qInit.data.length(dof);
    for (unsigned int i=0; i<dof; i++) m_qInit.data[i] = 0.0;
    Link *root = m_robot->rootLink();
    m_basePosInit.data.x = root->p[0]; m_basePosInit.data.y = root->p[1]; m_basePosInit.data.z = root->p[2];
    hrp::Vector3 rpy = hrp::rpyFromRot(root->R);
    m_baseRpyInit.data.r = rpy[0]; m_baseRpyInit.data.p = rpy[1]; m_baseRpyInit.data.y = rpy[2];
    m_zmpRefInit.data.x = 0; m_zmpRefInit.data.y = 0; m_zmpRefInit.data.z = 0;

    // allocate memory for outPorts
    m_qRef.data.length(dof);
    m_tqRef.data.length(dof);
    m_optionalData.data.length(optional_data_dim);

    return RTC::RTC_OK;
}



RTC::ReturnCode_t SequencePlayer::onFinalize()
{
    if ( m_debugLevel > 0 ) {
        std::cerr << __PRETTY_FUNCTION__ << std::endl;
    }
    return RTC::RTC_OK;
}

/*
  RTC::ReturnCode_t SequencePlayer::onStartup(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t SequencePlayer::onShutdown(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

RTC::ReturnCode_t SequencePlayer::onActivated(RTC::UniqueId ec_id)
{
    std::cout << "SequencePlayer::onActivated(" << ec_id << ")" << std::endl;

    return RTC::RTC_OK;
}

/*
  RTC::ReturnCode_t SequencePlayer::onDeactivated(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

RTC::ReturnCode_t SequencePlayer::onExecute(RTC::UniqueId ec_id)
{
    static int loop = 0;
    loop++;
    if ( m_debugLevel > 0 && loop % 1000 == 0) {
        std::cerr << __PRETTY_FUNCTION__ << "(" << ec_id << ")" << std::endl;
    }
    if (m_qInitIn.isNew()) m_qInitIn.read();
    if (m_basePosInitIn.isNew()) m_basePosInitIn.read();
    if (m_baseRpyInitIn.isNew()) m_baseRpyInitIn.read();
    if (m_zmpRefInitIn.isNew()) m_zmpRefInitIn.read();
    if (m_hitTargetIn.isNew()) {
        m_hitTargetIn.read();
        m_isTargetValid = true;
    } else {
        m_isTargetValid = false;
    }

    if (m_gname != "" && m_seq->isEmpty(m_gname.c_str())){
        if (m_waitFlag){
            m_gname = "";
            m_waitFlag = false;
            sem_post(&m_waitSem);
        }
    }

    const std::string gname = "RARM";
    if (m_rarm_indices.empty()) {
      if (! m_seq->getJointGroup(gname.c_str(), m_rarm_indices)) {
        // std::cerr << "Could not find joint group " << gname << std::endl;
      }
    }

    if (m_seq->isEmpty()){
        m_clearFlag = false;
        if (m_waitFlag){
            m_waitFlag = false;
            sem_post(&m_waitSem);
        }
    }else{
	Guard guard(m_mutex);

        // calc current q from p
        /*
         * TODO
         * m_bsplines -- ok
         * m_p -- ok
         * m_tCurrent -- ok
         * m_tHit -- ok
         * がvalidになるようにしておく
         * validでないときはループをスルーする
         */
        // ここでタイマーを更新する
        if (m_onlineModifyStarted) {
          m_tCurrent += dt;
          if (m_isTargetValid) {
            this->onlineTrajectoryModification();
          }
        }

        if (m_future.is_ready()) {
          if (m_qp_last_ready) {
            m_qp_ready = false;
          } else {
            m_qp_ready = true;
          }
          m_qp_last_ready = true;
        }

        if (m_qp_ready) {
          *m_ofs_bsp_debug << "dp is modified" << std::endl;
          hrp::dvector dp = m_future.get();
          *m_ofs_bsp_debug << "dp: " << std::endl;
          for (int i = 0; i < m_bsplines_length; i++) {
              for (int j = 0; j < m_id_max; j++) {
                  *m_ofs_bsp_debug << dp[m_id_max * i + j] << " ";
              }
              *m_ofs_bsp_debug << std::endl;
          }
          *m_ofs_bsp_debug << dp[dp.size() - 1] << std::endl;
          m_p += dp;
        }

        double zmp[3], acc[3], pos[3], rpy[3], wrenches[6*m_wrenches.size()];
        m_seq->get(m_qRef.data.get_buffer(), zmp, acc, pos, rpy, m_tqRef.data.get_buffer(), wrenches, m_optionalData.data.get_buffer());

        m_zmpRef.data.x = zmp[0];
        m_zmpRef.data.y = zmp[1];
        m_zmpRef.data.z = zmp[2];
        m_accRef.data.ax = acc[0];
        m_accRef.data.ay = acc[1];
        m_accRef.data.az = acc[2];

        if (m_onlineModifyStarted and m_tCurrent > m_tMax) {
            m_onlineModifyStarted = false;
        }
        if (m_onlineModifyStarted) {
            // overwrite for online trajectory modification
            // m_qRef.data.length() is 37(numJoints(=33) + THKhand(2*2))
            // m_bsplines.size() is 39(numJoints(=33) + rootlink)
            // rarm only (because original jpos is calculated with both feet ik onto the ground)
            for (int i = m_rarm_indices.at(0); i <= m_rarm_indices.at(m_rarm_indices.size() - 1); i++) {
                m_qRef.data[i] = m_bsplines.at(i).calc(m_tCurrent, m_p.segment(m_id_max * i, m_id_max));
            }
        }

        if (m_fixedLink != ""){
            for (int i=0; i<m_robot->numJoints(); i++){
                m_robot->joint(i)->q = m_qRef.data[i];
            }
            for (int i=0; i<3; i++){
                m_robot->rootLink()->p[i] = pos[i];
            }
            m_robot->rootLink()->R = hrp::rotFromRpy(rpy[0], rpy[1], rpy[2]);
            m_robot->calcForwardKinematics();
            hrp::Link *root = m_robot->rootLink();
            hrp::Vector3 rootP;
            hrp::Matrix33 rootR;
            if (m_timeToStartPlaying > 0){
                m_timeToStartPlaying -= dt;
                hrp::Link *fixed = m_robot->link(m_fixedLink);
                hrp::Matrix33 fixed2rootR = fixed->R.transpose()*root->R;
                hrp::Vector3 fixed2rootP = fixed->R.transpose()*(root->p - fixed->p);
                rootR = m_fixedR*fixed2rootR;
                rootP = m_fixedR*fixed2rootP + m_fixedP;
            }else{
                rootR = m_offsetR*m_robot->rootLink()->R;
                rootP = m_offsetR*m_robot->rootLink()->p + m_offsetP;
            }
            hrp::Vector3 rootRpy = hrp::rpyFromRot(rootR);
            pos[0] = rootP[0];
            pos[1] = rootP[1];
            pos[2] = rootP[2];
            rpy[0] = rootRpy[0];
            rpy[1] = rootRpy[1];
            rpy[2] = rootRpy[2];
        }
        /* これはB-Splineの座標系が正しい必要がある
#warning ここの値の順番が正しいか，またdegなのかradなのか調べる
        if (m_onlineModifyStarted) {
            int joint_num_without_thk = m_robot->numJoints() - (m_isChoreonoid ? 4 : 0);
#warning this is wrong because rotation is not took into account
            pos[0] = m_bsplines.at(joint_num_without_thk + 0).calc(m_tCurrent, m_p.segment(m_id_max * (joint_num_without_thk + 0), m_id_max)) + m_rootlink_6dof_offset[0];
            pos[1] = m_bsplines.at(joint_num_without_thk + 1).calc(m_tCurrent, m_p.segment(m_id_max * (joint_num_without_thk + 1), m_id_max)) + m_rootlink_6dof_offset[1];
            pos[2] = m_bsplines.at(joint_num_without_thk + 2).calc(m_tCurrent, m_p.segment(m_id_max * (joint_num_without_thk + 2), m_id_max)) + m_rootlink_6dof_offset[2];
#warning in euslisp data alignment is ypr?
            rpy[0] = m_bsplines.at(joint_num_without_thk + 5).calc(m_tCurrent, m_p.segment(m_id_max * (joint_num_without_thk + 5), m_id_max)) + m_rootlink_6dof_offset[3];
            rpy[1] = m_bsplines.at(joint_num_without_thk + 4).calc(m_tCurrent, m_p.segment(m_id_max * (joint_num_without_thk + 4), m_id_max)) + m_rootlink_6dof_offset[4];
            rpy[2] = m_bsplines.at(joint_num_without_thk + 3).calc(m_tCurrent, m_p.segment(m_id_max * (joint_num_without_thk + 3), m_id_max)) + m_rootlink_6dof_offset[5];
        }
        */
        m_basePos.data.x = pos[0];
        m_basePos.data.y = pos[1];
        m_basePos.data.z = pos[2];
        m_baseRpy.data.r = rpy[0];
        m_baseRpy.data.p = rpy[1];
        m_baseRpy.data.y = rpy[2];
        size_t force_i = 0;
        for (size_t i = 0; i < m_wrenches.size(); i++) {
          m_wrenches[i].data[0] = wrenches[force_i++];
          m_wrenches[i].data[1] = wrenches[force_i++];
          m_wrenches[i].data[2] = wrenches[force_i++];
          m_wrenches[i].data[3] = wrenches[force_i++];
          m_wrenches[i].data[4] = wrenches[force_i++];
          m_wrenches[i].data[5] = wrenches[force_i++];
        }
        m_qRef.tm = m_qInit.tm;
        m_qRefOut.write();
        m_tqRefOut.write();
        m_zmpRefOut.write();
        m_accRefOut.write();
        m_basePosOut.write();
        m_baseRpyOut.write();
        m_optionalDataOut.write();
        for (size_t i = 0; i < m_wrenchesOut.size(); i++) {
          m_wrenchesOut[i]->write();
        }

        if (m_clearFlag){
            m_seq->clear(0.001);
        }
    }
    return RTC::RTC_OK;
}

/*
  RTC::ReturnCode_t SequencePlayer::onAborting(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t SequencePlayer::onError(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t SequencePlayer::onReset(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t SequencePlayer::onStateUpdate(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t SequencePlayer::onRateChanged(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

void SequencePlayer::setClearFlag()
{
    if ( m_debugLevel > 0 ) {
        std::cerr << __PRETTY_FUNCTION__ << std::endl;
    }
    m_clearFlag = true;
}

void SequencePlayer::waitInterpolation()
{
    if ( m_debugLevel > 0 ) {
        std::cerr << __PRETTY_FUNCTION__ << std::endl;
    }
    m_waitFlag = true;
    sem_wait(&m_waitSem);
}

bool SequencePlayer::waitInterpolationOfGroup(const char *gname)
{
    if ( m_debugLevel > 0 ) {
        std::cerr << __PRETTY_FUNCTION__ << std::endl;
    }
    m_gname = gname;
    m_waitFlag = true;
    sem_wait(&m_waitSem);
    return true;
}


bool SequencePlayer::setJointAngle(short id, double angle, double tm)
{
    if ( m_debugLevel > 0 ) {
        std::cerr << __PRETTY_FUNCTION__ << std::endl;
    }
    Guard guard(m_mutex);
    if (!setInitialState()) return false;
    dvector q(m_robot->numJoints());
    m_seq->getJointAngles(q.data());
    q[id] = angle;
    for (unsigned int i=0; i<m_robot->numJoints(); i++){
        hrp::Link *j = m_robot->joint(i);
        if (j) j->q = q[i];
    }
    m_robot->calcForwardKinematics();
    hrp::Vector3 absZmp = m_robot->calcCM();
    absZmp[2] = 0;
    hrp::Link *root = m_robot->rootLink();
    hrp::Vector3 relZmp = root->R.transpose()*(absZmp - root->p);
    m_seq->setJointAngles(q.data(), tm);
    m_seq->setZmp(relZmp.data(), tm);
    return true;
}

bool SequencePlayer::setJointAngles(const double *angles, double tm)
{
    if ( m_debugLevel > 0 ) {
        std::cerr << __PRETTY_FUNCTION__ << std::endl;
    }
    Guard guard(m_mutex);
    if (!setInitialState()) return false;
    for (unsigned int i=0; i<m_robot->numJoints(); i++){
        hrp::Link *j = m_robot->joint(i);
        if (j) j->q = angles[i];
    }
    m_robot->calcForwardKinematics();
    hrp::Vector3 absZmp = m_robot->calcCM();
    absZmp[2] = 0;
    hrp::Link *root = m_robot->rootLink();
    hrp::Vector3 relZmp = root->R.transpose()*(absZmp - root->p);
    std::vector<const double*> v_poss;
    std::vector<double> v_tms;
    v_poss.push_back(angles);
    v_tms.push_back(tm);
    m_seq->setJointAnglesSequence(v_poss, v_tms);
    m_seq->setZmp(relZmp.data(), tm);
    return true;
}

bool SequencePlayer::setJointAngles(const double *angles, const bool *mask,
                                    double tm)
{
    if ( m_debugLevel > 0 ) {
        std::cerr << __PRETTY_FUNCTION__ << std::endl;
    }
    Guard guard(m_mutex);

    if (!setInitialState()) return false;

    double pose[m_robot->numJoints()];
    for (unsigned int i=0; i<m_robot->numJoints(); i++){
        pose[i] = mask[i] ? angles[i] : m_qInit.data[i];
    }
    m_seq->setJointAngles(pose, tm);
    return true;
}

bool SequencePlayer::setJointAnglesSequence(const OpenHRP::dSequenceSequence angless, const OpenHRP::bSequence& mask, const OpenHRP::dSequence& times)
{
    if ( m_debugLevel > 0 ) {
        std::cerr << __PRETTY_FUNCTION__ << std::endl;
    }
    Guard guard(m_mutex);

    if (!setInitialState()) return false;

    bool tmp_mask[robot()->numJoints()];
    if (mask.length() != robot()->numJoints()) {
        for (unsigned int i=0; i < robot()->numJoints(); i++) tmp_mask[i] = true;
    }else{
        for (unsigned int i=0; i < robot()->numJoints(); i++) tmp_mask[i] = mask.get_buffer()[i];
    }
    int len = angless.length();
    std::vector<const double*> v_poss;
    std::vector<double> v_tms;
    for ( unsigned int i = 0; i < angless.length(); i++ ) v_poss.push_back(angless[i].get_buffer());
    for ( unsigned int i = 0; i <  times.length();  i++ )  v_tms.push_back(times[i]);
    return m_seq->setJointAnglesSequence(v_poss, v_tms);
}

bool SequencePlayer::clearJointAngles()
{
    if ( m_debugLevel > 0 ) {
        std::cerr << __PRETTY_FUNCTION__ << std::endl;
    }
    Guard guard(m_mutex);

    if (!setInitialState()) return false;

    return m_seq->clearJointAngles();
}

bool SequencePlayer::setJointAnglesSequenceOfGroup(const char *gname, const OpenHRP::dSequenceSequence angless, const OpenHRP::dSequence& times)
{
    if ( m_debugLevel > 0 ) {
        std::cerr << __PRETTY_FUNCTION__ << std::endl;
    }
    Guard guard(m_mutex);
    if (!setInitialState()) return false;

    if (!m_seq->resetJointGroup(gname, m_qInit.data.get_buffer())) return false;

    std::vector<const double*> v_poss;
    std::vector<double> v_tms;
    for ( unsigned int i = 0; i < angless.length(); i++ ) v_poss.push_back(angless[i].get_buffer());
    for ( unsigned int i = 0; i <  times.length();  i++ )  v_tms.push_back(times[i]);
    return m_seq->setJointAnglesSequenceOfGroup(gname, v_poss, v_tms, angless.length()>0?angless[0].length():0);
}

bool SequencePlayer::clearJointAnglesOfGroup(const char *gname)
{
    if ( m_debugLevel > 0 ) {
        std::cerr << __PRETTY_FUNCTION__ << std::endl;
    }
    Guard guard(m_mutex);
    if (!setInitialState()) return false;

    if (!m_seq->resetJointGroup(gname, m_qInit.data.get_buffer())) return false;

    return m_seq->clearJointAnglesOfGroup(gname);
}

bool SequencePlayer::setJointAnglesSequenceFull(const OpenHRP::dSequenceSequence i_jvss, const OpenHRP::dSequenceSequence i_vels, const OpenHRP::dSequenceSequence i_torques, const OpenHRP::dSequenceSequence i_poss, const OpenHRP::dSequenceSequence i_rpys, const OpenHRP::dSequenceSequence i_accs, const OpenHRP::dSequenceSequence i_zmps, const OpenHRP::dSequenceSequence i_wrenches, const OpenHRP::dSequenceSequence i_optionals, const dSequence i_tms)
{
    if ( m_debugLevel > 0 ) {
        std::cerr << __PRETTY_FUNCTION__ << std::endl;
    }
    Guard guard(m_mutex);

    if (!setInitialState()) return false;

    int len = i_jvss.length();
    std::vector<const double*> v_jvss, v_vels, v_torques, v_poss, v_rpys, v_accs, v_zmps, v_wrenches, v_optionals;
    std::vector<double> v_tms;
    for ( unsigned int i = 0; i < i_jvss.length(); i++ ) v_jvss.push_back(i_jvss[i].get_buffer());
    for ( unsigned int i = 0; i < i_vels.length(); i++ ) v_vels.push_back(i_vels[i].get_buffer());
    for ( unsigned int i = 0; i < i_torques.length(); i++ ) v_torques.push_back(i_torques[i].get_buffer());
    for ( unsigned int i = 0; i < i_poss.length(); i++ ) v_poss.push_back(i_poss[i].get_buffer());
    for ( unsigned int i = 0; i < i_rpys.length(); i++ ) v_rpys.push_back(i_rpys[i].get_buffer());
    for ( unsigned int i = 0; i < i_accs.length(); i++ ) v_accs.push_back(i_accs[i].get_buffer());
    for ( unsigned int i = 0; i < i_zmps.length(); i++ ) v_zmps.push_back(i_zmps[i].get_buffer());
    for ( unsigned int i = 0; i < i_wrenches.length(); i++ ) v_wrenches.push_back(i_wrenches[i].get_buffer());
    for ( unsigned int i = 0; i < i_optionals.length(); i++ ) v_optionals.push_back(i_optionals[i].get_buffer());
    for ( unsigned int i = 0; i < i_tms.length();  i++ )  v_tms.push_back(i_tms[i]);
    *m_ofs_bsp_debug << "SequencePlayer::setJointAnglesSequenceFull called" << std::endl;
    return m_seq->setJointAnglesSequenceFull(v_jvss, v_vels, v_torques, v_poss, v_rpys, v_accs, v_zmps, v_wrenches, v_optionals, v_tms);
}

bool SequencePlayer::setJointAnglesSequenceFullWithBSpline(short i_bsorder, short i_bsid, double i_bstmin, double i_bsthit, double i_bstmax, const OpenHRP::dSequence i_bsp, const OpenHRP::dSequenceSequence i_jvss, const OpenHRP::dSequenceSequence i_vels, const OpenHRP::dSequenceSequence i_torques, const OpenHRP::dSequenceSequence i_poss, const OpenHRP::dSequenceSequence i_rpys, const OpenHRP::dSequenceSequence i_accs, const OpenHRP::dSequenceSequence i_zmps, const OpenHRP::dSequenceSequence i_wrenches, const OpenHRP::dSequenceSequence i_optionals, const dSequence i_tms)
{
    if ( m_debugLevel > 0 ) {
        std::cerr << __PRETTY_FUNCTION__ << std::endl;
    }
    Guard guard(m_mutex);

    if (!setInitialState()) return false;

    m_onlineModifyStarted = true;
    m_id_max = i_bsid;
    m_tMin = i_bstmin;
    m_tMax = i_bstmax;
    m_tCurrent = i_bstmin;
    m_tHit = i_bsthit;
    // 初期化処理
    m_bsplines.clear();
    for (int i = 0; i < m_bsplines_length; i++) {
        m_bsplines.push_back(BSpline::BSpline(i_bsorder, i_bsorder, i_bsid, i_bsid, i_bstmin, i_bstmax));
    }

    int len = i_jvss.length();
    std::vector<const double*> v_jvss, v_vels, v_torques, v_poss, v_rpys, v_accs, v_zmps, v_wrenches, v_optionals;
    std::vector<double> v_tms;
    m_p = hrp::dvector::Zero(i_bsp.length());
    for ( unsigned int i = 0; i < i_bsp.length(); i++ ) m_p[i] = i_bsp[i];
    for ( unsigned int i = 0; i < i_jvss.length(); i++ ) v_jvss.push_back(i_jvss[i].get_buffer());
    for ( unsigned int i = 0; i < i_vels.length(); i++ ) v_vels.push_back(i_vels[i].get_buffer());
    for ( unsigned int i = 0; i < i_torques.length(); i++ ) v_torques.push_back(i_torques[i].get_buffer());
    for ( unsigned int i = 0; i < i_poss.length(); i++ ) v_poss.push_back(i_poss[i].get_buffer());
    for ( unsigned int i = 0; i < i_rpys.length(); i++ ) v_rpys.push_back(i_rpys[i].get_buffer());
    for ( unsigned int i = 0; i < i_accs.length(); i++ ) v_accs.push_back(i_accs[i].get_buffer());
    for ( unsigned int i = 0; i < i_zmps.length(); i++ ) v_zmps.push_back(i_zmps[i].get_buffer());
    for ( unsigned int i = 0; i < i_wrenches.length(); i++ ) v_wrenches.push_back(i_wrenches[i].get_buffer());
    for ( unsigned int i = 0; i < i_optionals.length(); i++ ) v_optionals.push_back(i_optionals[i].get_buffer());
    for ( unsigned int i = 0; i < i_tms.length();  i++ )  v_tms.push_back(i_tms[i]);
    *m_ofs_bsp_debug << "SequencePlayer::setJointAnglesSequenceFullWithBSpline called" << std::endl;
    return m_seq->setJointAnglesSequenceFullWithBSpline(v_jvss, v_vels, v_torques, v_poss, v_rpys, v_accs, v_zmps, v_wrenches, v_optionals, v_tms);
}

bool SequencePlayer::setBasePos(const double *pos, double tm)
{
    if ( m_debugLevel > 0 ) {
        std::cerr << __PRETTY_FUNCTION__ << std::endl;
    }
    Guard guard(m_mutex);
    m_seq->setBasePos(pos, tm);
    return true;
}

bool SequencePlayer::setBaseRpy(const double *rpy, double tm)
{
    if ( m_debugLevel > 0 ) {
        std::cerr << __PRETTY_FUNCTION__ << std::endl;
    }
    Guard guard(m_mutex);
    m_seq->setBaseRpy(rpy, tm);
    return true;
}

bool SequencePlayer::setZmp(const double *zmp, double tm)
{
    if ( m_debugLevel > 0 ) {
        std::cerr << __PRETTY_FUNCTION__ << std::endl;
    }
    Guard guard(m_mutex);
    m_seq->setZmp(zmp, tm);
    return true;
}

bool SequencePlayer::setWrenches(const double *wrenches, double tm)
{
    Guard guard(m_mutex);
    m_seq->setWrenches(wrenches, tm);
    return true;
}

bool SequencePlayer::setTargetPose(const char* gname, const double *xyz, const double *rpy, double tm, const char* frame_name)
{
    if ( m_debugLevel > 0 ) {
        std::cerr << __PRETTY_FUNCTION__ << std::endl;
    }
    Guard guard(m_mutex);
    if (!setInitialState()) return false;
    // setup
    std::vector<int> indices;
    hrp::dvector start_av, end_av;
    std::vector<hrp::dvector> avs;
    if (! m_seq->getJointGroup(gname, indices) ) {
        std::cerr << "[setTargetPose] Could not find joint group " << gname << std::endl;
        return false;
    }
    start_av.resize(indices.size());
    end_av.resize(indices.size());

    //std::cerr << std::endl;
    if ( ! m_robot->joint(indices[0])->parent ) {
        std::cerr << "[setTargetPose] " << m_robot->joint(indices[0])->name << " does not have parent" << std::endl;
        return false;
    }
    string base_parent_name = m_robot->joint(indices[0])->parent->name;
    string target_name = m_robot->joint(indices[indices.size()-1])->name;
    // prepare joint path
    hrp::JointPathExPtr manip = hrp::JointPathExPtr(new hrp::JointPathEx(m_robot, m_robot->link(base_parent_name), m_robot->link(target_name), dt, true, std::string(m_profile.instance_name)));

    // calc fk
    for (unsigned int i=0; i<m_robot->numJoints(); i++){
        hrp::Link *j = m_robot->joint(i);
        if (j) j->q = m_qRef.data.get_buffer()[i];
    }
    m_robot->calcForwardKinematics();
    for ( unsigned int i = 0; i < manip->numJoints(); i++ ){
        start_av[i] = manip->joint(i)->q;
    }

    // xyz and rpy are relateive to root link, where as pos and rotatoin of manip->calcInverseKinematics are relative to base link

    // ik params
    hrp::Vector3 start_p(m_robot->link(target_name)->p);
    hrp::Matrix33 start_R(m_robot->link(target_name)->R);
    hrp::Vector3 end_p(xyz[0], xyz[1], xyz[2]);
    hrp::Matrix33 end_R = m_robot->link(target_name)->calcRfromAttitude(hrp::rotFromRpy(rpy[0], rpy[1], rpy[2]));

    // change start and end must be relative to the frame_name
    if ( (frame_name != NULL) && (! m_robot->link(frame_name) ) ) {
        std::cerr << "[setTargetPose] Could not find frame_name " << frame_name << std::endl;
        return false;
    } else if ( frame_name != NULL ) {
        hrp::Vector3 frame_p(m_robot->link(frame_name)->p);
        hrp::Matrix33 frame_R(m_robot->link(frame_name)->attitude());
        // fix start/end references from root to frame;
        end_p = frame_R * end_p + frame_p;
        end_R = frame_R * end_R;
    }
    manip->setMaxIKError(m_error_pos,m_error_rot);
    manip->setMaxIKIteration(m_iteration);
    std::cerr << "[setTargetPose] Solveing IK with frame" << frame_name << ", Error " << m_error_pos << m_error_rot << ", Iteration " << m_iteration << std::endl;
    std::cerr << "                Start " << start_p << start_R<< std::endl;
    std::cerr << "                End   " << end_p << end_R<< std::endl;

    // interpolate & calc ik
    int len = max(((start_p - end_p).norm() / 0.02 ), // 2cm
                  ((hrp::omegaFromRot(start_R.transpose() * end_R).norm()) / 0.025)); // 2 deg
    len = max(len, 1);

    std::vector<const double *> v_pos;
    std::vector<double> v_tm;
    v_pos.resize(len);
    v_tm.resize(len);

    // do loop
    for (int i = 0; i < len; i++ ) {
        double a = (1+i)/(double)len;
        hrp::Vector3 p = (1-a)*start_p + a*end_p;
        hrp::Vector3 omega = hrp::omegaFromRot(start_R.transpose() * end_R);
        hrp::Matrix33 R = start_R * rodrigues(omega.isZero()?omega:omega.normalized(), a*omega.norm());
        bool ret = manip->calcInverseKinematics2(p, R);

        if ( m_debugLevel > 0 ) {
            // for debug
            std::cerr << "target pos/rot : " << i << "/" << a << " : "
                      << p[0] << " " << p[1] << " " << p[2] << ","
                      << omega[0] << " " << omega[1] << " " << omega[2] << std::endl;
        }
        if ( ! ret ) {
            std::cerr << "[setTargetPose] IK failed" << std::endl;
            return false;
        }
        v_pos[i] = (const double *)malloc(sizeof(double)*manip->numJoints());
        for ( unsigned int j = 0; j < manip->numJoints(); j++ ){
            ((double *)v_pos[i])[j] = manip->joint(j)->q;
        }
        v_tm[i] = tm/len;
    }

    if ( m_debugLevel > 0 ) {
        // for debug
        for(int i = 0; i < len; i++ ) {
            std::cerr << v_tm[i] << ":";
            for(int j = 0; j < start_av.size(); j++ ) {
                std::cerr << v_pos[i][j] << " ";
            }
            std::cerr << std::endl;
        }
    }

    if (!m_seq->resetJointGroup(gname, m_qInit.data.get_buffer())) return false; // reset sequencer
    bool ret = m_seq->playPatternOfGroup(gname, v_pos, v_tm, m_qInit.data.get_buffer(), v_pos.size()>0?indices.size():0);

    // clean up memory, need to improve
    for (int i = 0; i < len; i++ ) {
        free((double *)v_pos[i]);
    }

    return ret;
}

void SequencePlayer::loadPattern(const char *basename, double tm)
{
    if ( m_debugLevel > 0 ) {
        std::cerr << __PRETTY_FUNCTION__ << std::endl;
    }
    Guard guard(m_mutex);
    if (setInitialState()){
        if (m_fixedLink != ""){
            hrp::Link *l = m_robot->link(m_fixedLink);
            if (!l) {
                std::cerr << __PRETTY_FUNCTION__ << "can't find a fixed link("
                          << m_fixedLink << ")" << std::endl;
                m_fixedLink = "";
                return;
            }
            m_robot->calcForwardKinematics(); // this is not called by setinitialstate()
            m_fixedP = l->p;
            m_fixedR = l->R;

            std::string pos = std::string(basename)+".pos";
            std::string wst = std::string(basename)+".waist";
            std::ifstream ifspos(pos.c_str());
            std::ifstream ifswst(wst.c_str());
            if (!ifspos.is_open() || !ifswst.is_open()){
                std::cerr << __PRETTY_FUNCTION__ << "can't open " << pos << " or "
                          << wst << ")" << std::endl;
                m_fixedLink = "";
                return;
            }
            double time;
            ifspos >> time;
            for (int i=0; i<m_robot->numJoints(); i++){
                ifspos >> m_robot->joint(i)->q;
            }
            ifswst >> time;
            for (int i=0; i<3; i++) ifswst >> m_robot->rootLink()->p[i];
            hrp::Vector3 rpy;
            for (int i=0; i<3; i++) ifswst >> rpy[i];
            m_robot->rootLink()->R = hrp::rotFromRpy(rpy);
            m_robot->calcForwardKinematics();

            m_offsetR = m_fixedR*l->R.transpose();
            m_offsetP = m_fixedP - m_offsetR*l->p;
            m_timeToStartPlaying = tm;
        }
        m_seq->loadPattern(basename, tm);
    }
}

bool SequencePlayer::setInitialState(double tm)
{
    if ( m_debugLevel > 0 ) {
        std::cerr << __PRETTY_FUNCTION__ << "m_seq-isEmpty() " << m_seq->isEmpty() << ", m_Init.data.length() " << m_qInit.data.length() << std::endl;
    }
    if (!m_seq->isEmpty()) return true;

    if (m_qInit.data.length() == 0){
        std::cerr << "can't determine initial posture" << std::endl;
        return false;
    }else{
        m_seq->setJointAngles(m_qInit.data.get_buffer(), tm);
        for (unsigned int i=0; i<m_robot->numJoints(); i++){
            Link *l = m_robot->joint(i);
            l->q = m_qInit.data[i];
            m_qRef.data[i] = m_qInit.data[i]; // update m_qRef for setTargetPose()
        }

        Link *root = m_robot->rootLink();

        root->p << m_basePosInit.data.x,
            m_basePosInit.data.y,
            m_basePosInit.data.z;
        m_seq->setBasePos(root->p.data(), tm);

        double rpy[] = {m_baseRpyInit.data.r,
                        m_baseRpyInit.data.p,
                        m_baseRpyInit.data.y};
        m_seq->setBaseRpy(rpy, tm);
        calcRotFromRpy(root->R, rpy[0], rpy[1], rpy[2]);

        double zmp[] = {m_zmpRefInit.data.x, m_zmpRefInit.data.y, m_zmpRefInit.data.z};
        m_seq->setZmp(zmp, tm);
        double zero[] = {0,0,0};
        m_seq->setBaseAcc(zero, tm);
        return true;
    }
}

void SequencePlayer::playPattern(const dSequenceSequence& pos, const dSequenceSequence& rpy, const dSequenceSequence& zmp, const dSequence& tm)
{
    if ( m_debugLevel > 0 ) {
        std::cerr << __PRETTY_FUNCTION__ << std::endl;
    }
    Guard guard(m_mutex);
    if (!setInitialState()) return;

    std::vector<const double *> v_pos, v_rpy, v_zmp;
    std::vector<double> v_tm;
    for ( unsigned int i = 0; i < pos.length(); i++ ) v_pos.push_back(pos[i].get_buffer());
    for ( unsigned int i = 0; i < rpy.length(); i++ ) v_rpy.push_back(rpy[i].get_buffer());
    for ( unsigned int i = 0; i < zmp.length(); i++ ) v_zmp.push_back(zmp[i].get_buffer());
    for ( unsigned int i = 0; i < tm.length() ; i++ ) v_tm.push_back(tm[i]);
    return m_seq->playPattern(v_pos, v_rpy, v_zmp, v_tm, m_qInit.data.get_buffer(), pos.length()>0?pos[0].length():0);
}

bool SequencePlayer::setInterpolationMode(OpenHRP::SequencePlayerService::interpolationMode i_mode_)
{
    if ( m_debugLevel > 0 ) {
        std::cerr << __PRETTY_FUNCTION__ << std::endl;
    }
    Guard guard(m_mutex);
    interpolator::interpolation_mode new_mode;
    if (i_mode_ == OpenHRP::SequencePlayerService::LINEAR){
        new_mode = interpolator::LINEAR;
    }else if (i_mode_ == OpenHRP::SequencePlayerService::HOFFARBIB){
        new_mode = interpolator::HOFFARBIB;
    }else{
        return false;
    }
    return m_seq->setInterpolationMode(new_mode);
}

bool SequencePlayer::addJointGroup(const char *gname, const OpenHRP::SequencePlayerService::StrSequence& jnames)
{
    std::cerr << "[addJointGroup] group name = " << gname << std::endl;
    if ( m_debugLevel > 0 ) {
        std::cerr << __PRETTY_FUNCTION__ << std::endl;
    }
    if (!waitInterpolationOfGroup(gname)) return false;

    Guard guard(m_mutex);
    std::vector<int> indices;
    for (size_t i=0; i<jnames.length(); i++){
        hrp::Link *l = m_robot->link(std::string(jnames[i]));
        if (l){
            indices.push_back(l->jointId);
        }else{
            std::cerr << "[addJointGroup] link name " << jnames[i] << "is not found" << std::endl;
            return false;
        }
    }
    return m_seq->addJointGroup(gname, indices);
}

bool SequencePlayer::removeJointGroup(const char *gname)
{
    std::cerr << "[removeJointGroup] group name = " << gname << std::endl;
    if (!waitInterpolationOfGroup(gname)) return false;
    bool ret;
    {
        Guard guard(m_mutex);
        ret = m_seq->removeJointGroup(gname);
    }
    return ret;
}

bool SequencePlayer::setJointAnglesOfGroup(const char *gname, const dSequence& jvs, double tm)
{
    if ( m_debugLevel > 0 ) {
        std::cerr << __PRETTY_FUNCTION__ << std::endl;
    }
    Guard guard(m_mutex);
    if (!setInitialState()) return false;

    if (!m_seq->resetJointGroup(gname, m_qInit.data.get_buffer())) return false;
    return m_seq->setJointAnglesOfGroup(gname, jvs.get_buffer(), jvs.length(), tm);
}

bool SequencePlayer::playPatternOfGroup(const char *gname, const dSequenceSequence& pos, const dSequence& tm)
{
    if ( m_debugLevel > 0 ) {
        std::cerr << __PRETTY_FUNCTION__ << std::endl;
    }
    Guard guard(m_mutex);
    if (!setInitialState()) return false;

    std::vector<const double *> v_pos;
    std::vector<double> v_tm;
    for ( unsigned int i = 0; i < pos.length(); i++ ) v_pos.push_back(pos[i].get_buffer());
    for ( unsigned int i = 0; i < tm.length() ; i++ ) v_tm.push_back(tm[i]);
    return m_seq->playPatternOfGroup(gname, v_pos, v_tm, m_qInit.data.get_buffer(), pos.length()>0?pos[0].length():0);
}

void SequencePlayer::setMaxIKError(double pos, double rot){
    m_error_pos = pos;
    m_error_rot = rot;
}

void SequencePlayer::setMaxIKIteration(short iter){
    m_iteration= iter;
}
/*
 * @brief BSpline関数で示された各関節軌道をオンライン調整する関数
 * @param 関節角をまとめた配列
 * @note この関数を呼ぶ際は上位の関節角指定を必ず書き換え続けなければならない．
 * この関数を呼ぶ前に以下の変数は初期化されていなければならない．
 * m_bsplines(関節数+rootlinkの6dofの長さある．1要素あたりの山の数はm_id_maxという変数で管理することになる)
 * m_p(bsplineの制御点は関節ごとにm_id_max個あるが，これをflattenして(関節数+6)*m_id_maxとしたもの)
 * m_tCurrent
 * m_tHit(関節角がこの時刻で正しくなっていて欲しいという時刻)
 * m_target(ラケット面の目標位置姿勢)
 * 補正のtarget(6次元)
 */
void SequencePlayer::onlineTrajectoryModification(){
#warning TODO ターゲットが正しいか判定 フィルタかけるかも
    struct timeval time_s, time_e;
    gettimeofday(&time_s, NULL);

    m_modificator = boost::shared_ptr<OnlineTrajectoryModificatorMT>(new OnlineTrajectoryModificatorMT(
                m_robot,
                m_rarm_indices,
                dt,
                m_ofs_bsp_debug,
                std::string(m_profile.instance_name),
                m_p_rarm_to_racket,
                m_R_rarm_to_racket,
                m_p_rarm_to_end_effector,
                m_R_rarm_to_end_effector,
                m_p_end_effector_to_racket,
                m_R_end_effector_to_racket,
                m_hitTarget,
                m_p,
                m_bsplines,
                m_tCurrent,
                m_tHit,
                m_isChoreonoid,
                m_bsplines_length,
                m_rootlink_6dof_offset,
                m_id_max)); // boost::make_shared takes up to 9 arguments
    m_task = boost::make_shared<boost::packaged_task<hrp::dvector> >(boost::bind(&OnlineTrajectoryModificatorMT::calc, m_modificator.get()));
    m_qp_ready = false;
    m_qp_last_ready = false;
    m_future = m_task->get_future();
    m_thread = boost::make_shared<boost::thread>(boost::move(*m_task));
    gettimeofday(&time_e, NULL);
    *m_ofs_bsp_debug << "onlineTrajectoryModification elapsed time: " << time_e.tv_sec - time_s.tv_sec + (time_e.tv_usec - time_s.tv_usec)*1.0e-6 << "[s]" << std::endl;
}


extern "C"
{

    void SequencePlayerInit(RTC::Manager* manager)
    {
        RTC::Properties profile(sequenceplayer_spec);
        manager->registerFactory(profile,
                                 RTC::Create<SequencePlayer>,
                                 RTC::Delete<SequencePlayer>);
    }

};

// clang-format on

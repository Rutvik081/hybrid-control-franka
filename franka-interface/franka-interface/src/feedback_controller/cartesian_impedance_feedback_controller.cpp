//
// Created by mohit on 11/25/18.
//

#include "franka-interface/feedback_controller/cartesian_impedance_feedback_controller.h"

#include <exception>

#include "franka-interface/trajectory_generator/pose_trajectory_generator.h"
#include "pseudo_inversion.h"

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/SVD>
#include <iostream>
#include <fstream>


void CartesianImpedanceFeedbackController::parse_parameters() {
  // First parameter is reserved for the type

  int data_size = (params_[1] + (params_[2] << 8) + (params_[3] << 16) + (params_[4] << 24));

  bool parsed_params = cartesian_impedance_feedback_params_.ParseFromArray(params_ + 5, data_size);

  if(parsed_params){

    if(cartesian_impedance_feedback_params_.translational_stiffnesses_size() == 3){
      for(size_t i = 0; i < 3; i++) {
        translational_stiffnesses_[i] = cartesian_impedance_feedback_params_.translational_stiffnesses(i);
      }
    }

    if(cartesian_impedance_feedback_params_.rotational_stiffnesses_size() == 3){
      for(size_t i = 0; i < 3; i++) {
        rotational_stiffnesses_[i] = cartesian_impedance_feedback_params_.rotational_stiffnesses(i);
      }
    }
  } else {
    std::cout << "Parsing CartesianImpedanceFeedbackController params failed. Data size = " << data_size << std::endl;
  }
}

void CartesianImpedanceFeedbackController::initialize_controller(FrankaRobot *robot) {
  model_ = robot->getModel();

  stiffness_ = Eigen::MatrixXd(6,6);
  stiffness_.setZero();
  damping_ = Eigen::MatrixXd(6,6);
  damping_.setZero();
  nullspace_stiffness = Eigen::MatrixXd(7,7);
  nullspace_stiffness.setZero();
  nullspace_damping = Eigen::MatrixXd(7,7);
  nullspace_damping.setZero();
  q_nullspace = Eigen::VectorXd(7);
  q_nullspace << 0, 0.1963, 0, -2.6180, 0, 2.9416, 0.7854;
  std::array<double,7> ns_p = {12.0,48.0,12.0,48.0,12.0,48.0,12.0};

  for (int i = 0; i < 3; i++) {
    stiffness_(i, i) = translational_stiffnesses_[i];
    stiffness_(i + 3, i + 3) = rotational_stiffnesses_[i];
  }
  for (int i = 0; i < 6; i++) {
    damping_(i, i) = 2. * sqrt(stiffness_(i, i));
  }
  for (int i = 0; i < 7; i++){
    nullspace_stiffness(i,i) = ns_p[i];
    nullspace_damping(i,i) = 2. * sqrt(nullspace_stiffness(i,i));
  }

  myFile.open("log.txt");
  iter = 1;
}

void CartesianImpedanceFeedbackController::parse_sensor_data(const franka::RobotState &robot_state) {
  SensorDataManagerReadStatus sensor_msg_status = sensor_data_manager_->readFeedbackControllerSensorMessage(cartesian_impedance_sensor_msg_);
  if (sensor_msg_status == SensorDataManagerReadStatus::SUCCESS) {
    for (int i = 0; i < 3; i++) {
      stiffness_(i, i) = cartesian_impedance_sensor_msg_.translational_stiffnesses(i);
      stiffness_(i + 3, i + 3) = cartesian_impedance_sensor_msg_.rotational_stiffnesses(i);
    }
    for (int i = 0; i < 6; i++) {
      damping_(i, i) = 2. * sqrt(stiffness_(i, i));
    }
  }
}

void CartesianImpedanceFeedbackController::get_next_step(const franka::RobotState &robot_state,
                                                         TrajectoryGenerator *traj_generator) {
  std::array<double, 7> coriolis_array = model_->coriolis(robot_state);
  std::array<double, 42> jacobian_array = model_->zeroJacobian(franka::Frame::kEndEffector, robot_state);
  std::array<double, 49> mass_array = model_->mass(robot_state);

  double m_ee = robot_state.m_ee;
  std::array<double, 9> I_ee = robot_state.I_ee;
  std::array<double, 3> F_x_Cee = robot_state.F_x_Cee;
  double m_load = robot_state.m_load;
  std::array<double, 9> I_load = robot_state.I_load;

  // std::cout << "m_ee = " << m_ee << std::endl;
  // std::cout << "I_ee = ";
  // for (int i = 0; i < 9; i++){
  //   std::cout << I_ee[i] << " ";
  // }
  // std::cout << std::endl << "F_x_Cee = ";
  // for (int i = 0; i < 3; i++){
  //   std::cout << F_x_Cee[i] << " ";
  // }
  // std::cout << std::endl << "m_load = " << m_load << std::endl << "I_load =";
  // for (int i = 0; i < 9; i++){
  //   std::cout << I_load[i] << " ";
  // }
  // std::cout << std::endl;

  // convert to Eigen
  Eigen::Map<const Eigen::Matrix<double, 7, 1> > coriolis(coriolis_array.data());
  Eigen::Map<const Eigen::Matrix<double, 6, 7> > jacobian(jacobian_array.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 1> > dq(robot_state.dq.data());
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.linear());
  Eigen::Map<const Eigen::Matrix<double, 7, 1> > q(robot_state.q.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 7> > mass(mass_array.data());
  mm_inv = mass.inverse();
  m_eef_inv = jacobian * mm_inv * jacobian.transpose();
  m_eef = m_eef_inv.inverse();
  j_eef_inv = m_eef * jacobian * mm_inv;
  // std::cout << "m_eef = " << m_eef << std::endl;
  std::array<double, 6> actual_force_array = robot_state.O_F_ext_hat_K;
  Eigen::Map<Eigen::VectorXd> actual_force(actual_force_array.data(), 6);

  PoseTrajectoryGenerator* pose_trajectory_generator = dynamic_cast<PoseTrajectoryGenerator*>(traj_generator);

  if (pose_trajectory_generator == nullptr) {
    throw std::bad_cast();
  }

  Eigen::Vector3d position_d(pose_trajectory_generator->get_desired_position());
  Eigen::Quaterniond orientation_d(pose_trajectory_generator->get_desired_orientation());

  // compute error to desired equilibrium pose
  // position error
  Eigen::Matrix<double, 6, 1> error;
  error.head(3) << position - position_d;

  // orientation error
  // "difference" quaternion
  if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  Eigen::Quaterniond error_quaternion(orientation * orientation_d.inverse());
  // convert to axis angle
  Eigen::AngleAxisd error_quaternion_angle_axis(error_quaternion);
  // compute "orientation error"
  error.tail(3) << error_quaternion_angle_axis.axis() * error_quaternion_angle_axis.angle();
  //error.tail(3) << error_quaternion.x(),error_quaternion.y(), error_quaternion.z();
  //error.tail(3) << -transform.rotation() * error.tail(3); 

  // compute kinematic pseudoinverse
  Eigen::MatrixXd jacobian_transpose_pinv;
  pseudoInverse(jacobian.transpose(), &jacobian_transpose_pinv);
  
  // compute control
  Eigen::VectorXd tau_task(7), tau_d(7), tau_nullspace(7);

  tau_ee = Eigen::VectorXd(7);
  tau_ee = -stiffness_ * error - damping_ * (jacobian * dq);

  // Spring damper system with damping ratio=1
  tau_task << jacobian.transpose() * tau_ee;

  // tau_task << mass * jacobian.transpose() * (-stiffness_ * error - damping_ * (jacobian * dq));

  // Torque for joint impedance control with respect to a desired configuration and projected in the null-space of the robot's Jacobian, so it should not affect the Cartesian motion of the robot's end-effector.
  tau_nullspace << (Eigen::MatrixXd::Identity(7,7) - jacobian.transpose() * jacobian_transpose_pinv) *
                      (nullspace_stiffness * (q_nullspace - q) - nullspace_damping * dq);
                      // (- nullspace_damping * dq);

  tau_d << tau_task + coriolis;

  // std::cout << "Iteration: " << iter << "\n";
  // std::cout << "tau_task: " << tau_task << "\n";
 // std::cout << "Jacobian: " << jacobian << "\n";

  myFile << tau_task(0) << " " << tau_task(1) << " " << tau_task(2) << " " << tau_task(3) << " " << tau_task(4)
            << " " << tau_task(5) << " " << tau_task(6) << " ";
  // myFile << actual_force(0) << " " << actual_force(1) << " " << actual_force(2) << " " << actual_force(3) << " " << actual_force(4)
  //           << " " << actual_force(5) << "\n";
  myFile << tau_ee(0) << " " << tau_ee(1) << " " << tau_ee(2) << " " << tau_ee(3) << " " << tau_ee(4)
            << " " << tau_ee(5) << "\n";
  iter++;

  Eigen::VectorXd::Map(&tau_d_array_[0], 7) = tau_d;
}

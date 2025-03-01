/*
 * robotStateRT.h
 *
 * Copyright 2015 Thomas Timm Andersen
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef ROBOT_STATE_RT_H_
#define ROBOT_STATE_RT_H_

#include <array>
#include <condition_variable>
#include <inttypes.h>
#include <mutex>
#include <netinet/in.h>
#include <stdlib.h>
#include <string.h>

class RobotStateRT
{
private:
  double version_; // protocol version

  double time_; // Time elapsed since the controller was started
  std::array<double, 6> q_target_; // Target joint positions
  std::array<double, 6> qd_target_; // Target joint velocities
  std::array<double, 6> qdd_target_; // Target joint accelerations
  std::array<double, 6> i_target_; // Target joint currents
  std::array<double, 6> m_target_; // Target joint moments (torques)
  std::array<double, 6> q_actual_; // Actual joint positions
  std::array<double, 6> qd_actual_; // Actual joint velocities
  std::array<double, 6> i_actual_; // Actual joint currents
  std::array<double, 6> i_control_; // Joint control currents
  std::array<double, 6> tool_vector_actual_; // Actual Cartesian coordinates of the tool:
                                             // (x,y,z,rx,ry,rz), where rx, ry and rz is a
                                             // rotation vector representation of the tool
                                             // orientation
  std::array<double, 6> tcp_speed_actual_; // Actual speed of the tool given
                                           // in Cartesian coordinates
  std::array<double, 6> tcp_force_; // Generalised forces in the TC
  std::array<double, 6> tool_vector_target_; // Target Cartesian coordinates of the tool:
                                             // (x,y,z,rx,ry,rz), where rx, ry and rz is a
                                             // rotation vector representation of the tool
                                             // orientation
  std::array<double, 6> tcp_speed_target_; // Target speed of the tool given
                                           // in Cartesian coordinates
  std::array<bool, 64> digital_input_bits_; // Current state of the digital inputs. NOTE: these
                                            // are bits encoded as int64_t, e.g. a value of 5
                                            // corresponds to bit 0 and bit 2 set high
  std::array<double, 6> motor_temperatures_; // Temperature of each joint in degrees celsius
  double controller_timer_; // Controller realtime thread execution time
  double robot_mode_; // Robot mode
  std::array<double, 6> joint_modes_; // Joint control modes
  double safety_mode_; // Safety mode
  std::array<double, 3> tool_accelerometer_values_; // Tool x,y and z accelerometer values
                                                    // (software version 1.7)
  double speed_scaling_; // Speed scaling of the trajectory limiter
  double linear_momentum_norm_; // Norm of Cartesian linear momentum
  double v_main_; // Masterboard: Main voltage
  double v_robot_; // Matorborad: Robot voltage (48V)
  double i_robot_; // Masterboard: Robot current
  std::array<double, 6> v_actual_; // Actual joint voltages

  std::mutex val_lock_; // Locks the variables while unpack parses data;

  std::condition_variable * pMsg_cond_; // Signals that new vars are available
  bool data_published_; // to avoid spurious wakes
  bool controller_updated_; // to avoid spurious wakes
public:
  RobotStateRT(std::condition_variable & msg_cond);
  ~RobotStateRT();
  double getVersion();
  double getTime();
  std::array<double, 6> getQTarget();
  void getQTarget(double * data);
  std::array<double, 6> getQdTarget();
  void getQdTarget(double * data);
  std::array<double, 6> getQddTarget();
  void getQddTarget(double * data);
  std::array<double, 6> getITarget();
  void getITarget(double * data);
  std::array<double, 6> getMTarget();
  void getMTarget(double * data);
  std::array<double, 6> getQActual();
  void getQActual(double * data);
  std::array<double, 6> getQdActual();
  void getQdActual(double * data);
  std::array<double, 6> getIActual();
  void getIActual(double * data);
  std::array<double, 6> getIControl();
  void getIControl(double * data);
  std::array<double, 6> getToolVectorActual();
  void getToolVectorActual(double * data);
  std::array<double, 6> getTcpSpeedActual();
  void getTcpSpeedActual(double * data);
  std::array<double, 6> getTcpForce();
  void getTcpForce(double * data);
  std::array<double, 6> getToolVectorTarget();
  void getToolVectorTarget(double * data);
  std::array<double, 6> getTcpSpeedTarget();
  void getTcpSpeedTarget(double * data);
  std::array<bool, 64> getDigitalInputBits();
  void getDigitalInputBits(bool * data);
  std::array<double, 6> getMotorTemperatures();
  void getMotorTemperatures(double * data);
  double getControllerTimer();
  double getRobotMode();
  std::array<double, 6> getJointModes();
  void getJointModes(double * data);
  double getSafety_mode();
  std::array<double, 3> getToolAccelerometerValues();
  void getToolAccelerometerValues(double * data);
  double getSpeedScaling();
  double getLinearMomentumNorm();
  double getVMain();
  double getVRobot();
  double getIRobot();

  void setVersion(double ver);

  void setDataPublished();
  bool getDataPublished();
  bool getControllerUpdated();
  void setControllerUpdated();
  std::array<double, 6> getVActual();
  void getVActual(double * data);
  void unpack(uint8_t * buf);
};

#endif /* ROBOT_STATE_RT_H_ */

/*
 * robotStateRT.cpp
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

#include "ur_modern_driver/robot_state_RT.h"

namespace
{

double ntohd(uint64_t nf)
{
  double x;
  nf = be64toh(nf);
  memcpy(&x, &nf, sizeof(x));
  return x;
}

std::array<bool, 64> unpackDigitalInputBits(int64_t data)
{
  std::array<bool, 64> ret;
  for(int i = 0; i < 64; i++)
  {
    ret[i] = (data & (1 << i)) >> i;
  }
  return ret;
}

template<int Size>
std::array<double, Size> unpackVector(uint8_t * buf, int start_index)
{
  uint64_t q;
  std::array<double, Size> ret;
  for(int i = 0; i < Size; i++)
  {
    memcpy(&q, &buf[start_index + i * sizeof(q)], sizeof(q));
    ret[i] = ntohd(q);
  }
  return ret;
}
} // namespace

RobotStateRT::RobotStateRT(std::condition_variable & msg_cond)
{
  version_ = 0.0;
  time_ = 0.0;
  q_target_.fill(0.0);
  qd_target_.fill(0.0);
  qdd_target_.fill(0.0);
  i_target_.fill(0.0);
  m_target_.fill(0.0);
  q_actual_.fill(0.0);
  qd_actual_.fill(0.0);
  i_actual_.fill(0.0);
  i_control_.fill(0.0);
  tool_vector_actual_.fill(0.0);
  tcp_speed_actual_.fill(0.0);
  tcp_force_.fill(0.0);
  tool_vector_target_.fill(0.0);
  tcp_speed_target_.fill(0.0);
  digital_input_bits_.fill(false);
  motor_temperatures_.fill(0.0);
  controller_timer_ = 0.0;
  robot_mode_ = 0.0;
  joint_modes_.fill(0.0);
  safety_mode_ = 0.0;
  tool_accelerometer_values_.fill(0.0);
  speed_scaling_ = 0.0;
  linear_momentum_norm_ = 0.0;
  v_main_ = 0.0;
  v_robot_ = 0.0;
  i_robot_ = 0.0;
  v_actual_.fill(0.0);
  data_published_ = false;
  controller_updated_ = false;
  pMsg_cond_ = &msg_cond;
}

RobotStateRT::~RobotStateRT()
{
  /* Make sure nobody is waiting after this thread is destroyed */
  data_published_ = true;
  controller_updated_ = true;
  pMsg_cond_->notify_all();
}

void RobotStateRT::setDataPublished()
{
  data_published_ = false;
}
bool RobotStateRT::getDataPublished()
{
  return data_published_;
}

void RobotStateRT::setControllerUpdated()
{
  controller_updated_ = false;
}
bool RobotStateRT::getControllerUpdated()
{
  return controller_updated_;
}

void RobotStateRT::setVersion(double ver)
{
  val_lock_.lock();
  version_ = ver;
  val_lock_.unlock();
}

double RobotStateRT::getVersion()
{
  double ret;
  val_lock_.lock();
  ret = version_;
  val_lock_.unlock();
  return ret;
}
double RobotStateRT::getTime()
{
  double ret;
  val_lock_.lock();
  ret = time_;
  val_lock_.unlock();
  return ret;
}
std::array<double, 6> RobotStateRT::getQTarget()
{
  std::array<double, 6> ret;
  val_lock_.lock();
  ret = q_target_;
  val_lock_.unlock();
  return ret;
}
void RobotStateRT::getQTarget(double * data)
{
  val_lock_.lock();
  memcpy(data, q_target_.data(), 6 * sizeof(double));
  val_lock_.unlock();
}
void RobotStateRT::getQdTarget(double * data)
{
  val_lock_.lock();
  memcpy(data, qd_target_.data(), 6 * sizeof(double));
  val_lock_.unlock();
}
std::array<double, 6> RobotStateRT::getQdTarget()
{
  std::array<double, 6> ret;
  val_lock_.lock();
  ret = qd_target_;
  val_lock_.unlock();
  return ret;
}
void RobotStateRT::getQddTarget(double * data)
{
  val_lock_.lock();
  memcpy(data, qdd_target_.data(), 6 * sizeof(double));
  val_lock_.unlock();
}
std::array<double, 6> RobotStateRT::getQddTarget()
{
  std::array<double, 6> ret;
  val_lock_.lock();
  ret = qdd_target_;
  val_lock_.unlock();
  return ret;
}
void RobotStateRT::getITarget(double * data)
{
  val_lock_.lock();
  memcpy(data, i_target_.data(), 6 * sizeof(double));
  val_lock_.unlock();
}
std::array<double, 6> RobotStateRT::getITarget()
{
  std::array<double, 6> ret;
  val_lock_.lock();
  ret = i_target_;
  val_lock_.unlock();
  return ret;
}
void RobotStateRT::getMTarget(double * data)
{
  val_lock_.lock();
  memcpy(data, m_target_.data(), 6 * sizeof(double));
  val_lock_.unlock();
}
std::array<double, 6> RobotStateRT::getMTarget()
{
  std::array<double, 6> ret;
  val_lock_.lock();
  ret = m_target_;
  val_lock_.unlock();
  return ret;
}
void RobotStateRT::getQActual(double * data)
{
  val_lock_.lock();
  memcpy(data, q_actual_.data(), 6 * sizeof(double));
  val_lock_.unlock();
}
std::array<double, 6> RobotStateRT::getQActual()
{
  std::array<double, 6> ret;
  val_lock_.lock();
  ret = q_actual_;
  val_lock_.unlock();
  return ret;
}
void RobotStateRT::getQdActual(double * data)
{
  val_lock_.lock();
  memcpy(data, qd_actual_.data(), 6 * sizeof(double));
  val_lock_.unlock();
}
std::array<double, 6> RobotStateRT::getQdActual()
{
  std::array<double, 6> ret;
  val_lock_.lock();
  ret = qd_actual_;
  val_lock_.unlock();
  return ret;
}
void RobotStateRT::getIActual(double * data)
{
  val_lock_.lock();
  memcpy(data, i_actual_.data(), 6 * sizeof(double));
  val_lock_.unlock();
}
std::array<double, 6> RobotStateRT::getIActual()
{
  std::array<double, 6> ret;
  val_lock_.lock();
  ret = i_actual_;
  val_lock_.unlock();
  return ret;
}
void RobotStateRT::getIControl(double * data)
{
  val_lock_.lock();
  memcpy(data, i_control_.data(), 6 * sizeof(double));
  val_lock_.unlock();
}
std::array<double, 6> RobotStateRT::getIControl()
{
  std::array<double, 6> ret;
  val_lock_.lock();
  ret = i_control_;
  val_lock_.unlock();
  return ret;
}
void RobotStateRT::getToolVectorActual(double * data)
{
  val_lock_.lock();
  memcpy(data, tool_vector_actual_.data(), 6 * sizeof(double));
  val_lock_.unlock();
}
std::array<double, 6> RobotStateRT::getToolVectorActual()
{
  std::array<double, 6> ret;
  val_lock_.lock();
  ret = tool_vector_actual_;
  val_lock_.unlock();
  return ret;
}
void RobotStateRT::getTcpSpeedActual(double * data)
{
  val_lock_.lock();
  memcpy(data, tcp_speed_actual_.data(), 6 * sizeof(double));
  val_lock_.unlock();
}
std::array<double, 6> RobotStateRT::getTcpSpeedActual()
{
  std::array<double, 6> ret;
  val_lock_.lock();
  ret = tcp_speed_actual_;
  val_lock_.unlock();
  return ret;
}
void RobotStateRT::getTcpForce(double * data)
{
  val_lock_.lock();
  memcpy(data, tcp_force_.data(), 6 * sizeof(double));
  val_lock_.unlock();
}
std::array<double, 6> RobotStateRT::getTcpForce()
{
  std::array<double, 6> ret;
  val_lock_.lock();
  ret = tcp_force_;
  val_lock_.unlock();
  return ret;
}
void RobotStateRT::getToolVectorTarget(double * data)
{
  val_lock_.lock();
  memcpy(data, tool_vector_target_.data(), 6 * sizeof(double));
  val_lock_.unlock();
}
std::array<double, 6> RobotStateRT::getToolVectorTarget()
{
  std::array<double, 6> ret;
  val_lock_.lock();
  ret = tool_vector_target_;
  val_lock_.unlock();
  return ret;
}
void RobotStateRT::getTcpSpeedTarget(double * data)
{
  val_lock_.lock();
  memcpy(data, tcp_speed_target_.data(), 6 * sizeof(double));
  val_lock_.unlock();
}
std::array<double, 6> RobotStateRT::getTcpSpeedTarget()
{
  std::array<double, 6> ret;
  val_lock_.lock();
  ret = tcp_speed_target_;
  val_lock_.unlock();
  return ret;
}
std::array<bool, 64> RobotStateRT::getDigitalInputBits()
{
  std::array<bool, 64> ret;
  val_lock_.lock();
  ret = digital_input_bits_;
  val_lock_.unlock();
  return ret;
}
void RobotStateRT::getDigitalInputBits(bool * data)
{
  val_lock_.lock();
  memcpy(data, digital_input_bits_.data(), 64 * sizeof(bool));
  val_lock_.unlock();
}
void RobotStateRT::getMotorTemperatures(double * data)
{
  val_lock_.lock();
  memcpy(data, motor_temperatures_.data(), 6 * sizeof(double));
  val_lock_.unlock();
}
std::array<double, 6> RobotStateRT::getMotorTemperatures()
{
  std::array<double, 6> ret;
  val_lock_.lock();
  ret = motor_temperatures_;
  val_lock_.unlock();
  return ret;
}
double RobotStateRT::getControllerTimer()
{
  double ret;
  val_lock_.lock();
  ret = controller_timer_;
  val_lock_.unlock();
  return ret;
}
double RobotStateRT::getRobotMode()
{
  double ret;
  val_lock_.lock();
  ret = robot_mode_;
  val_lock_.unlock();
  return ret;
}
void RobotStateRT::getJointModes(double * data)
{
  val_lock_.lock();
  memcpy(data, joint_modes_.data(), 6 * sizeof(double));
  val_lock_.unlock();
}
std::array<double, 6> RobotStateRT::getJointModes()
{
  std::array<double, 6> ret;
  val_lock_.lock();
  ret = joint_modes_;
  val_lock_.unlock();
  return ret;
}
double RobotStateRT::getSafety_mode()
{
  double ret;
  val_lock_.lock();
  ret = safety_mode_;
  val_lock_.unlock();
  return ret;
}
void RobotStateRT::getToolAccelerometerValues(double * data)
{
  val_lock_.lock();
  memcpy(data, tool_accelerometer_values_.data(), 3 * sizeof(double));
  val_lock_.unlock();
}
std::array<double, 3> RobotStateRT::getToolAccelerometerValues()
{
  std::array<double, 3> ret;
  val_lock_.lock();
  ret = tool_accelerometer_values_;
  val_lock_.unlock();
  return ret;
}
double RobotStateRT::getSpeedScaling()
{
  double ret;
  val_lock_.lock();
  ret = speed_scaling_;
  val_lock_.unlock();
  return ret;
}
double RobotStateRT::getLinearMomentumNorm()
{
  double ret;
  val_lock_.lock();
  ret = linear_momentum_norm_;
  val_lock_.unlock();
  return ret;
}
double RobotStateRT::getVMain()
{
  double ret;
  val_lock_.lock();
  ret = v_main_;
  val_lock_.unlock();
  return ret;
}
double RobotStateRT::getVRobot()
{
  double ret;
  val_lock_.lock();
  ret = v_robot_;
  val_lock_.unlock();
  return ret;
}
double RobotStateRT::getIRobot()
{
  double ret;
  val_lock_.lock();
  ret = i_robot_;
  val_lock_.unlock();
  return ret;
}
void RobotStateRT::getVActual(double * data)
{
  val_lock_.lock();
  memcpy(data, v_actual_.data(), 6 * sizeof(double));
  val_lock_.unlock();
}
std::array<double, 6> RobotStateRT::getVActual()
{
  std::array<double, 6> ret;
  val_lock_.lock();
  ret = v_actual_;
  val_lock_.unlock();
  return ret;
}
void RobotStateRT::unpack(uint8_t * buf)
{
  int64_t digital_input_bits;
  uint64_t unpack_to;
  uint16_t offset = 0;
  val_lock_.lock();
  int len;
  memcpy(&len, &buf[offset], sizeof(len));

  offset += sizeof(len);
  len = ntohl(len);

  // Check the correct message length is received
  bool len_good = true;
  if(version_ >= 1.6 && version_ < 1.7)
  { // v1.6
    if(len != 756) len_good = false;
  }
  else if(version_ >= 1.7 && version_ < 1.8)
  { // v1.7
    if(len != 764) len_good = false;
  }
  else if(version_ >= 1.8 && version_ < 1.9)
  { // v1.8
    if(len != 812) len_good = false;
  }
  else if(version_ >= 3.0 && version_ < 3.2)
  { // v3.0 & v3.1
    if(len != 1044) len_good = false;
  }
  else if(version_ >= 3.2 && version_ < 3.3)
  { // v3.2
    if(len != 1060) len_good = false;
  }

  if(!len_good)
  {
    printf("Wrong length of message on RT interface: %i\n", len);
    val_lock_.unlock();
    return;
  }

  memcpy(&unpack_to, &buf[offset], sizeof(unpack_to));
  time_ = ntohd(unpack_to);
  offset += sizeof(double);
  q_target_ = unpackVector<6>(buf, offset);
  offset += sizeof(double) * 6;
  qd_target_ = unpackVector<6>(buf, offset);
  offset += sizeof(double) * 6;
  qdd_target_ = unpackVector<6>(buf, offset);
  offset += sizeof(double) * 6;
  i_target_ = unpackVector<6>(buf, offset);
  offset += sizeof(double) * 6;
  m_target_ = unpackVector<6>(buf, offset);
  offset += sizeof(double) * 6;
  q_actual_ = unpackVector<6>(buf, offset);
  offset += sizeof(double) * 6;
  qd_actual_ = unpackVector<6>(buf, offset);
  offset += sizeof(double) * 6;
  i_actual_ = unpackVector<6>(buf, offset);
  offset += sizeof(double) * 6;
  if(version_ <= 1.9)
  {
    if(version_ > 1.6) tool_accelerometer_values_ = unpackVector<3>(buf, offset);
    offset += sizeof(double) * (3 + 15);
    tcp_force_ = unpackVector<6>(buf, offset);
    offset += sizeof(double) * 6;
    tool_vector_actual_ = unpackVector<6>(buf, offset);
    offset += sizeof(double) * 6;
    tcp_speed_actual_ = unpackVector<6>(buf, offset);
  }
  else
  {
    i_control_ = unpackVector<6>(buf, offset);
    offset += sizeof(double) * 6;
    tool_vector_actual_ = unpackVector<6>(buf, offset);
    offset += sizeof(double) * 6;
    tcp_speed_actual_ = unpackVector<6>(buf, offset);
    offset += sizeof(double) * 6;
    tcp_force_ = unpackVector<6>(buf, offset);
    offset += sizeof(double) * 6;
    tool_vector_target_ = unpackVector<6>(buf, offset);
    offset += sizeof(double) * 6;
    tcp_speed_target_ = unpackVector<6>(buf, offset);
  }
  offset += sizeof(double) * 6;

  memcpy(&digital_input_bits, &buf[offset], sizeof(digital_input_bits));
  digital_input_bits_ = unpackDigitalInputBits(be64toh(digital_input_bits));
  offset += sizeof(double);
  motor_temperatures_ = unpackVector<6>(buf, offset);
  offset += sizeof(double) * 6;
  memcpy(&unpack_to, &buf[offset], sizeof(unpack_to));
  controller_timer_ = ntohd(unpack_to);
  if(version_ > 1.6)
  {
    offset += sizeof(double) * 2;
    memcpy(&unpack_to, &buf[offset], sizeof(unpack_to));
    robot_mode_ = ntohd(unpack_to);
    if(version_ > 1.7)
    {
      offset += sizeof(double);
      joint_modes_ = unpackVector<6>(buf, offset);
    }
  }
  if(version_ > 1.8)
  {
    offset += sizeof(double) * 6;
    memcpy(&unpack_to, &buf[offset], sizeof(unpack_to));
    safety_mode_ = ntohd(unpack_to);
    offset += sizeof(double);
    tool_accelerometer_values_ = unpackVector<3>(buf, offset);
    offset += sizeof(double) * 3;
    memcpy(&unpack_to, &buf[offset], sizeof(unpack_to));
    speed_scaling_ = ntohd(unpack_to);
    offset += sizeof(double);
    memcpy(&unpack_to, &buf[offset], sizeof(unpack_to));
    linear_momentum_norm_ = ntohd(unpack_to);
    offset += sizeof(double);
    memcpy(&unpack_to, &buf[offset], sizeof(unpack_to));
    v_main_ = ntohd(unpack_to);
    offset += sizeof(double);
    memcpy(&unpack_to, &buf[offset], sizeof(unpack_to));
    v_robot_ = ntohd(unpack_to);
    offset += sizeof(double);
    memcpy(&unpack_to, &buf[offset], sizeof(unpack_to));
    i_robot_ = ntohd(unpack_to);
    offset += sizeof(double);
    v_actual_ = unpackVector<6>(buf, offset);
  }
  val_lock_.unlock();
  controller_updated_ = true;
  data_published_ = true;
  pMsg_cond_->notify_all();
}

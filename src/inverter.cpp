#include "inverter.hpp"
#include "car.h"
#include "parameters.hpp"

Inverter::Inverter(bool (*timer_mc_kick)(), bool (*timer_current_limit)(),
                   bool (*timer_motor_controller_send)(), bool spin_direction,
                   canMan *can, canMan *daq_can, can_obj_car_h_t *dbc,
                   float over_power_decay_factor)
{
  this->timer_mc_kick = timer_mc_kick;
  this->timer_current_limit = timer_current_limit;
  this->timer_motor_controller_send = timer_motor_controller_send;

  // this->timer_overpower_decay = timer_overpower_decay;

  this->spin_forward = spin_direction;

  this->over_power_decay_factor = over_power_decay_factor;

  this->can = can;
  this->daq_can = daq_can;
  this->dbc = dbc;

  this->ping();
}

void Inverter::set_current_limits(uint16_t charge_limit,
                                  uint16_t discharge_limit)
{
  encode_can_0x202_BMS_Max_Charge_Current(dbc, charge_limit);
  encode_can_0x202_BMS_Max_Discharge_Current(dbc, discharge_limit);

  can_message out_msg;
  out_msg.id = CAN_ID_BMS_CURRENT_LIMIT;
  out_msg.length =
      pack_message(dbc, CAN_ID_BMS_CURRENT_LIMIT, &out_msg.buf.val);

  can->send_controller_message(out_msg);
  daq_can->send_controller_message(out_msg);
}

void Inverter::update_bus_current(uint64_t msg_in, uint8_t length)
{
  unpack_message(dbc, CAN_ID_M166_CURRENT_INFO, msg_in, length, 0);

  decode_can_0x0a6_INV_DC_Bus_Current(dbc, &bus_current);
}

void Inverter::update_bus_voltage(uint64_t msg_in, uint8_t length)
{
  unpack_message(dbc, CAN_ID_M167_VOLTAGE_INFO, msg_in, length, 0);

  decode_can_0x0a7_INV_DC_Bus_Voltage(dbc, &bus_voltage);
}

void Inverter::update_motor_feedback(uint64_t msg_in, uint8_t length)
{
  unpack_message(dbc, CAN_ID_M165_MOTOR_POSITION_INFO, msg_in, length, 0);

  decode_can_0x0a5_INV_Motor_Speed(dbc, &motor_rpm);
}

void Inverter::calculate_motor_distance_M(uint32_t time_msec)
{
  uint32_t time_elaped_msec = time_msec - time_last_msec;

  double velocity_Msec =
      (double(motor_rpm) / 60 / GEAR_RATIO) * WHEEL_CIRCUMFRANCE_M;

  distance_M += (double(time_elaped_msec) / 1000) * velocity_Msec;

  time_last_msec = time_msec;
}

void Inverter::ping()
{
  encode_can_0x0c0_VCU_INV_Torque_Command(dbc, 0.0);
  encode_can_0x0c0_VCU_INV_Torque_Limit_Command(dbc, 0.0);
  encode_can_0x0c0_VCU_INV_Speed_Command(dbc, 0);
  encode_can_0x0c0_VCU_INV_Speed_Mode_Enable(dbc, 0);
  encode_can_0x0c0_VCU_INV_Direction_Command(dbc, spin_forward);
  encode_can_0x0c0_VCU_INV_Inverter_Discharge(dbc, inverter_discharge);
  encode_can_0x0c0_VCU_INV_Inverter_Enable(dbc, inverter_enable);

  can_message out_msg;
  out_msg.id = CAN_ID_M192_COMMAND_MESSAGE;
  out_msg.length =
      pack_message(dbc, CAN_ID_M192_COMMAND_MESSAGE, &out_msg.buf.val);

  can->send_controller_message(out_msg);
  daq_can->send_controller_message(out_msg);
}

void Inverter::send_clear_faults()
{
  encode_can_0x0c1_VCU_INV_Parameter_Address(dbc, 20);
  encode_can_0x0c1_VCU_INV_Parameter_RW_Command(dbc, 1);
  encode_can_0x0c1_VCU_INV_Parameter_Data(dbc, 0);

  can_message out_msg;
  out_msg.length =
      pack_message(dbc, CAN_ID_M192_COMMAND_MESSAGE, &out_msg.buf.val);

  can->send_controller_message(out_msg);
  daq_can->send_controller_message(out_msg);
}

void Inverter::command_torque(double torque_request)
{
  double torque_target = torque_request;

  // EV. 1.4.4 A violation is defined as using more than the specified maximum power OR exceeding
  // the maximum voltage EITHER:
  // a. Continuously for 100 ms or more
  // b. After a moving average over 500 ms is applied

  // electronic rev limiter

  // https://www.desmos.com/calculator/j8kydktjry

  // double P = bus_current * bus_voltage;
  // double power_over_w = std::max(0.0, P - (power_limit_kw * 1000));

  // if (power_over_w > 1e-6)
  // {
  //   double torque_over_nm = power_over_w / std::max(1e-6, (motor_rpm / 60.0) * 2.0 * M_PI);
  //   // torque_D += (torque_over_nm * torque_kd) * dt_s;
  //   torque_I += torque_ki * torque_over_nm / dt_s;
  //   torque_target -= torque_kp * torque_over_nm + torque_I + torque_D;
  // }
  // else
  // {
  //   torque_I *= 0.98;
  //   // torque_target -= torque_kp * torque_over_nm + torque_I;
  // }
  // speed limiter
  // if (angular_vel_over_rad_s > 0)
  // {
  //   torque_over_nm += power_over_w / angular_vel_over_rad_s;
  // }
  // else
  // {
  //   speed_I *= 0.98;
  // }
  // double speed_err = speed_kp * angular_vel_over_rad_s + speed_I;
  // torque_target -= speed_err;

  // if (motor_rpm >= speed_limit && torque_target > 0)
  // {
  //   speed_I += speed_ki * ((motor_rpm - speed_limit) / speed_limit) * dt_s;
  //   torque_target *= 1 - (speed_kp * ((motor_rpm - speed_limit) / speed_limit) + speed_I);
  // }
  // else
  // {
  //   speed_I *= 0.98;
  // }

  encode_can_0x0c0_VCU_INV_Torque_Command(dbc, torque_target); // torque command to INV
  encode_can_0x0c0_VCU_INV_Torque_Limit_Command(dbc, torque_limit_nm);
  encode_can_0x0c0_VCU_INV_Speed_Command(dbc, 0);                       // unused
  encode_can_0x0c0_VCU_INV_Speed_Mode_Enable(dbc, 0);                   // unused
  encode_can_0x0c0_VCU_INV_Direction_Command(dbc, spin_forward);        // unused
  encode_can_0x0c0_VCU_INV_Inverter_Discharge(dbc, inverter_discharge); // unused
  encode_can_0x0c0_VCU_INV_Inverter_Enable(dbc, inverter_enable);       // unused

  can_message out_msg;
  out_msg.id = CAN_ID_M192_COMMAND_MESSAGE;
  out_msg.length =
      pack_message(dbc, CAN_ID_M192_COMMAND_MESSAGE, &out_msg.buf.val);

  can->send_controller_message(out_msg);
  daq_can->send_controller_message(out_msg);
}

void Inverter::command_speed(int16_t speed_request)
{
  encode_can_0x0c0_VCU_INV_Torque_Command(dbc, 0.0);
  encode_can_0x0c0_VCU_INV_Torque_Limit_Command(dbc, torque_limit_nm);
  encode_can_0x0c0_VCU_INV_Speed_Command(dbc, speed_request);
  encode_can_0x0c0_VCU_INV_Speed_Mode_Enable(dbc, speed_mode);
  encode_can_0x0c0_VCU_INV_Direction_Command(dbc, spin_forward);
  encode_can_0x0c0_VCU_INV_Inverter_Discharge(dbc, inverter_discharge);
  encode_can_0x0c0_VCU_INV_Inverter_Enable(dbc, inverter_enable);

  can_message out_msg;
  out_msg.id = CAN_ID_M192_COMMAND_MESSAGE;
  out_msg.length =
      pack_message(dbc, CAN_ID_M192_COMMAND_MESSAGE, &out_msg.buf.val);

  can->send_controller_message(out_msg);
  daq_can->send_controller_message(out_msg);
}

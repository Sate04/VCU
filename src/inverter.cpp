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

  this->timer_overpower_decay = timer_overpower_decay;

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

  // if we tune it well enough we can overcurrent without a hard limit ??

  double dt = 0.005; // 200 Hz
  static double I = 0.0;

  double P = bus_current * bus_voltage;
  double P_over = std::max(0.0, P - 79000.0);
  double w = std::max(1e-6, (motor_rpm / 60.0) * 2.0 * M_PI / GEAR_RATIO);
  double T_over = P_over / w;
  if (P_over > 0)
  {
    I += t_ki * T_over * dt;
  }
  else
  {
    I *= 0.98;
  }
  double T_err = t_kp * T_over + I;
  torque_target -= T_err;

  // clamp again just in case
  torque_target = std::min(torque_target, torque_limit_nm);

  encode_can_0x0c0_VCU_INV_Torque_Command(dbc, torque_target); // torque command to INV

  encode_can_0x0c0_VCU_INV_Torque_Limit_Command(dbc, torque_limit_nm);  // unused
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

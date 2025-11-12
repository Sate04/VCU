#pragma once

// ksu-fw-common
#include <adc.hpp>
#include <can_tools.hpp>
#include <logger.hpp>

// CAN lib stuff
#include <car.h>
can_obj_car_h_t kms_can;

// Local
#include "accumulator.hpp"
#include "inverter.hpp"
#include "parameters.hpp"
#include "pedal_handeler.hpp"
#include "vcu.hpp"

#ifdef ARDUINO
#include <Arduino.h>

// TODO: Make the timer stuff into its own utility, maybe a part of the logger?
#include <Metro.h>

Metro timer_1s = Metro(1000, true);    // Used for VCU status message
Metro timer_2hz = Metro(500, true);    // Used for ACU and Precharge messages
Metro timer_10hz = Metro(100, true);   // Used for VCU pedals message
Metro timer_10hz_2 = Metro(100, true); // A naming convention so good I made 2
Metro timer_20hz = Metro(50, true);    // Used for inverter timeout
Metro timer_100hz = Metro(10, true);   // Used for inverter current limit
Metro timer_200hz = Metro(5, true);    // Used for inverter command message

Metro buzzer_timer = Metro(2215, false);

bool wrapped_1s()
{
  if (timer_1s.check())
  {
    return true;
    timer_1s.reset();
  }
  else
  {
    return false;
  }
}

bool wrapped_2hz()
{
  if (timer_2hz.check())
  {
    return true;
    timer_2hz.reset();
  }
  else
  {
    return false;
  }
}

bool wrapped_10hz()
{
  if (timer_10hz.check())
  {
    return true;
    timer_10hz.reset();
  }
  else
  {
    return false;
  }
}

bool wrapped_20hz()
{
  if (timer_20hz.check())
  {
    return true;
    timer_20hz.reset();
  }
  else
  {
    return false;
  }
}

bool wrapped_100hz()
{
  if (timer_100hz.check())
  {
    return true;
    timer_100hz.reset();
  }
  else
  {
    return false;
  }
}

bool wrapped_200hz()
{
  if (timer_200hz.check())
  {
    return true;
    timer_200hz.reset();
  }
  else
  {
    return false;
  }
}
#endif

//
//// Comms
// loggers
Logger consol(serial);
// FILE std_out_wrap;

// CAN controllers
canMan acc_can(TEENSY_CAN1, ACCUMULATOR_CAN_BAUD_RATE);
canMan inv_can(TEENSY_CAN2, INVERTER_CAN_BAUD_RATE);
canMan daq_can(TEENSY_CAN3, DAQ_CAN_BAUD_RATE);

// Pots
adc apps1(mcp, ADC_CS, ADC_ACCEL_1_CHANNEL, 0.980483996877);
adc apps2(mcp, ADC_CS, ADC_ACCEL_2_CHANNEL, 0.980483996877);
adc bse(mcp, ADC_CS, ADC_BSE_CHANNEL, 0.980483996877);

//
//// Critical components
Pedals pedals(MIN_BRAKE_PEDAL, START_BRAKE_PEDAL, END_BRAKE_PEDAL,
              MAX_BRAKE_PEDAL, MIN_APPS_PEDAL, START_ACCELERATOR_PEDAL_1,
              END_ACCELERATOR_PEDAL_1, START_ACCELERATOR_PEDAL_2,
              END_ACCELERATOR_PEDAL_2);

Inverter inverter(&wrapped_20hz, &wrapped_100hz, &wrapped_200hz, false,
                  &inv_can, &daq_can, &kms_can, -0.69314718056);

Accumulator accumulator(&kms_can, &acc_can, &wrapped_2hz);

VCU vcu(&pedals, &inverter, &accumulator, &kms_can, &acc_can, &inv_can,
        &daq_can, &wrapped_1s, &wrapped_10hz);

//
//// Gizmos
// Aditional ADC chanels
adc steering_angle(mcp, static_cast<uint8_t>(ADC_CS),
                   static_cast<uint8_t>(ADC_STEERING_CHANNEL));

// Voltage / Current sense lines
adc vsense_bspd(avr, BSPD_SENSE);
adc vsense_sdc(avr, VSENSE_SDC);
adc isense_sdc(avr, ISENSE_SDC);
adc vsense_12v(avr, VSENSE_GLV);
adc isense_12v(avr, ISENSE_GLV);
adc vsense_5v(avr, VSENSE_5V);

adc sense_lines[] = {vsense_bspd, vsense_sdc, isense_sdc,
                     vsense_12v, isense_12v, vsense_5v};

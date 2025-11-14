#pragma once

//
//// Some car values
#define MAX_TORQUE_LIMIT_NM 190
#define POWER_LIMIT_KW 80
#define SOFT_MOTOR_RPM_LIMIT 6000
#define MAX_MOTOR_RPM_LIMIT 7000

#define TRACTIVE_SYSTEM_MINIMUM_VOLTAGE 400
#define PRECHARGE_OK_STATE 2
#define MINIMUM_BRAKE_FOR_RTD 0.3

#define INVERTER_CHARGE_LIMIT 60
#define INVERTER_DISCHARGE_LIMIT 600

#define WHEEL_CIRCUMFRANCE_M 1.435608
#define GEAR_RATIO 4.18

//
//// Hardcoded pedal values
#define MIN_BRAKE_PEDAL 2000
#define START_BRAKE_PEDAL 2840
#define END_BRAKE_PEDAL 3341
#define MAX_BRAKE_PEDAL 4000

#define MIN_APPS_PEDAL 100

#define START_ACCELERATOR_PEDAL_1 2862
#define END_ACCELERATOR_PEDAL_1 3484

#define START_ACCELERATOR_PEDAL_2 585
#define END_ACCELERATOR_PEDAL_2 1816

//
//// Default CAN settings
#define ACCUMULATOR_CAN_BAUD_RATE 500000
#define INVERTER_CAN_BAUD_RATE 500000
#define DAQ_CAN_BAUD_RATE 1000000

//
//// Teensy pins
#define ADC_CS 10

#define WSFL 28 // Digital input pullup
#define WSFR 29 // Digital input pullup

#define BSPD_SENSE 16 // Analog signal
#define ISENSE_SDC 20 // Analog signal
#define ISENSE_GLV 38 // Analog signal
#define VSENSE_SDC 39 // Analog signal
#define VSENSE_5V 40  // Analog signal
#define VSENSE_GLV 41 // Analog signal
#define A9 27         // Analog signal
#define A10 26        // Analog signal

#define BUZZER 4   // Output
#define LOWSIDE1 5 // Output
#define LOWSIDE2 6 // Output

//
//// MCP pins
#define ADC_ACCEL_1_CHANNEL 0
#define ADC_ACCEL_2_CHANNEL 1
#define ADC_BSE_CHANNEL 2
#define ADC_STEERING_CHANNEL 3

//
//// Default SPI settings
#define DEFAULT_SPI_SPEED 1000000

//
//// GIT status message defs
// These values are provided by the python script ran by the lib_dep
// https://github.com/KSU-MS/pio-git-hash-gen
#ifndef AUTO_VERSION
#warning "AUTO_VERSION was not defined by the generator!"
#define AUTO_VERSION 0xdeadbeef
#endif

#ifndef FW_PROJECT_IS_DIRTY
#warning "FW_PROJECT_IS_DIRTY was not defined by the generator!"
#define FW_PROJECT_IS_DIRTY 1
#endif

#ifndef FW_PROJECT_IS_MAIN_OR_MASTER
#warning "FW_PROJECT_IS_MAIN_OR_MASTER was not defined by the generator!"
#define FW_PROJECT_IS_MAIN_OR_MASTER 0
#endif

#pragma once

/**
 * OULWare v1.7
 */

#if NOT_TARGET(__SAM3X8E__)
  #error "Oops! Select 'Arduino Due'"
#endif

#define BOARD_INFO_NAME "RADDS"

//
// EEPROM
//
#define SRAM_EEPROM_EMULATION
#define MARLIN_EEPROM_SIZE              0x2000  // 8KB

//
// Limit Switches
//
#define X_MIN_PIN                             54
#define X_MAX_PIN                             55
#define Y_MIN_PIN                             56
//#define Y_MAX_PIN                             57 // NEOPIXEL2_PIN
//#define Z_MIN_PIN                             58
#define Z_MAX_PIN                             58 // 59 = NEOPIXEL_PIN

//
// Steppers
//
// Motor1
#define Y_STEP_PIN                            38
#define Y_DIR_PIN                             36
#define Y_ENABLE_PIN                          40
#define Y_CS_PIN                              34

// Motor2
#define X_STEP_PIN                            28
#define X_DIR_PIN                             26
#define X_ENABLE_PIN                          30
#define X_CS_PIN                              32

// Motor3
#define Z_STEP_PIN                            23
#define Z_DIR_PIN                             29
#define Z_ENABLE_PIN                          25
#define Z_CS_PIN                              27

// Motor4
#define E0_STEP_PIN                           44
#define E0_DIR_PIN                            42
#define E0_ENABLE_PIN                         46
#define E0_CS_PIN                             48

// Motor5
#define E1_STEP_PIN                           33
#define E1_DIR_PIN                            31
#define E1_ENABLE_PIN                         35
#define E1_CS_PIN                             37

// Motor6
#define E2_STEP_PIN                           41
#define E2_DIR_PIN                            39
#define E2_ENABLE_PIN                         43
#define E2_CS_PIN                             45

// Axis 'A'
#define I_STEP_PIN   E2_STEP_PIN
#define I_DIR_PIN    E2_DIR_PIN
#define I_ENABLE_PIN E2_ENABLE_PIN
#define I_CS_PIN     E2_CS_PIN

// Axis 'B'
#define J_STEP_PIN   E1_STEP_PIN
#define J_DIR_PIN    E1_DIR_PIN
#define J_ENABLE_PIN E1_ENABLE_PIN
#define J_CS_PIN     E1_CS_PIN

//#define K_STEP_PIN   E2_STEP_PIN
//#define K_DIR_PIN    E2_DIR_PIN
//#define K_ENABLE_PIN E2_ENABLE_PIN
//#define K_CS_PIN     E2_CS_PIN

//
// Temperature Sensors
//
#define TEMP_0_PIN                             9  // Analog Input
#define TEMP_1_PIN                            10  // Analog Input
#define TEMP_BED_PIN                          11  // Analog Input

// SPI for MAX Thermocouple
#define TEMP_0_CS_PIN                         47

//
// Heaters / Fans
//
#define HEATER_0_PIN                           7
#define HEATER_1_PIN                           6
#define HEATER_2_PIN                           9
#if !HAS_CUTTER
  #define HEATER_BED_PIN                       8  // BED
#endif

#define FAN_PIN                               10
#define FAN1_PIN                              11
#define FAN2_PIN                              12
//#define FAN3_PIN                              13

#define CASE_LIGHT_PIN  13 //FAN3_PIN   = Top light
#define NEOPIXEL_PIN    59 //Z_MAX_PIN  = Bottom light
#define NEOPIXEL2_PIN   56 //Y_MAX_PIN  - Top light

// Not used but has to be defined
#define SDSS 4

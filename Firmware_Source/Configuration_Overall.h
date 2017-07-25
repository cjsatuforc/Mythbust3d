/**
 * Configuration_Overall.h
 * Here you can define all your custom settings and they will overwrite configurations in the main configuration files.
 */

#define CONFIGURATION_OVERALL


/***********************
 * Configuration_Basic *
 ***********************/
#define SERIAL_PORT 0
#define BAUDRATE 115200
//#define BLUETOOTH
#define BLUETOOTH_PORT 0
#define BLUETOOTH_BAUD 115200
#define STRING_CONFIG_H_AUTHOR "(none, default config)"
#define MACHINE_UUID "00000000-0000-0000-0000-000000000000"
#define KILL_METHOD 0
#define NO_TIMEOUTS 1000
//#define ADVANCED_OK
//#define HOST_KEEPALIVE_FEATURE
#define DEFAULT_KEEPALIVE_INTERVAL 2
#define MOTHERBOARD BOARD_ANET_V_10
#define MECHANISM MECH_CARTESIAN
//#define MECHANISM MECH_COREXY
//#define MECHANISM MECH_COREYX
//#define MECHANISM MECH_COREXZ
//#define MECHANISM MECH_COREZX
//#define MECHANISM MECH_COREYZ
//#define MECHANISM MECH_COREZY
//#define MECHANISM MECH_DELTA
//#define MECHANISM MECH_MORGAN_SCARA
//#define MECHANISM MECH_MAKERARM_SCARA
//#define MECHANISM MECH_MUVE3D
#define POWER_SUPPLY 0
//#define PS_DEFAULT_OFF
#define EXTRUDERS 1
#define DRIVER_EXTRUDERS 1

/*****************************
 * Configuration_Temperature *
 *****************************/
//#define TEMPERATURE_UNITS_SUPPORT
#define TEMP_SENSOR_0 1
#define TEMP_SENSOR_1 0
#define TEMP_SENSOR_2 0
#define TEMP_SENSOR_3 0
#define TEMP_SENSOR_BED 1
#define TEMP_SENSOR_CHAMBER 0
#define TEMP_SENSOR_COOLER 0
#define TEMP_SENSOR_AD595_OFFSET 0
#define TEMP_SENSOR_AD595_GAIN 1
#define DUMMY_THERMISTOR_998_VALUE 25
#define DUMMY_THERMISTOR_999_VALUE 25
//#define SHOW_TEMP_ADC_VALUES
//#define MAX_CONSECUTIVE_LOW_TEMPERATURE_ERROR_ALLOWED 0
//#define MILLISECONDS_PREHEAT_TIME 0
#define TEMP_RESIDENCY_TIME 10  // (seconds)
#define TEMP_HYSTERESIS 3       // (degC) range of +/- temperatures considered "close" to the target one
#define TEMP_WINDOW     1       // (degC) Window around target to start the residency timer x degC early.
#define TEMP_BED_RESIDENCY_TIME 0   // (seconds)
#define TEMP_BED_HYSTERESIS 3       // (degC) range of +/- temperatures considered "close" to the target one
#define TEMP_BED_WINDOW     1       // (degC) Window around target to start the residency timer x degC early.
#define TEMP_CHAMBER_RESIDENCY_TIME 0   // (seconds)
#define TEMP_CHAMBER_HYSTERESIS 3       // (degC) range of +/- temperatures considered "close" to the target one
#define TEMP_CHAMBER_WINDOW     1       // (degC) Window around target to start the residency timer x degC early.
#define TEMP_COOLER_RESIDENCY_TIME 0    // (seconds)
#define TEMP_COOLER_HYSTERESIS 1        // (degC) range of +/- temperatures considered "close" to the target one
#define TEMP_COOLER_WINDOW     1        // (degC) Window around target to start the residency timer x degC early.
#define HEATER_0_MAXTEMP 275
#define HEATER_1_MAXTEMP 275
#define HEATER_2_MAXTEMP 275
#define HEATER_3_MAXTEMP 275
#define BED_MAXTEMP 150
#define CHAMBER_MAXTEMP 150
#define COOLER_MAXTEMP 150
#define HEATER_0_MINTEMP 5
#define HEATER_1_MINTEMP 5
#define HEATER_2_MINTEMP 5
#define HEATER_3_MINTEMP 5
#define BED_MINTEMP 5
#define CHAMBER_MINTEMP 5
#define COOLER_MINTEMP 5
#define PREHEAT_1_TEMP_HOTEND 200
#define PREHEAT_1_TEMP_BED 60
#define PREHEAT_1_FAN_SPEED 60
#define PREHEAT_2_TEMP_HOTEND 250
#define PREHEAT_2_TEMP_BED 90
#define PREHEAT_2_FAN_SPEED 0
#define PREHEAT_3_TEMP_HOTEND 230
#define PREHEAT_3_TEMP_BED 60
#define PREHEAT_3_FAN_SPEED 255
#define AUTOTEMP
#define AUTOTEMP_OLDWEIGHT 0.98
//#define HEATERS_PARALLEL
//#define TEMP_SENSOR_1_AS_REDUNDANT
#define MAX_REDUNDANT_TEMP_SENSOR_DIFF 10 // (degC)
//#define TEMP_STAT_LEDS
#define PIDTEMP
#define BANG_MAX 255
#define PID_MAX BANG_MAX    // limits current to nozzle while PID is active (see PID_FUNCTIONAL_RANGE below); 255=full current
//#define PID_AUTOTUNE_MENU // Add PID Autotune to the LCD "Temperature" menu to run M303 and apply the result.
//#define PID_DEBUG         // Sends debug data to the serial port.
//#define PID_OPENLOOP 1    // Puts PID in open loop. M104/M140 sets the output power from 0 to PID_MAX
//#define SLOW_PWM_HEATERS  // PWM with very low frequency (roughly 0.125Hz=8s) and minimum state time of approximately 1s useful for heaters driven by a relay
#define PID_FUNCTIONAL_RANGE 10
#define K1 0.95
#define PID_dT_FACTOR 1
//#define PID_ADD_EXTRUSION_RATE
#define LPQ_MAX_LEN 50
#define DEFAULT_Kp {14.75,41.51,41.51,41.51}
#define DEFAULT_Ki {0.95,7.28,7.28,7.28}
#define DEFAULT_Kd {57.27,59.17,59.17,59.17}
#define DEFAULT_Kc {100,100,100,100}
//#define PIDTEMPBED
//#define BED_LIMIT_SWITCHING
#define BED_HYSTERESIS        2 // Only disable heating if T>target+BED_HYSTERESIS and enable heating if T>target-BED_HYSTERESIS (works only if BED_LIMIT_SWITCHING is enabled)
#define BED_CHECK_INTERVAL 5000 // ms between checks in bang-bang control
#define MAX_BED_POWER 255
#define DEFAULT_bedKp 10
#define DEFAULT_bedKi 1
#define DEFAULT_bedKd 305
//#define PID_BED_DEBUG // Sends debug data to the serial port.
//#define PIDTEMPCHAMBER
//#define CHAMBER_LIMIT_SWITCHING
#define CHAMBER_HYSTERESIS 2 //only disable heating if T>target+CHAMBER_HYSTERESIS and enable heating if T>target-CHAMBER_HYSTERESIS (works only if CHAMBER_LIMIT_SWITCHING is enabled)
#define CHAMBER_CHECK_INTERVAL 5000 //ms between checks in bang-bang control
#define MAX_CHAMBER_POWER 255
#define DEFAULT_chamberKp 10
#define DEFAULT_chamberKi 1
#define DEFAULT_chamberKd 305
//#define PID_CHAMBER_DEBUG // Sends debug data to the serial port.
//#define PIDTEMPCOOLER
//#define FAST_PWM_COOLER
//#define COOLER_LIMIT_SWITCHING
#define COOLER_HYSTERESIS 2 //only disable heating if T<target-COOLER_HYSTERESIS and enable heating if T<target+COOLER_HYSTERESIS (works only if COOLER_LIMIT_SWITCHING is enabled)
#define COOLER_CHECK_INTERVAL 5000 //ms between checks in bang-bang control
#define MAX_COOLER_POWER 255
#define DEFAULT_coolerKp 10
#define DEFAULT_coolerKi 1
#define DEFAULT_coolerKd 305
//#define PID_COOLER_DEBUG // Sends debug data to the serial port.
//#define INVERTED_HEATER_PINS
//#define INVERTED_BED_PIN
//#define INVERTED_CHAMBER_PIN
//#define INVERTED_COOLER_PIN
//#define THERMAL_PROTECTION_HOTENDS
#define THERMAL_PROTECTION_PERIOD 40
#define THERMAL_PROTECTION_HYSTERESIS 4
#define WATCH_TEMP_PERIOD  20               // Seconds
#define WATCH_TEMP_INCREASE 2               // Degrees Celsius
//#define THERMAL_PROTECTION_BED
#define THERMAL_PROTECTION_BED_PERIOD 20
#define THERMAL_PROTECTION_BED_HYSTERESIS 2
#define WATCH_BED_TEMP_PERIOD  60           // Seconds
#define WATCH_BED_TEMP_INCREASE 2           // Degrees Celsius
//#define THERMAL_PROTECTION_CHAMBER
#define THERMAL_PROTECTION_CHAMBER_PERIOD 20
#define THERMAL_PROTECTION_CHAMBER_HYSTERESIS 2
#define WATCH_CHAMBER_TEMP_PERIOD  60           // Seconds
#define WATCH_CHAMBER_TEMP_INCREASE 2           // Degrees Celsius
//#define THERMAL_PROTECTION_COOLER
#define THERMAL_PROTECTION_COOLER_PERIOD 20
#define THERMAL_PROTECTION_COOLER_HYSTERESIS 2
#define WATCH_TEMP_COOLER_PERIOD 60          // Seconds
#define WATCH_TEMP_COOLER_DECREASE 1         // Degree Celsius
#define PREVENT_COLD_EXTRUSION
#define EXTRUDE_MINTEMP 165
//#define PREVENT_LENGTHY_EXTRUDE
#define EXTRUDE_MAXLENGTH 400

/***************************
 * Configuration_Cartesian *
 ***************************/
#define KNOWN_MECH
#define CUSTOM_MACHINE_NAME "MythBust3d"
#undef ENDSTOPPULLUPS
#define ENDSTOPPULLUP_XMIN
#define ENDSTOPPULLUP_YMIN
#define ENDSTOPPULLUP_ZMIN
#define ENDSTOPPULLUP_Z2MIN
#define ENDSTOPPULLUP_Z3MIN
#define ENDSTOPPULLUP_Z4MIN
#define ENDSTOPPULLUP_XMAX
#define ENDSTOPPULLUP_YMAX
#define ENDSTOPPULLUP_ZMAX
#define ENDSTOPPULLUP_Z2MAX
#define ENDSTOPPULLUP_Z3MAX
#define ENDSTOPPULLUP_Z4MAX
#define ENDSTOPPULLUP_ZPROBE
#define ENDSTOPPULLUP_EMIN
#define X_MIN_ENDSTOP_LOGIC true
#define Y_MIN_ENDSTOP_LOGIC true
#define Z_MIN_ENDSTOP_LOGIC true
#define Z2_MIN_ENDSTOP_LOGIC false
#define Z3_MIN_ENDSTOP_LOGIC false
#define Z4_MIN_ENDSTOP_LOGIC false
#define X_MAX_ENDSTOP_LOGIC false
#define Y_MAX_ENDSTOP_LOGIC false
#define Z_MAX_ENDSTOP_LOGIC false
#define Z2_MAX_ENDSTOP_LOGIC false
#define Z3_MAX_ENDSTOP_LOGIC false
#define Z4_MAX_ENDSTOP_LOGIC false
#define Z_PROBE_ENDSTOP_LOGIC false
#define E_MIN_ENDSTOP_LOGIC false
//#define ENDSTOP_INTERRUPTS_FEATURE
#define Z_ENDSTOP_SERVO_NR -1
#define Z_ENDSTOP_SERVO_ANGLES {90,0} // Z Servo Deploy and Stow angles
//#define Z_PROBE_FIX_MOUNTED
//#define BLTOUCH
//#define Z_PROBE_SLED
#define SLED_DOCKING_OFFSET 5
#define X_PROBE_OFFSET_FROM_NOZZLE 0
#define Y_PROBE_OFFSET_FROM_NOZZLE 0
#define Z_PROBE_OFFSET_FROM_NOZZLE -1
#define XY_PROBE_SPEED 8000
#define Z_PROBE_SPEED_FAST 200
#define Z_PROBE_SPEED_SLOW 100
//#define PROBE_DOUBLE_TOUCH
//#define Z_MIN_PROBE_REPEATABILITY_TEST
#define Z_PROBE_DEPLOY_HEIGHT 15
#define Z_PROBE_BETWEEN_HEIGHT 10
#define Z_PROBE_OFFSET_RANGE_MIN -50
#define Z_PROBE_OFFSET_RANGE_MAX  50
#define X_HOME_DIR -1
#define Y_HOME_DIR -1
#define Z_HOME_DIR -1
#define E_HOME_DIR -1
#define MIN_Z_HEIGHT_FOR_HOMING 0
#define X_ENABLE_ON 0
#define Y_ENABLE_ON 0
#define Z_ENABLE_ON 0
#define E_ENABLE_ON 0
#define INVERT_X_STEP_PIN false
#define INVERT_Y_STEP_PIN false
#define INVERT_Z_STEP_PIN false
#define INVERT_E_STEP_PIN false
#define INVERT_X_DIR false
#define INVERT_Y_DIR false
#define INVERT_Z_DIR true
#define INVERT_E0_DIR false
#define INVERT_E1_DIR false
#define INVERT_E2_DIR false
#define INVERT_E3_DIR false
#define INVERT_E4_DIR false
#define INVERT_E5_DIR false
#define DISABLE_X false
#define DISABLE_Y false
#define DISABLE_Z false
#define DISABLE_E false
//#define DISABLE_INACTIVE_EXTRUDER
#define X_MAX_POS 240
#define X_MIN_POS -33
#define Y_MAX_POS 240
#define Y_MIN_POS -9
#define Z_MAX_POS 240
#define Z_MIN_POS -2.33
#define E_MIN_POS 0
#define AXIS_RELATIVE_MODES {false, false, false, false}
//#define Z_SAFE_HOMING
#define Z_SAFE_HOMING_X_POINT 100
#define Z_SAFE_HOMING_Y_POINT 100
//#define MESH_BED_LEVELING
#define MESH_INSET 10
#define MESH_NUM_X_POINTS 3
#define MESH_NUM_Y_POINTS 3
#define MESH_HOME_SEARCH_Z 5
//#define MESH_G28_REST_ORIGIN
//#define MANUAL_BED_LEVELING
#define MBL_Z_STEP 0.025
//#define AUTO_BED_LEVELING_3POINT
//#define AUTO_BED_LEVELING_LINEAR
//#define AUTO_BED_LEVELING_BILINEAR
//#define DEBUG_LEVELING_FEATURE
#define ABL_GRID_POINTS_X 3
#define ABL_GRID_POINTS_Y 3
#define LEFT_PROBE_BED_POSITION 20
#define RIGHT_PROBE_BED_POSITION 180
#define FRONT_PROBE_BED_POSITION 20
#define BACK_PROBE_BED_POSITION 180
#define MIN_PROBE_EDGE 10
//#define PROBE_Y_FIRST
//#define ABL_BILINEAR_SUBDIVISION
#define BILINEAR_SUBDIVISIONS 3
#define ABL_PROBE_PT_1_X 15
#define ABL_PROBE_PT_1_Y 180
#define ABL_PROBE_PT_2_X 15
#define ABL_PROBE_PT_2_Y 15
#define ABL_PROBE_PT_3_X 180
#define ABL_PROBE_PT_3_Y 15
//#define Z_PROBE_END_SCRIPT "G1 Z10 F8000\nG1 X10 Y10\nG1 Z0.5"
#define ENABLE_LEVELING_FADE_HEIGHT
//#define BED_CENTER_AT_0_0
//#define MANUAL_X_HOME_POS 0
//#define MANUAL_Y_HOME_POS 0
//#define MANUAL_Z_HOME_POS 0 // Distance between the nozzle to printbed after homing
// Default steps per unit               X,  Y,    Z,  E0...(per extruder)
#define DEFAULT_AXIS_STEPS_PER_UNIT   {99.80, 99.62, 400, 98.03, 100, 100, 100}
#define DEFAULT_MAX_FEEDRATE {200,200,6,50,100,100,100,100,100}
#define MANUAL_FEEDRATE {100*60,100*60,2*60,60}
#define DEFAULT_MINIMUMFEEDRATE       0.0                       // minimum feedrate
#define DEFAULT_MINTRAVELFEEDRATE     0.0
#define MINIMUM_PLANNER_SPEED         0.05                      // (mm/sec)
#define DEFAULT_MAX_ACCELERATION {550,550,120,5000,3000,1000,1000,1000,1000}
#define DEFAULT_RETRACT_ACCELERATION {5000,5000,5000,5000,5000,5000}
#define DEFAULT_ACCELERATION 550
#define DEFAULT_TRAVEL_ACCELERATION 680
#define DEFAULT_XJERK 4
#define DEFAULT_YJERK 4
#define DEFAULT_ZJERK 0.4
#define DEFAULT_EJERK {5,5,5,5,5,5}
#define HOMING_FEEDRATE_X (100*60)
#define HOMING_FEEDRATE_Y (100*60)
#define HOMING_FEEDRATE_Z (3*60)
#define X_HOME_BUMP_MM 5
#define Y_HOME_BUMP_MM 5
#define Z_HOME_BUMP_MM 2
#define HOMING_BUMP_DIVISOR {5,5,2}
#define HOTEND_OFFSET_X {0,0,0,0}
#define HOTEND_OFFSET_Y {0,0,0,0}
#define HOTEND_OFFSET_Z {0,0,0,0}
//#define HYSTERESIS
//#define ZWOBBLE
#define DEFAULT_HYSTERESIS_MM   0, 0, 0, 0  // X, Y, Z, E hysteresis in mm.
#define DEFAULT_ZWOBBLE         0, 0, 0     // A, W, P

/*************************
 * Configuration_Feature *
 *************************/
//#define FAST_PWM_FAN
//#define FAN_SOFT_PWM
#define SOFT_PWM_SCALE 0
//#define FAN_KICKSTART_TIME 100
#define FAN_MIN_PWM 45
//#define CONTROLLERFAN
#define CONTROLLERFAN_SECS       60   // How many seconds, after all motors were disabled, the fan should run
#define CONTROLLERFAN_SPEED     255   // 255 = full speed
#define CONTROLLERFAN_MIN_SPEED   0
//#define HOTEND_AUTO_FAN
#define HOTEND_AUTO_FAN_TEMPERATURE 50
#define HOTEND_AUTO_FAN_SPEED 255
#define HOTEND_AUTO_FAN_MIN_SPEED 0
//#define VOLUMETRIC_DEFAULT_ON
#define DEFAULT_NOMINAL_FILAMENT_DIA 1.75
//#define SINGLENOZZLE
//#define BARICUDA
//#define COLOR_MIXING_EXTRUDER
#define MIXING_STEPPERS 2
#define MIXING_VIRTUAL_TOOLS 16
//#define MKR4
//#define INVERTED_RELE_PINS
//#define MKR6
//#define INVERTED_RELE_PINS
//#define MKSE6
#define MKSE6_SERVO_INDEX    0
#define MKSE6_SERVOPOS_E0  -60
#define MKSE6_SERVOPOS_E1  -30
#define MKSE6_SERVOPOS_E2    0
#define MKSE6_SERVOPOS_E3   30
#define MKSE6_SERVOPOS_E4   60
#define MKSE6_SERVOPOS_E5   90
#define MKSE6_SERVO_DELAY 1000
//#define NPR2
#define COLOR_STEP {0,45,90,135}
#define COLOR_SLOWRATE 170           // MICROSECOND delay for carter motor routine (Carter Motor Feedrate: upper value-slow feedrate)  
#define COLOR_HOMERATE 4             // FEEDRATE for carter home
#define MOTOR_ANGLE 1.8              // Nema angle for single step 
#define DRIVER_MICROSTEP 4           // Microstep moltiplicator driver (set jumper MS1-2-3) off-on-off 1/4 microstepping.
#define CARTER_MOLTIPLICATOR 14.22   // CARTER MOLTIPLICATOR (gear ratio 13/31-10/31)
//#define DONDOLO_SINGLE_MOTOR
//#define DONDOLO_DUAL_MOTOR
#define DONDOLO_SERVO_INDEX 0
#define DONDOLO_SERVOPOS_E0 120
#define DONDOLO_SERVOPOS_E1 10
#define DONDOLO_SERVO_DELAY 1000
//#define IDLE_OOZING_PREVENT
#define IDLE_OOZING_MINTEMP           190
#define IDLE_OOZING_FEEDRATE          50    //default feedrate for retracting (mm/s)
#define IDLE_OOZING_SECONDS           5
#define IDLE_OOZING_LENGTH            15    //default retract length (positive mm)
#define IDLE_OOZING_RECOVER_LENGTH    0     //default additional recover length (mm, added to retract length when recovering)
#define IDLE_OOZING_RECOVER_FEEDRATE  50    //default feedrate for recovering from retraction (mm/s)
//#define EXTRUDER_RUNOUT_PREVENT
#define EXTRUDER_RUNOUT_MINTEMP 190
#define EXTRUDER_RUNOUT_SECONDS  30
#define EXTRUDER_RUNOUT_SPEED  1500 // mm/m
#define EXTRUDER_RUNOUT_EXTRUDE   5 // mm
//#define EASY_LOAD
#define BOWDEN_LENGTH 250
#define LCD_PURGE_LENGTH 10
#define LCD_RETRACT_LENGTH 5
#define LCD_PURGE_FEEDRATE 3
#define LCD_RETRACT_FEEDRATE 5
#define LCD_LOAD_FEEDRATE 20
#define LCD_UNLOAD_FEEDRATE 20
//#define ADVANCE
#define EXTRUDER_ADVANCE_K 0.0
#define D_FILAMENT 1.75
//#define LIN_ADVANCE
#define LIN_ADVANCE_K 75
#define FILAMENT_CHANGE_FEATURE
#define FILAMENT_CHANGE_X_POS 3             // X position of hotend
#define FILAMENT_CHANGE_Y_POS 3             // Y position of hotend
#define FILAMENT_CHANGE_Z_ADD 10            // Z addition of hotend (lift)
#define FILAMENT_CHANGE_XY_FEEDRATE 100
#define FILAMENT_CHANGE_Z_FEEDRATE 5
#define FILAMENT_CHANGE_RETRACT_LENGTH 2
#define FILAMENT_CHANGE_RETRACT_FEEDRATE 50
#define FILAMENT_CHANGE_UNLOAD_LENGTH 100
#define FILAMENT_CHANGE_UNLOAD_FEEDRATE 100
#define FILAMENT_CHANGE_LOAD_LENGTH 100
#define FILAMENT_CHANGE_LOAD_FEEDRATE 100
#define FILAMENT_CHANGE_EXTRUDE_LENGTH 50
#define FILAMENT_CHANGE_EXTRUDE_FEEDRATE 5
#define FILAMENT_CHANGE_PRINTER_OFF 5
#define SOFTWARE_MIN_ENDSTOPS true  // If true, axis won't move to coordinates less than HOME_POS.
#define SOFTWARE_MAX_ENDSTOPS true  // If true, axis won't move to coordinates greater than the defined lengths below.
#define ENDSTOPS_ONLY_FOR_HOMING
//#define ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED
#define ABORT_ON_ENDSTOP_HIT_INIT true
#define MESH_MIN_X (X_MIN_POS + MESH_INSET)
#define MESH_MAX_X (X_MAX_POS - (MESH_INSET))
#define MESH_MIN_Y (Y_MIN_POS + MESH_INSET)
#define MESH_MAX_Y (Y_MAX_POS - (MESH_INSET))
//#define ENABLE_SERVOS
#define NUM_SERVOS 0
//#define DEACTIVATE_SERVOS_AFTER_MOVE
#define SERVO_DEACTIVATION_DELAY 300
//#define Z_LATE_ENABLE
#define SLOWDOWN
//#define QUICK_HOME
//#define HOME_Y_BEFORE_X
//#define FORCE_HOME_XY_BEFORE_Z
//#define BABYSTEPPING
#define BABYSTEP_XY  
#define BABYSTEP_INVERT_Z false
#define BABYSTEP_MULTIPLICATOR 2
//#define FWRETRACT                     //ONLY PARTIALLY TESTED
#define MIN_RETRACT                 0.1 //minimum extruded mm to accept a automatic gcode retraction attempt
#define RETRACT_LENGTH              3   //default retract length (positive mm)
#define RETRACT_LENGTH_SWAP        13   //default swap retract length (positive mm), for extruder change
#define RETRACT_FEEDRATE           45   //default feedrate for retracting (mm/s)
#define RETRACT_ZLIFT               0   //default retract Z-lift
#define RETRACT_RECOVER_LENGTH      0   //default additional recover length (mm, added to retract length when recovering)
#define RETRACT_RECOVER_LENGTH_SWAP 0   //default additional swap recover length (mm, added to retract length when recovering from extruder change)
#define RETRACT_RECOVER_FEEDRATE    8   //default feedrate for recovering from retraction (mm/s)
//#define DUAL_X_CARRIAGE
#define X2_MIN_POS 80     // set minimum to ensure second x-carriage doesn't hit the parked first X-carriage
#define X2_MAX_POS 353    // set maximum to the distance between toolheads when both heads are homed
#define X2_HOME_DIR 1     // the second X-carriage always homes to the maximum endstop position
#define X2_HOME_POS X2_MAX_POS // default home position is the maximum carriage position
#define DEFAULT_DUAL_X_CARRIAGE_MODE DXC_FULL_CONTROL_MODE
#define TOOLCHANGE_PARK_ZLIFT   0.2      // the distance to raise Z axis when parking an extruder
#define TOOLCHANGE_UNPARK_ZLIFT 1        // the distance to raise Z axis when unparking an extruder
#define DEFAULT_DUPLICATION_X_OFFSET 100
//#define X_TWO_STEPPER
#define INVERT_X2_VS_X_DIR false
//#define Y_TWO_STEPPER
#define INVERT_Y2_VS_Y_DIR false
//#define Z_TWO_STEPPER
//#define Z_THREE_STEPPER
//#define Z_FOUR_STEPPER
//#define Z_TWO_ENDSTOPS
//#define Z_THREE_ENDSTOPS
//#define Z_FOUR_ENDSTOPS
//#define XY_FREQUENCY_LIMIT  15
//#define SF_ARC_FIX
//#define FILAMENT_SENSOR
#define FILAMENT_SENSOR_EXTRUDER_NUM 0
#define MEASUREMENT_DELAY_CM         14     //measurement delay in cm.  This is the distance from filament sensor to middle of barrel
#define MEASURED_UPPER_LIMIT 2
#define MEASURED_LOWER_LIMIT 1.35
#define MAX_MEASUREMENT_DELAY        20     //delay buffer size in bytes (1 byte = 1cm)- limits maximum measurement delay allowable (must be larger than MEASUREMENT_DELAY_CM  and lower number saves RAM)
#define DEFAULT_MEASURED_FILAMENT_DIA  DEFAULT_NOMINAL_FILAMENT_DIA  //set measured to nominal initially
//#define FILAMENT_LCD_DISPLAY
//#define FILAMENT_RUNOUT_SENSOR
#define FIL_RUNOUT_PIN_INVERTING false
//#define ENDSTOPPULLUP_FIL_RUNOUT
#define FILAMENT_RUNOUT_SCRIPT "M600"
//#define POWER_CONSUMPTION
#define POWER_VOLTAGE      12.00    //(V) The power supply OUT voltage
#define POWER_SENSITIVITY   0.066   //(V/A) How much increase V for 1A of increase
#define POWER_OFFSET        0.005   //(A) Help to get 0A when no load is connected.
#define POWER_ZERO          2.500   //(V) The /\V coming out from the sensor when no current flow.
#define POWER_ERROR         0.0     //(%) Ammortize measure error.
#define POWER_EFFICIENCY  100.0     //(%) The power efficency of the power supply
//#define POWER_CONSUMPTION_LCD_DISPLAY
//#define FLOWMETER_SENSOR
#define FLOWMETER_MAXFLOW  6.0      // Liters per minute max
#define FLOWMETER_MAXFREQ  55       // frequency of pulses at max flow
//#define MINFLOW_PROTECTION 4      
//#define DOOR_OPEN
#define DOOR_OPEN_LOGIC false
#define DOOR_OPEN_PULLUP
#define EEPROM_SETTINGS
#define EEPROM_CHITCHAT // Uncomment this to enable EEPROM Serial responses.
//#define DISABLE_M503
#define SDSUPPORT
//#define SDSLOW              // Use slower SD transfer mode (not normally needed - uncomment if you're getting volume init error)
//#define SDEXTRASLOW         // Use even slower SD transfer mode (not normally needed - uncomment if you're getting volume init error)
//#define SD_CHECK_AND_RETRY  // Use CRC checks and retries on the SD communication
//#define SD_EXTENDED_DIR     // Show extended directory including file length. Don't use this with Pronterface
//#define SD_DISABLED_DETECT
//#define SD_DETECT_INVERTED
#define SD_FINISHED_STEPPERRELEASE true  //if sd support and the file is finished: disable steppers?
#define SD_FINISHED_RELEASECOMMAND "M84 X Y Z E" // You might want to keep the z enabled so your bed stays in place.
#define SDCARD_RATHERRECENTFIRST  //reverse file order of sd card menu display. Its sorted practically after the file system block order.
//#define MENU_ADDAUTOSTART
//#define SD_SETTINGS                     // Uncomment to enable
#define SD_CFG_SECONDS        300         // seconds between update
#define CFG_SD_FILE           "INFO.CFG"  // name of the configuration file
#define LCD_LANGUAGE it
#define DISPLAY_CHARSET_HD44780 JAPANESE
#define SHOW_BOOTSCREEN
//#define SHOW_CUSTOM_BOOTSCREEN
#define STRING_SPLASH_LINE1 "v" SHORT_BUILD_VERSION       // will be shown during bootup in line 1
#define STRING_SPLASH_LINE2 STRING_DISTRIBUTION_DATE      // will be shown during bootup in line 2
#define SPLASH_SCREEN_DURATION 5000                       // SPLASH SCREEN duration in millisecond
//#define ULTRA_LCD   // Character based
//#define DOGLCD      // Full graphics display
#define XYZ_HOLLOW_FRAME
#define MENU_HOLLOW_FRAME
//#define USE_BIG_EDIT_FONT
//#define USE_SMALL_INFOFONT
//#define DOGM_SPI_DELAY_US 5
//#define ENCODER_PULSES_PER_STEP 1
//#define ENCODER_STEPS_PER_MENU_ITEM 5
//#define LCD_SCREEN_ROT_90    // Rotate screen orientation for graphics display by 90 degree clockwise
//#define LCD_SCREEN_ROT_180   // Rotate screen orientation for graphics display by 180 degree clockwise
//#define LCD_SCREEN_ROT_270   // Rotate screen orientation for graphics display by 270 degree clockwise
//#define INVERT_CLICK_BUTTON           // Option for invert encoder button logic
//#define INVERT_BACK_BUTTON            // Option for invert back button logic if avaible
//#define REVERSE_ENCODER_DIRECTION
//#define REVERSE_MENU_DIRECTION
#define ENCODER_RATE_MULTIPLIER         // If defined, certain menu edit operations automatically multiply the steps when the encoder is moved quickly
#define ENCODER_10X_STEPS_PER_SEC 75    // If the encoder steps per sec exceeds this value, multiply steps moved x10 to quickly advance the value
#define ENCODER_100X_STEPS_PER_SEC 160  // If the encoder steps per sec exceeds this value, multiply steps moved x100 to really quickly advance the value
#define ULTIPANEL_FEEDMULTIPLY          // Comment to disable setting feedrate multiplier via encoder
//#define SPEAKER
//#define LCD_FEEDBACK_FREQUENCY_DURATION_MS 100
//#define LCD_FEEDBACK_FREQUENCY_HZ 1000
//#define UI_VOLTAGE_LEVEL 0 // 3.3 V
#define UI_VOLTAGE_LEVEL 1   // 5 V
#define LCD_INFO_MENU
//#define ULTIMAKERCONTROLLER
//#define ULTIPANEL
//#define CARTESIO_UI
//#define RADDS_DISPLAY
//#define PANEL_ONE
//#define MAKRPANEL
//#define REPRAPWORLD_GRAPHICAL_LCD
//#define VIKI2
//#define miniVIKI
//#define ELB_FULL_GRAPHIC_CONTROLLER
//#define REPRAP_DISCOUNT_SMART_CONTROLLER
//#define G3D_PANEL
//#define REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER
//#define MINIPANEL
//#define REPRAPWORLD_KEYPAD
//#define REPRAPWORLD_KEYPAD_MOVE_STEP 1.0
//#define RIGIDBOT_PANEL
//#define BQ_LCD_SMART_CONTROLLER
//#define RA_CONTROL_PANEL
//#define LCD_I2C_SAINSMART_YWROBOT
//#define LCM1602
//#define LCD_I2C_PANELOLU2
//#define LCD_I2C_VIKI
//#define U8GLIB_SSD1306
//#define SAV_3DGLCD
//#define SAV_3DLCD
#define NEXTION
#define NEXTION_SERIAL 1
#define NEXTION_UPDATE_INTERVAL 3000
#define NEXTION_GFX
//#define NEXTION_WAVETEMP
#define NEXTION_FIRMWARE_FILE "MythBust3d.tft"
//#define LCD_PROGRESS_BAR
#define PROGRESS_BAR_BAR_TIME 5000
#define PROGRESS_BAR_MSG_TIME 1500
#define PROGRESS_MSG_EXPIRE 0
//#define PROGRESS_MSG_ONCE
//#define LCD_PROGRESS_BAR_TEST
//#define PHOTOGRAPH
//#define CHDK
#define CHDK_DELAY 50   //How long in ms the pin should stay HIGH before going LOW again
//#define RFID_MODULE
#define RFID_SERIAL 1
//#define BLINKM
//#define RGB_LED
//#define LASERBEAM
//#define CASE_LIGHT
#define INVERT_CASE_LIGHT false
//#define CASE_LIGHT_DEFAULT_ON
#define DEFAULT_STEPPER_DEACTIVE_TIME 120
#define MINIMUM_STEPPER_PULSE 0
//#define ENABLE_HIGH_SPEED_STEPPING
//#define USE_MICROSTEPS
#define MICROSTEP_MODES {16,16,16,16}
#define MOTOR_CURRENT {1,1,1,1,1,1,1}
#define DIGIPOT_MOTOR_CURRENT {135, 135, 135, 135, 135}
#define PWM_MOTOR_CURRENT {1200, 1000, 1000}
//#define DIGIPOT_I2C
#define DIGIPOT_I2C_NUM_CHANNELS 8
#define DIGIPOT_I2C_MOTOR_CURRENTS {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}
//#define CONFIG_STEPPERS_TOSHIBA
//#define HAVE_TMCDRIVER
//#define HAVE_TMCDRIVER
  //#define X_IS_TMC
  //#define X2_IS_TMC
  //#define Y_IS_TMC
  //#define Y2_IS_TMC
  //#define Z_IS_TMC
  //#define Z2_IS_TMC
  //#define E0_IS_TMC
  //#define E1_IS_TMC
  //#define E2_IS_TMC
  //#define E3_IS_TMC
  #define X_MAX_CURRENT     1000 // in mA
  #define X_SENSE_RESISTOR    91 // in mOhms
  #define X_MICROSTEPS        16 // number of microsteps
  #define X2_MAX_CURRENT    1000
  #define X2_SENSE_RESISTOR   91
  #define X2_MICROSTEPS       16
  #define Y_MAX_CURRENT     1000
  #define Y_SENSE_RESISTOR    91
  #define Y_MICROSTEPS        16
  #define Y2_MAX_CURRENT    1000
  #define Y2_SENSE_RESISTOR   91
  #define Y2_MICROSTEPS       16
  #define Z_MAX_CURRENT     1000
  #define Z_SENSE_RESISTOR    91
  #define Z_MICROSTEPS        16
  #define Z2_MAX_CURRENT    1000
  #define Z2_SENSE_RESISTOR   91
  #define Z2_MICROSTEPS       16
  #define E0_MAX_CURRENT    1000
  #define E0_SENSE_RESISTOR   91
  #define E0_MICROSTEPS       16
  #define E1_MAX_CURRENT    1000
  #define E1_SENSE_RESISTOR   91
  #define E1_MICROSTEPS       16
  #define E2_MAX_CURRENT    1000
  #define E2_SENSE_RESISTOR   91
  #define E2_MICROSTEPS       16
  #define E3_MAX_CURRENT    1000
  #define E3_SENSE_RESISTOR   91
  #define E3_MICROSTEPS       16
//#define HAVE_L6470DRIVER
  //#define X_IS_L6470
  //#define X2_IS_L6470
  //#define Y_IS_L6470
  //#define Y2_IS_L6470
  //#define Z_IS_L6470
  //#define Z2_IS_L6470
  //#define E0_IS_L6470
  //#define E1_IS_L6470
  //#define E2_IS_L6470
  //#define E3_IS_L6470
  #define X_MICROSTEPS      16 // number of microsteps
  #define X_K_VAL           50 // 0 - 255, Higher values, are higher power. Be careful not to go too high
  #define X_OVERCURRENT   2000 // maxc current in mA. If the current goes over this value, the driver will switch off
  #define X_STALLCURRENT  1500 // current in mA where the driver will detect a stall
  #define X2_MICROSTEPS     16
  #define X2_K_VAL          50
  #define X2_OVERCURRENT  2000
  #define X2_STALLCURRENT 1500
  #define Y_MICROSTEPS      16
  #define Y_K_VAL           50
  #define Y_OVERCURRENT   2000
  #define Y_STALLCURRENT  1500
  #define Y2_MICROSTEPS     16
  #define Y2_K_VAL          50
  #define Y2_OVERCURRENT  2000
  #define Y2_STALLCURRENT 1500
  #define Z_MICROSTEPS      16
  #define Z_K_VAL           50
  #define Z_OVERCURRENT   2000
  #define Z_STALLCURRENT  1500
  #define Z2_MICROSTEPS     16
  #define Z2_K_VAL          50
  #define Z2_OVERCURRENT  2000
  #define Z2_STALLCURRENT 1500
  #define E0_MICROSTEPS     16
  #define E0_K_VAL          50
  #define E0_OVERCURRENT  2000
  #define E0_STALLCURRENT 1500
  #define E1_MICROSTEPS     16
  #define E1_K_VAL          50
  #define E1_OVERCURRENT  2000
  #define E1_STALLCURRENT 1500
  #define E2_MICROSTEPS     16
  #define E2_K_VAL          50
  #define E2_OVERCURRENT  2000
  #define E2_STALLCURRENT 1500
  #define E3_MICROSTEPS     16
  #define E3_K_VAL          50
  #define E3_OVERCURRENT  2000
  #define E3_STALLCURRENT 1500
#define BLOCK_BUFFER_SIZE 16
#define MAX_CMD_SIZE 96
#define BUFSIZE 4
#define NUM_POSITON_SLOTS 2
#define DEFAULT_MINSEGMENTTIME 20000
#define ARC_SUPPORT
#define MM_PER_ARC_SEGMENT 1
#define N_ARC_CORRECTION 25
#define MIN_STEPS_PER_SEGMENT 6
//#define M100_FREE_MEMORY_WATCHER
#define M100_FREE_MEMORY_DUMPER
#define M100_FREE_MEMORY_CORRUPTOR
//#define NOZZLE_CLEAN_FEATURE
#define NOZZLE_CLEAN_STROKES 12
#define NOZZLE_CLEAN_TRIANGLES 3
#define NOZZLE_CLEAN_START_POINT {30,30,1}
#define NOZZLE_CLEAN_END_POINT {100,60,100}
#define NOZZLE_CLEAN_GOBACK
//#define NOZZLE_PARK_FEATURE
#define NOZZLE_PARK_POINT { (X_MIN_POS + 10), (Y_MAX_POS - 10), 20 }
//#define INCH_MODE_SUPPORT
//#define JSON_OUTPUT
//#define PINS_DEBUGGING
//#define AUTO_REPORT_TEMPERATURES
//#define EXTENDED_CAPABILITIES_REPORT
//#define USE_WATCHDOG
//#define WATCHDOG_RESET_MANUAL
//#define START_GCODE
#define START_PRINTING_SCRIPT "G28\nG1 Z10 F8000"
//#define STOP_GCODE
#define STOP_PRINTING_SCRIPT "G28\nM107\nM104 T0 S0\nM140 S0\nM84\nM81"


/*
========== Start configuration string ==========
{
"printer": "prusa_i3_r2",
"processor": 0,
"serial": 0,
"baudrates": 250000,
"btserial": 0,
"btbaudrates": 115200,
"customname": "none",
"customconfig": "default config",
"machineuuid": "00000000-0000-0000-0000-000000000000",
"notimeouts": 1000,
"advancedok": "0",
"killMethod": 0,
"motherboards": "",
"mechanism": 0,
"power": "0",
"defaultpower": "0",
"extruders": 1,
"driverextruders": 1,
"bed": "1",
"chamber": "0",
"cooler": "1",
"tempunitsupport": "0",
"tempsensor0": "1",
"tempsensor1": "0",
"tempsensor2": "0",
"tempsensor3": "0",
"tempsensorbed": "1",
"tempsensorchamber": "0",
"tempsensorcooler": "0",
"ad595offset": 0,
"ad595gain": 1,
"dummy998": 25,
"dummy999": 25,
"showadc": "0",
"maxtemp0": 275,
"maxtemp1": 275,
"maxtemp2": 275,
"maxtemp3": 275,
"maxtempbed": 150,
"maxtempchamber": 150,
"maxtempcooler": 150,
"mintemp0": 5,
"mintemp1": 5,
"mintemp2": 5,
"mintemp3": 5,
"mintempbed": 5,
"mintempchamber": 5,
"mintempcooler": 5,
"plahotendtemp": 200,
"plabedtemp": 60,
"plafanspeed": 60,
"abshotendtemp": 250,
"absbedtemp": 90,
"absfanspeed": 0,
"gumhotendtemp": 230,
"gumbedtemp": 60,
"gumfanspeed": 255,
"autotemp": "1",
"autotempoldweight": 0.98,
"bangmax": 255,
"pidtemp": "1",
"pidextrusionrate": "0",
"pidkp0": 14.75,
"pidki0": 0.95,
"pidkd0": 57.27,
"pidkc0": 100,
"pidkp1": 41.51,
"pidki1": 7.28,
"pidkd1": 59.17,
"pidkc1": 100,
"pidkp2": 41.51,
"pidki2": 7.28,
"pidkd2": 59.17,
"pidkc2": 100,
"pidkp3": 41.51,
"pidki3": 7.28,
"pidkd3": 59.17,
"pidkc3": 100,
"pidbedtemp": "0",
"maxbedpower": 255,
"pidbedkp": 10,
"pidbedki": 1,
"pidbedkd": 305,
"pidchambertemp": "0",
"maxchamberpower": 255,
"pidchamberkp": 10,
"pidchamberki": 1,
"pidchamberkd": 305,
"pidcoolertemp": "0",
"fastpwmcooler": "0",
"maxcoolerpower": 255,
"pidcoolerkp": 10,
"pidcoolerki": 1,
"pidcoolerkd": 305,
"invertedheaterpins": "0",
"invertedbedpin": "0",
"invertedchamberpin": "0",
"invertedcoolerpin": "0",
"thermalprotectionhotend": "0",
"thermalprotectionperiod": 40,
"thermalprotectionhysteresis": 4,
"thermalprotectionbed": "0",
"thermalprotectionbedperiod": 20,
"thermalprotectionbedhysteresis": 2,
"thermalprotectionchamber": "0",
"thermalprotectionchamberperiod": 20,
"thermalprotectionchamberhysteresis": 2,
"thermalprotectioncooler": "0",
"thermalprotectioncoolerperiod": 20,
"thermalprotectioncoolerhysteresis": 2,
"uiprintername": "Anet A8 Pcelli85",
"Xminendstop": "1",
"Xmaxendstop": "0",
"Yminendstop": "1",
"Ymaxendstop": "0",
"Zminendstop": "1",
"Zmaxendstop": "0",
"Z2minendstop": "0",
"Z2maxendstop": "0",
"Z3minendstop": "0",
"Z3maxendstop": "0",
"Z4minendstop": "0",
"Z4maxendstop": "0",
"Zprobeendstop": "0",
"Eminendstop": "0",
"Xhoming": 0,
"Yhoming": 0,
"Zhoming": 0,
"Ehoming": 0,
"Xinvertenable": 0,
"Yinvertenable": 0,
"Zinvertenable": 0,
"Einvertenable": 0,
"Xinvertstep": "0",
"Yinvertstep": "0",
"Zinvertstep": "0",
"Einvertstep": "0",
"Xinvertdir": "0",
"Yinvertdir": "0",
"Zinvertdir": "0",
"E0invertdir": "0",
"E1invertdir": "0",
"E2invertdir": "0",
"E3invertdir": "0",
"E4invertdir": "0",
"E5invertdir": "0",
"disableX": 0,
"disableY": 0,
"disableZ": 0,
"disableE": 0,
"Xmaxpos": 240,
"Xminpos": -10,
"Ymaxpos": 270,
"Yminpos": -58,
"Zmaxpos": 240,
"Zminpos": 0,
"Zsafehoming": "0",
"ZsafehomingX": 100,
"ZsafehomingY": 100,
"Zminheightbeforehoming": 0,
"Zprobetype": 0,
"Zprobesledoffset": 5,
"Xprobeoffset": 0,
"Yprobeoffset": 0,
"Zprobeoffset": -1,
"xyprobespeed": 8000,
"zprobespeed": 3600,
"zprobespeedfast": 200,
"zprobespeedslow": 100,
"zprobingrepeat": "0",
"Zraiseprobedeploystow": 15,
"Zraisebetweenprobe": 10,
"levelingfadeheight": "1",
"meshlevel": "0",
"meshinset": 10,
"meshnumxpoint": 3,
"meshnumypoint": 3,
"meshhomesearchz": 5,
"meshg28rest": "0",
"manualbedlevel": "0",
"mblzstep": 0.025,
"abl3point": "0",
"abllinear": "0",
"ablbilinear": "0",
"gridpointX": 3,
"gridpointY": 3,
"leftprobe": 20,
"rightprobe": 180,
"backprobe": 180,
"frontprobe": 20,
"Xprobe1": 15,
"Yprobe1": 180,
"Xprobe2": 15,
"Yprobe2": 15,
"Xprobe3": 180,
"Yprobe3": 15,
"manualhomepos": "0",
"bedcenter00": "0",
"Xhomepos": 0,
"Yhomepos": 0,
"Zhomepos": 0,
"Xstepspermm": 80,
"Ystepspermm": 80,
"Zstepspermm": 3200,
"E0stepspermm": 620,
"E1stepspermm": 625,
"E2stepspermm": 625,
"E3stepspermm": 625,
"E4stepspermm": 625,
"E5stepspermm": 625,
"Xmaxspeed": 1000,
"Ymaxspeed": 1000,
"Zmaxspeed": 3,
"E0maxspeed": 50,
"E1maxspeed": 100,
"E2maxspeed": 100,
"E3maxspeed": 100,
"E4maxspeed": 100,
"E5maxspeed": 100,
"Xmanualspeed": 100,
"Ymanualspeed": 100,
"Zmanualspeed": 2,
"minimumspeed": 0,
"minimumtravelspeed": 0,
"minimumplannerspeed": 0.05,
"Xmaxacceleration": 5000,
"Ymaxacceleration": 5000,
"Zmaxacceleration": 50,
"E0maxacceleration": 5000,
"E1maxacceleration": 3000,
"E2maxacceleration": 3000,
"E3maxacceleration": 3000,
"E4maxacceleration": 3000,
"E5maxacceleration": 3000,
"E0retractacceleration": 10000,
"E1retractacceleration": 10000,
"E2retractacceleration": 10000,
"E3retractacceleration": 10000,
"E4retractacceleration": 10000,
"E5retractacceleration": 10000,
"defaultacceleration": 3000,
"defaulttravelacceleration": 3000,
"maxXjerk": 10,
"maxYjerk": 10,
"maxZjerk": 0.4,
"maxE0jerk": 5,
"maxE1jerk": 5,
"maxE2jerk": 5,
"maxE3jerk": 5,
"maxE4jerk": 5,
"maxE5jerk": 5,
"Xhomingspeed": 100,
"Yhomingspeed": 100,
"Zhomingspeed": 3,
"XbumpMM": 5,
"YbumpMM": 5,
"ZbumpMM": 2,
"Xbumpdivisor": 5,
"Ybumpdivisor": 5,
"Zbumpdivisor": 2,
"hotendoffsetXE1": 0,
"hotendoffsetXE2": 0,
"hotendoffsetXE3": 0,
"hotendoffsetYE1": 0,
"hotendoffsetYE2": 0,
"hotendoffsetYE3": 0,
"hotendoffsetZE1": 0,
"hotendoffsetZE2": 0,
"hotendoffsetZE3": 0,
"deltasegmentpersecond": 200,
"deltadiagonalrod": 220,
"deltasmoothrodoffset": 145,
"deltaeffectoroffset": 20,
"deltacarriageoffset": 20,
"deltaprinterradius": 70,
"deltaheight": 210,
"towerAendstop": 0,
"towerBendstop": 0,
"towerCendstop": 0,
"towerAposition": 0,
"towerBposition": 0,
"towerCposition": 0,
"towerAradius": 0,
"towerBradius": 0,
"towerCradius": 0,
"towerAdiagonalrod": 0,
"towerBdiagonalrod": 0,
"towerCdiagonalrod": 0,
"deltaautoprecision": 0.1,
"deltaautogrid": 20,
"deltaXdeploystart": 0,
"deltaYdeploystart": 0,
"deltaZdeploystart": 30,
"deltaXdeployend": 0,
"deltaYdeployend": 0,
"deltaZdeployend": 0,
"deltaXretractstart": 0,
"deltaYretractstart": 0,
"deltaZretractstart": 30,
"deltaXretractend": 0,
"deltaYretractend": 0,
"deltaZretractend": 0,
"deltaautobed": "0",
"deltaautocalibration": "0",
"deltaautocalibration7p": "0",
"deltahomesafezone": "1",
"fastpwmfan": "0",
"softpwmfan": "0",
"hotendautofan": "1",
"Ecoolertemp": 50,
"Ecoolerspeed": 255,
"Ecoolerminspeed": 0,
"defaultfilamentdia": 1.75,
"dangerousextrude": "1",
"extrudemintemp": 165,
"lengthextrude": "1",
"extrudemaxlenght": 400,
"singlenozzle": "0",
"baricuda": "0",
"colormixingextruder": "0",
"mixingsteppers": "2",
"virtualtools": 16,
"mkr4": "0",
"invertrelepin": "0",
"E0E1pin": 5,
"E0E2pin": 5,
"E1E3pin": 6,
"mkr6": "0",
"EX1pin": -1,
"EX2pin": -1,
"npr2": "0",
"E0angle": 0,
"E1angle": 45,
"E2angle": 90,
"E3angle": 135,
"E4angle": 180,
"E5angle": 225,
"dondolo": "0",
"dondolodualmotor": "0",
"dondoloservo": 0,
"dondoloservoe0": 120,
"dondoloservoe1": 10,
"dondolodelay": 1000,
"easyload": 0,
"bowdenlenght": 250,
"lcdpurgelenght": 10,
"lcdretractlenght": 5,
"lcdpurgefeedrate": 3,
"lcdretractfeedrate": 5,
"lcdloadfeedrate": 20,
"lcdunloadfeedrate": 20,
"filamentchangeenable": "1",
"filamentchangeXpos": 3,
"filamentchangeYpos": 3,
"filamentchangeZadd": 10,
"filamentchangexyfr": 100,
"filamentchangezfr": 5,
"filamentchangeretract": 2,
"filamentchangeretractfr": 50,
"filamentchangeunload": 100,
"filamentchangeunloadfr": 100,
"filamentchangeload": 100,
"filamentchangeloadfr": 100,
"filamentchangeextrude": 50,
"filamentchangeextrudefr": 5,
"filamentchangeprinteroff": 5,
"softwareminendstop": "1",
"softwaremaxendstop": "1",
"endstoponlyforhome": "1",
"abortendstophit": "0",
"abortendstophitinit": "1",
"servos": "0",
"numservos": 0,
"Zservo": "0",
"angleextendservosZ": 90,
"angleretractservosZ": 0,
"servodeactivate": "0",
"servodeactivatedelay": 300,
"Xtwostepper": "0",
"X2vsXdir": "0",
"Ytwostepper": "0",
"Y2vsYdir": "0",
"Zplusstepper": 0,
"Ztwoendstop": "0",
"Zthreeendstop": "0",
"Zfourendstop": "0",
"filamentsensor": "0",
"filamentsensorextruder": 0,
"filamentsensormaxdia": 2,
"filamentsensormindia": 1.35,
"filamentsensordia": 1.75,
"filamentsensorlcd": "0",
"filamentrunout": "0",
"filamentrunoutpininverting": "0",
"filamentrunoutpullup": "1",
"filamentrunoutscript": "M600",
"powerconsumption": "0",
"dooropen": "0",
"doorendstop": "0",
"eeprom": "1",
"eepromchitchat": "1",
"sdsupport": "1",
"sdslow": "0",
"sdextraslow": "0",
"sddisableddetect": "0",
"sddetectinverted": "0",
"sdsetting": "0",
"lcdlanguages": "it",
"invertclickbutton": "0",
"invertbackbutton": "0",
"invertrotaryswitch": 0,
"invertmenudirection": "0",
"displays": 12,
"nextion_port": 1,
"nextionGFX": "1",
"lcdprogressbar": 0,
"lcdprogressbarbartime": 3,
"lcdprogressbarmsgtime": 1,
"lcdprogressbarmsgexpire": 0,
"laserbeam": "0",
"lasercontrol": 1,
"laserfocus": "0",
"laserraster": "0",
"usemicrostep": "0",
"Xmicrostep": 16,
"Ymicrostep": 16,
"Zmicrostep": 16,
"Emicrostep": 16,
"Xcurrent": 1000,
"Ycurrent": 1000,
"Zcurrent": 1000,
"E0current": 1000,
"E1current": 1000,
"E2current": 1000,
"E3current": 1000,
"E4current": 1000,
"E5current": 1000,
"toshiba": "0",
"jsonoutput": "0",
"testmode": "0",
"inchmodesupport": "0",
"blockbuffersize": 16,
"bufsize": 4,
"nozzlecleanfeature": "0",
"nozzlecleanstrokes": 12,
"nozzlecleantriangle": 3,
"nozzlecleanstart_x": 30,
"nozzlecleanstart_y": 30,
"nozzlecleanstart_z": 1,
"nozzlecleanend_x": 100,
"nozzlecleanend_y": 60,
"nozzlecleanend_z": 1,
"nozzlecleangoback": "1",
"Xmotor": {
  "name": "X motor",
  "step": "ORIG_X_STEP_PIN",
  "dir": "ORIG_X_DIR_PIN",
  "enable": "ORIG_X_ENABLE_PIN"
},
"Ymotor": {
  "name": "Y motor",
  "step": "ORIG_Y_STEP_PIN",
  "dir": "ORIG_Y_DIR_PIN",
  "enable": "ORIG_Y_ENABLE_PIN"
},
"Zmotor": {
  "name": "Z motor",
  "step": "ORIG_Z_STEP_PIN",
  "dir": "ORIG_Z_DIR_PIN",
  "enable": "ORIG_Z_ENABLE_PIN"
},
"X2motor": {
  "name": "Extruder 1",
  "step": "ORIG_E1_STEP_PIN",
  "dir": "ORIG_E1_DIR_PIN",
  "enable": "ORIG_E1_ENABLE_PIN"
},
"Y2motor": {
  "name": "Extruder 1",
  "step": "ORIG_E1_STEP_PIN",
  "dir": "ORIG_E1_DIR_PIN",
  "enable": "ORIG_E1_ENABLE_PIN"
},
"Z2motor": {
  "name": "Extruder 1",
  "step": "ORIG_E1_STEP_PIN",
  "dir": "ORIG_E1_DIR_PIN",
  "enable": "ORIG_E1_ENABLE_PIN"
},
"Z3motor": {
  "name": "Extruder 2",
  "step": "ORIG_E2_STEP_PIN",
  "dir": "ORIG_E2_DIR_PIN",
  "enable": "ORIG_E2_ENABLE_PIN"
},
"Z4motor": {
  "name": "Extruder 3",
  "step": "ORIG_E3_STEP_PIN",
  "dir": "ORIG_E3_DIR_PIN",
  "enable": "ORIG_E3_ENABLE_PIN"
},
"E0motor": {
  "name": "Extruder 0",
  "step": "ORIG_E0_STEP_PIN",
  "dir": "ORIG_E0_DIR_PIN",
  "enable": "ORIG_E0_ENABLE_PIN"
},
"E1motor": {
  "name": "Extruder 1",
  "step": "ORIG_E1_STEP_PIN",
  "dir": "ORIG_E1_DIR_PIN",
  "enable": "ORIG_E1_ENABLE_PIN"
},
"E2motor": {
  "name": "Extruder 2",
  "step": "ORIG_E2_STEP_PIN",
  "dir": "ORIG_E2_DIR_PIN",
  "enable": "ORIG_E2_ENABLE_PIN"
},
"E3motor": {
  "name": "Extruder 3",
  "step": "ORIG_E3_STEP_PIN",
  "dir": "ORIG_E3_DIR_PIN",
  "enable": "ORIG_E3_ENABLE_PIN"
},
"E4motor": {
  "name": "Extruder 4",
  "step": "ORIG_E4_STEP_PIN",
  "dir": "ORIG_E4_DIR_PIN",
  "enable": "ORIG_E4_ENABLE_PIN"
},
"E5motor": {
  "name": "Extruder 5",
  "step": "ORIG_E5_STEP_PIN",
  "dir": "ORIG_E5_DIR_PIN",
  "enable": "ORIG_E5_ENABLE_PIN"
},
"heater0pin": "ORIG_HEATER_0_PIN",
"heater1pin": "ORIG_HEATER_1_PIN",
"heater2pin": "ORIG_HEATER_2_PIN",
"heater3pin": "ORIG_HEATER_3_PIN",
"heaterbedpin": "ORIG_HEATER_BED_PIN",
"heaterchamberpin": -1,
"coolerpin": -1,
"temp0pin": "ORIG_TEMP_0_PIN",
"temp1pin": "ORIG_TEMP_1_PIN",
"temp2pin": "ORIG_TEMP_2_PIN",
"temp3pin": "ORIG_TEMP_3_PIN",
"tempbedpin": "ORIG_TEMP_BED_PIN",
"tempchamberpin": -1,
"tempcoolerpin": -1,
"Xminpin": "ORIG_X_MIN_PIN",
"Xmaxpin": "ORIG_X_MAX_PIN",
"Yminpin": "ORIG_Y_MIN_PIN",
"Ymaxpin": "ORIG_Y_MAX_PIN",
"Zminpin": "ORIG_Z_MIN_PIN",
"Zmaxpin": "ORIG_Z_MAX_PIN",
"Z2minpin": -1,
"Z2maxpin": -1,
"Z3minpin": -1,
"Z3maxpin": -1,
"Z4minpin": -1,
"Z4maxpin": -1,
"Zprobepin": -1,
"Eminpin": -1,
"fanpin": "ORIG_FAN_PIN",
"PSONpin": "ORIG_PS_ON_PIN",
"beeperpin": -1,
"E0coolerpin": "ORIG_FAN_PIN",
"E1coolerpin": -1,
"E2coolerpin": -1,
"E3coolerpin": -1,
"filamentsensorpin": -1,
"filrunoutpin": -1,
"flowmeterpin": -1,
"laserpwrpin": 42,
"laserttlpin": 44,
"laserperipheralspin": -1,
"laserperipheralsstatuspin": -1,
"powerconsumptionpin": -1,
"doorpin": -1,
"drivesystems": 0,
"lengthyextrude": "1",
"autobed": "0",
"gridmode": "1",
"gridpoint": 2,
"Zraisebeforehoming": 10,
"Zraisebeforeprobe": 10,
"maxXYjerk": 15,
"maxEjerk": 5,
"defaultaccelleration": 2000,
"defaultretractionaccelleration": 4000,
"deltaXprobeoffset": 0,
"deltaYprobeoffset": 0,
"deltaZprobeoffset": -10,
"E0retractionspeed": 100,
"E1retractionspeed": 150,
"E2retractionspeed": 150,
"E3retractionspeed": 150,
"Xinvert": 0,
"Yinvert": "1",
"Zinvert": 0,
"E0invert": "1",
"E1invert": 0,
"E2invert": 0,
"E3invert": 0,
"filamentswitch": "0",
"pausepin": 19,
"Xservo": "-1",
"Yservo": "-1",
"angleextendservosX": 0,
"angleretractservosX": 0,
"angleextendservosY": 0,
"angleretractservosY": 0,
"uilanguages": 7
}
========== End configuration string ==========
*/

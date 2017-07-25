/**
 * MythBust3d 3D Printer Firmware
 *
 * Based on OpenSource Firmware, modified in some part from (Domenico Ponticelli) Pcelli85
 * MythBust3d Beta Testing Version modded on 26-01-2017
 * Able to run over the ANET V1.0 Original Controller of ANET A8
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 */

/**
 * Description:
 *
 * Supports platforms:
 *    ARDUINO_ARCH_SAM : For Arduino Due and other boards based on Atmel SAM3X8E
 *    ARDUINO_ARCH_AVR : For all Atmel AVR boards
 */

#ifndef _HAL_H
#define _HAL_H

/**
 * Public Variables
 */

constexpr uint32_t REFERENCE_F_CPU = 16000000; // 16MHz MEGA2560

/**
 * Timers
 */
constexpr uint32_t REFERENCE_STEPPER_TIMER_PRESCALE = 8;
constexpr double HAL_REFERENCE_STEPPER_TIMER_RATE = REFERENCE_F_CPU / REFERENCE_STEPPER_TIMER_PRESCALE; // timer1 of MEGA2560: 16000000 / 8 = 2MHz
constexpr double REFERENCE_STEPPER_TIMER_FREQUENCY = HAL_REFERENCE_STEPPER_TIMER_RATE / 2000; // note: timer0 is in mode2 (CTC), 1KHz at start

constexpr uint32_t REFERENCE_TEMP_TIMER_PRESCALE = 64;
constexpr double HAL_REFERENCE_TEMP_TIMER_RATE = REFERENCE_F_CPU / REFERENCE_TEMP_TIMER_PRESCALE; // timer0 of MEGA2560: 16000000 / 64 = 250KHz (sharing with advanced extruder)
constexpr double REFERENCE_TEMP_TIMER_FREQUENCY = HAL_REFERENCE_TEMP_TIMER_RATE / 256; // note: timer0 is in mode3 (8bit fast PWM), 976.5625Hz always

#if ENABLED(ARDUINO_ARCH_SAM)
  #include "HAL_SAM/HAL.h"
  #include "HAL_SAM/communication.h"
#elif ENABLED(ARDUINO_ARCH_AVR)
  #include "HAL_AVR/HAL.h"
  #include "HAL_AVR/communication.h"
#else
  #error "Unsupported Platform!"
#endif

#endif // _HAL_H

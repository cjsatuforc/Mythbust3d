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

#ifndef CONFIGURATION_VERSION_H
  #define CONFIGURATION_VERSION_H

  #define FIRMWARE_NAME "MythBust3d"
  #define SHORT_BUILD_VERSION "1.0.0"
  #define BUILD_VERSION FIRMWARE_NAME "_" SHORT_BUILD_VERSION
  #define STRING_DISTRIBUTION_DATE __DATE__ " " __TIME__    // build date and time
  // It might also be appropriate to define a location where additional information can be found
  #define FIRMWARE_URL  "http://nodemcu.altervista.org/a8anet/"
#endif

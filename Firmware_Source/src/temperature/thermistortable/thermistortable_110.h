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

// Pt100 with 1k0 pullup
const short temptable_110[][2] PROGMEM = {
  // only a few values are needed as the curve is very flat
  PtLine(  0, 100, 1000)
  PtLine( 50, 100, 1000)
  PtLine(100, 100, 1000)
  PtLine(150, 100, 1000)
  PtLine(200, 100, 1000)
  PtLine(250, 100, 1000)
  PtLine(300, 100, 1000)
};

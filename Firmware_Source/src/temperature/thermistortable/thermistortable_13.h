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

// Hisens thermistor B25/50 =3950 +/-1%
const short temptable_13[][2] PROGMEM = {
  {  20.04 * OVERSAMPLENR, 300 },
  {  23.19 * OVERSAMPLENR, 290 },
  {  26.71 * OVERSAMPLENR, 280 },
  {  31.23 * OVERSAMPLENR, 270 },
  {  36.52 * OVERSAMPLENR, 260 },
  {  42.75 * OVERSAMPLENR, 250 },
  {  50.68 * OVERSAMPLENR, 240 },
  {  60.22 * OVERSAMPLENR, 230 },
  {  72.03 * OVERSAMPLENR, 220 },
  {  86.84 * OVERSAMPLENR, 210 },
  { 102.79 * OVERSAMPLENR, 200 },
  { 124.46 * OVERSAMPLENR, 190 },
  { 151.02 * OVERSAMPLENR, 180 },
  { 182.86 * OVERSAMPLENR, 170 },
  { 220.72 * OVERSAMPLENR, 160 },
  { 316.96 * OVERSAMPLENR, 140 },
  { 447.17 * OVERSAMPLENR, 120 },
  { 590.61 * OVERSAMPLENR, 100 },
  { 737.31 * OVERSAMPLENR,  80 },
  { 857.77 * OVERSAMPLENR,  60 },
  { 939.52 * OVERSAMPLENR,  40 },
  { 986.03 * OVERSAMPLENR,  20 },
  { 1008.7 * OVERSAMPLENR,   0 }
};

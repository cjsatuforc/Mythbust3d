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

// mendel-parts
const short temptable_3[][2] PROGMEM = {
  {    1 * OVERSAMPLENR, 864 },
  {   21 * OVERSAMPLENR, 300 },
  {   25 * OVERSAMPLENR, 290 },
  {   29 * OVERSAMPLENR, 280 },
  {   33 * OVERSAMPLENR, 270 },
  {   39 * OVERSAMPLENR, 260 },
  {   46 * OVERSAMPLENR, 250 },
  {   54 * OVERSAMPLENR, 240 },
  {   64 * OVERSAMPLENR, 230 },
  {   75 * OVERSAMPLENR, 220 },
  {   90 * OVERSAMPLENR, 210 },
  {  107 * OVERSAMPLENR, 200 },
  {  128 * OVERSAMPLENR, 190 },
  {  154 * OVERSAMPLENR, 180 },
  {  184 * OVERSAMPLENR, 170 },
  {  221 * OVERSAMPLENR, 160 },
  {  265 * OVERSAMPLENR, 150 },
  {  316 * OVERSAMPLENR, 140 },
  {  375 * OVERSAMPLENR, 130 },
  {  441 * OVERSAMPLENR, 120 },
  {  513 * OVERSAMPLENR, 110 },
  {  588 * OVERSAMPLENR, 100 },
  {  734 * OVERSAMPLENR,  80 },
  {  856 * OVERSAMPLENR,  60 },
  {  938 * OVERSAMPLENR,  40 },
  {  986 * OVERSAMPLENR,  20 },
  { 1008 * OVERSAMPLENR,   0 },
  { 1018 * OVERSAMPLENR, -20 }
};
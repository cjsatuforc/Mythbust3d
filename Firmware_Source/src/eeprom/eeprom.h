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

#ifndef EEPROM_H
#define EEPROM_H

class EEPROM {

  public:

    static void ResetDefault();
    static void StoreSettings();

    #if DISABLED(DISABLE_M503)
      static void PrintSettings(bool forReplay = false);
    #else
      static inline void PrintSettings(bool forReplay = false) {}
    #endif

    #if ENABLED(EEPROM_SETTINGS)
      static void RetrieveSettings();
    #else
      static inline void RetrieveSettings() { ResetDefault(); PrintSettings(); }
    #endif

  private:

    static void writeData(int &pos, const uint8_t* value, uint16_t size);
    static void readData(int &pos, uint8_t* value, uint16_t size);
    static void Postprocess();

    static uint16_t eeprom_checksum;
    static const char version[6];

};

extern EEPROM eeprom;

#endif //CONFIGURATION_STORE_H

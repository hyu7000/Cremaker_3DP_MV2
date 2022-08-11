/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#include "../../inc/MarlinConfig.h"

#include "../../feature/filweight.h"
#include "../gcode.h"
#include "../parser.h"
#include "../queue.h"

// Parameter
// W : remianing filament weight value
// U : used filament weight per printing
// D : filament density

void GcodeSuite::M408() 
{
    float usedFil;
    if (parser.seenval('W')) {
        filWeight.filament_Remaining_Weight = parser.value_linear_units();
    }
    if(parser.seenval('U')){
        usedFil = parser.value_linear_units(); // unit is 'g'
        filWeight.filament_Remaining_Weight = filWeight.filament_Remaining_Weight - usedFil;
        filWeight.total_Used_Filament_Weight += (usedFil/1000); 
    }
    if(parser.seenval('D')){
        filWeight.filament_Density = parser.value_linear_units();
    }

    SERIAL_ECHOLNPAIR("Cur Fil W :", filWeight.filament_Remaining_Weight);
    SERIAL_ECHOLNPAIR("Total used Fil W :", filWeight.total_Used_Filament_Weight); // unit is 'kg'
    SERIAL_ECHOLNPAIR("Fila Density :", filWeight.filament_Density); //unity is 'g/cm^3'
}
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

#include "../inc/MarlinConfigPre.h"
#include "loadcheckfilament.h"

uint16_t LoadCheckFilament::numberOfTimesFunction = 0; 

bool LoadCheckFilament::isExtrudeTesting = false;

const feedRate_t real_fr_mm_s          = 20;
const feedRate_t real_fr_mm_s_Extruder = 5;

abce_pos_t targetExtruder = planner.get_axis_positions_mm();

void LoadCheckFilament::loadCheckFilament()
{
    planner.set_machine_position_mm(targetExtruder);

    if(numberOfTimesFunction==0)
    {
        targetExtruder[Z_AXIS] += 20; // move up by 20mm from current Z posision 

        planner.buffer_segment(targetExtruder
            #if HAS_DIST_MM_ARG
            , cart_dist_mm
            #endif
            , real_fr_mm_s, active_extruder
        );

        planner.synchronize();  
        planner.get_axis_positions_mm();
    }    

    planner.set_e_position_mm(0.0f);

    isExtrudeTesting = true;

    targetExtruder[E_AXIS] = 150; // extrude by 100mm from current E posision 

    planner.buffer_segment(targetExtruder
        #if HAS_DIST_MM_ARG
        , cart_dist_mm
        #endif
        , real_fr_mm_s_Extruder, active_extruder
     );

    planner.synchronize();
    targetExtruder = planner.get_axis_positions_mm();

    numberOfTimesFunction++;

    isExtrudeTesting = false;
    set_current_from_steppers_for_axis(ALL_AXES);

    SERIAL_ECHO("LOAD_CHECK_FINISH");
    SERIAL_EOL();
}
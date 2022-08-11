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

/*
 * Cremaker's feature
 *
 * if finished printing, remove automatically sheet and lay new sheet on Cremaker
 */

#include "../../inc/MarlinConfigPre.h"

#include "../../module/motion.h"
#include "../../module/stepper.h"
#include "../../module/endstops.h"
#include "../../module/planner.h"
#include "../../module/temperature.h"
#include "../../module/probe.h"
#include "../../feature/automaticPrinting.h"
#include "../../feature/filweight.h"

#include "../gcode.h"
#include "../queue.h"

void GcodeSuite::M2105()
{
    float weightToPrint = 0;

    if(parser.seen('P'))
    {
        if(automaticPrinting.isEnableAutoPrint)
        {
            SERIAL_ECHO("APSD"); //auto printing side
            if(automaticPrinting.rightOrLeft)
            {
                SERIAL_ECHO(" L"); //true is Left
            }
            else if(!automaticPrinting.rightOrLeft)
            {
                SERIAL_ECHO(" R"); //false is Right
            }            
        }
        else
        {
            SERIAL_ECHO("NAPSD");
        }       
        SERIAL_EOL();
    }
    else if(parser.seen('R'))
    {
        SERIAL_ECHO("APSD R");
        SERIAL_EOL();
        automaticPrinting.isEnableAutoPrint = true;
        automaticPrinting.rightOrLeft = false;
    }
    else if(parser.seen('L'))
    {
        SERIAL_ECHO("APSD L");
        SERIAL_EOL();
        automaticPrinting.isEnableAutoPrint = true;
        automaticPrinting.rightOrLeft = true;
    }
    else if(parser.seen('I'))
    {
        automaticPrinting.isEnableAutoPrint = false;
        SERIAL_ECHO("NAPSD");
        SERIAL_EOL();
    }
    else if(parser.seen_weight('W'))
    {
        weightToPrint = parser.value_float();

        if(weightToPrint > filWeight.filament_Remaining_Weight)
        {            
            SERIAL_ECHO("CANT_PRINT");
            SERIAL_EOL();
        }        
    }
    else if(parser.seen('F'))
    {
        PORT_REDIRECT(SerialMask::All);
        SERIAL_ECHO("FIRST_PRINT");
        SERIAL_EOL();
    }
    else
    {
        PORT_REDIRECT(SerialMask::All);
        SERIAL_ECHO("NEXTFILE");
        SERIAL_EOL();
    }
}
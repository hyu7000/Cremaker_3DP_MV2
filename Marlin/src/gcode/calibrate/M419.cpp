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

#include "../gcode.h"
#include "../queue.h"

//#define QUEUE_DEBUG

#ifdef QUEUE_DEBUG
  #include "../../core/serial.h"
#endif

#include "../../feature/lay1test.h"

void GcodeSuite::M419()
{
  lay1Test.numToLay1Test = 0;
  queue.is_lay1testing = false;
  
  if(parser.seen('F')){
    queue.is_printing_lay1 = false;
  }

  #ifdef QUEUE_DEBUG
    /*
    for (uint16_t i = 0; i < BUFSIZE_LAY1TEST; i++)
    {
      SERIAL_ECHO("Port_l1t: ");
      SERIAL_ECHO(queue.port_for_lay1test[i]);
      SERIAL_EOL();
    }
    
    SERIAL_ECHOLN(" ");
    

    SERIAL_ECHO("Q.length :");
    SERIAL_ECHO(queue.length);
    SERIAL_EOL();

    SERIAL_ECHOLN(" ");
    */

    for (uint16_t i = 0; i < BUFSIZE; i++)
    {
      SERIAL_ECHO("Port: ");
      SERIAL_ECHO(queue.port[i]);
      SERIAL_EOL();
    }

    SERIAL_ECHOLN(" ");
    SERIAL_ECHO("Q.length :");
    SERIAL_ECHO(queue.length);
    SERIAL_EOL();

    //PORT_REDIRECT(queue.port[queue.index_r]);

    SERIAL_ECHO("Before SEIRAL_NUM :");
    SERIAL_ECHO(serial_port_index);
    SERIAL_EOL();

    PORT_REDIRECT(SERIAL_BOTH);

    SERIAL_ECHO("After SERIAL BOTH redirect, SEIRAL_NUM :");
    SERIAL_ECHO(serial_port_index);
    SERIAL_EOL();
    
  #endif
}
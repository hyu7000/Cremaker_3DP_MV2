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

//#include "../../lcd/ultralcd.h"

void GcodeSuite::M411()
{
  if(parser.seen('F')){
    queue.is_z_home_protect = false;
  }
  else if(parser.seen('T')){
    queue.is_z_home_protect = true;
  }
}

void GcodeSuite::M418()
{
  queue.is_lay1testing = true;

  if(parser.seen('P')){
    queue.is_printing_lay1 = true;
  }
}


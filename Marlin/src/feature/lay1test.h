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

#include "../MarlinCore.h"

#include "../module/planner.h"
#include "../module/probe.h"

#include "babystep.h"

#include "../gcode/gcode.h"
#include "../gcode/queue.h"

//#include "../lcd/ultralcd.h"

#define LAY_1_TEST_START 0
#define LAY_1_TEST_END   0

#define TOTAL_LINE_NUM 54

class Lay1Test {

public:
  static uint8_t numToLay1Test;
  static uint32_t lay1EncoderPosition;
  static void lay1TestMove();  
};

extern Lay1Test lay1Test;

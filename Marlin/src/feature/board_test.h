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
#include "../module/stepper.h"
#include "../module/stepper/indirection.h"

#include "babystep.h"

#include "../gcode/gcode.h"
#include "../gcode/queue.h"

#define TEST_BOARD_AXIS_NUM 4

//#include "../lcd/ultralcd.h"

enum DirEnum : uint8_t {
    PLUS_DIR = 0, MINUS_DIR, TOTAL_DIR
};

class BoardTest {

public:
    static bool isBoardTesting;
    static bool isKnobTesting;
    static bool isWorkingAxisStepper[TEST_BOARD_AXIS_NUM];
    static bool stateOfStepper[TEST_BOARD_AXIS_NUM][2];
    static uint16_t stepperSGR_Start[TEST_BOARD_AXIS_NUM];
    static uint16_t stepperSGR_Num[TEST_BOARD_AXIS_NUM];
    static uint16_t stepperSGR_Working[TEST_BOARD_AXIS_NUM][TOTAL_DIR];

    static uint32_t encorderPos;

    static void setWorkingStepper(const AxisEnum AXIS);
    static void resetSGR();
    static void calAvgSGR(const AxisEnum AXIS,  bool DIR);
};

extern BoardTest boardTest;

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
#include "board_test.h"

bool BoardTest::isBoardTesting = false;
bool BoardTest::isKnobTesting = false;
bool BoardTest::isWorkingAxisStepper[TEST_BOARD_AXIS_NUM] = {false, false, false, false};
bool BoardTest::stateOfStepper[TEST_BOARD_AXIS_NUM][2] = {{true, true}, {true, true}, {true, true}, {true, true}};
uint16_t BoardTest::stepperSGR_Start[TEST_BOARD_AXIS_NUM];
uint16_t BoardTest::stepperSGR_Num[TEST_BOARD_AXIS_NUM] = {1, 1, 1, 1};
uint16_t BoardTest::stepperSGR_Working[TEST_BOARD_AXIS_NUM][TOTAL_DIR] = {{0,0}, {0,0}, {0,0}, {0,0}};

uint32_t BoardTest::encorderPos = 0;

void BoardTest::setWorkingStepper(const AxisEnum AXIS)
{
    for(uint16_t i = 0; i<TEST_BOARD_AXIS_NUM; i++)
    {
        if(i==AXIS)
        {
            isWorkingAxisStepper[i] = true;    
        }
        else
        {
            isWorkingAxisStepper[i] = false;
        }
    }
}

void BoardTest::resetSGR()
{
    for(uint16_t i = 0; i<TEST_BOARD_AXIS_NUM; i++)
    {
        for(uint16_t j = 0; j<TOTAL_DIR; j++)
        {
            boardTest.stepperSGR_Working[i][j] = 0;
        }
    }    
}

void BoardTest::calAvgSGR(const AxisEnum AXIS, bool DIR)
{
    uint16_t sgrValue = 0;
    uint16_t dirValue = 0;

    if(AXIS == X_AXIS)
    {
        sgrValue = stepperX.SG_RESULT();
    }
    else if(AXIS == Y_AXIS)
    {
        sgrValue = stepperY.SG_RESULT();
    }
    else if(AXIS == Z_AXIS)
    {
        sgrValue = stepperZ.SG_RESULT();
    }
    else if(AXIS == E_AXIS)
    {
        sgrValue = stepperE0.SG_RESULT();
    }

    if(DIR)
    {
        dirValue = MINUS_DIR;
    }
    else if(!DIR)
    {
        dirValue = PLUS_DIR;
    }

    stepperSGR_Working[AXIS][dirValue] = stepperSGR_Working[AXIS][dirValue] * stepperSGR_Num[AXIS];

    stepperSGR_Working[AXIS][dirValue] = stepperSGR_Working[AXIS][dirValue] + sgrValue;

    stepperSGR_Num[AXIS]++;

    stepperSGR_Working[AXIS][dirValue] = stepperSGR_Working[AXIS][dirValue] / stepperSGR_Num[AXIS];
}
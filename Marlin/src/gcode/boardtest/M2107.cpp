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

// cremaker code to test board

#include "../../inc/MarlinConfig.h"

#include "../../module/motion.h"
#include "../../module/stepper.h"
#include "../../module/endstops.h"
#include "../../module/planner.h"
#include "../../module/temperature.h"
#include "../../module/probe.h"

#include "../gcode.h"
#include "../queue.h"

#include "../../feature/tmc_util.h"
#include "../../feature/board_test.h"

abce_pos_t targetBoardTest;
sensorless_t stealth_states_BT;

bool isWorkProbe = false;
bool isWorkSensorless[3] = {false, false, false};

int16_t tempHomingThreshold;

void textStepperAxis(const AxisEnum AXIS, const float &distance)
{    
    TERN_(SENSORLESS_HOMING, stealth_states_BT = start_sensorless_homing_per_axis(AXIS));
    boardTest.stepperSGR_Num[AXIS] = 0;
    boardTest.isBoardTesting = true;

    targetBoardTest[AXIS] = 0;
    planner.set_machine_position_mm(targetBoardTest);

    targetBoardTest[AXIS] = distance;
    planner.buffer_segment(targetBoardTest
    #if HAS_DIST_MM_ARG
        ,
        cart_dist_mm
    #endif
        ,homing_feedrate(AXIS), active_extruder);

    planner.synchronize();

    if ((AXIS == X_AXIS) || (AXIS == Y_AXIS))
    {
        if (planner.get_axis_position_mm(AXIS) != distance)
        {
            isWorkSensorless[AXIS] = true;
        }
    }

    if(AXIS == Z_AXIS)
    {
        if(planner.get_axis_position_mm(Z_AXIS) != distance)
        {
            isWorkProbe = true;
        }
    }

    TERN_(SENSORLESS_HOMING, end_sensorless_homing_per_axis(AXIS, stealth_states_BT));
    boardTest.isBoardTesting = false;
}

void GcodeSuite::M2107() {
    if(parser.seen('T'))
    {      
        boardTest.resetSGR();          

        boardTest.stepperSGR_Start[X_AXIS] = 0;
        boardTest.stepperSGR_Start[Y_AXIS] = 0;
        boardTest.stepperSGR_Start[Z_AXIS] = 0;
        boardTest.stepperSGR_Start[E_AXIS] = 0;
        // SERIAL_ECHO("X : ");
        // SERIAL_ECHO(boardTest.stepperSGR_Start[X_AXIS]);
        // SERIAL_ECHO(", Y : ");
        // SERIAL_ECHO(boardTest.stepperSGR_Start[Y_AXIS]);
        // SERIAL_ECHO(", Z : ");
        // SERIAL_ECHO(boardTest.stepperSGR_Start[Z_AXIS]);
        // SERIAL_EOL();

        endstops.enable(true);
        endstops.startCremakerTest();          

        boardTest.setWorkingStepper(X_AXIS);
        textStepperAxis(X_AXIS, -100); // stepperX.dir()'s value is 'true'
        if(boardTest.stepperSGR_Start[X_AXIS] >= boardTest.stepperSGR_Working[X_AXIS][MINUS_DIR])
        {
            boardTest.stateOfStepper[X_AXIS][MINUS_DIR] = false;            
        }
        textStepperAxis(X_AXIS, 100);  // stepperX.dir()'s value is 'false'
        if(boardTest.stepperSGR_Start[X_AXIS] >= boardTest.stepperSGR_Working[X_AXIS][PLUS_DIR])
        {
            boardTest.stateOfStepper[X_AXIS][PLUS_DIR] = false;
        }

        boardTest.setWorkingStepper(Y_AXIS);
        tempHomingThreshold = stepperY.homing_threshold();
        stepperY.homing_threshold(100);
        delay(20);
        textStepperAxis(Y_AXIS, -100);
        if(boardTest.stepperSGR_Start[Y_AXIS] >= boardTest.stepperSGR_Working[Y_AXIS][MINUS_DIR])
        {
            boardTest.stateOfStepper[Y_AXIS][MINUS_DIR] = false;            
        }
        textStepperAxis(Y_AXIS, 100);  
        if(boardTest.stepperSGR_Start[Y_AXIS] >= boardTest.stepperSGR_Working[Y_AXIS][PLUS_DIR])
        {
            boardTest.stateOfStepper[Y_AXIS][PLUS_DIR] = false;            
        }
        stepperY.homing_threshold(tempHomingThreshold);

        endstops.enable_z_probe(true);
        boardTest.setWorkingStepper(Z_AXIS);
        isWorkProbe = false;
        textStepperAxis(Z_AXIS, -100);
        if(boardTest.stepperSGR_Start[Z_AXIS] >= boardTest.stepperSGR_Working[Z_AXIS][MINUS_DIR])
        {
            boardTest.stateOfStepper[Z_AXIS][MINUS_DIR] = false;            
        }
        endstops.enable_z_probe(false);

        textStepperAxis(Z_AXIS, 2);
        if(boardTest.stepperSGR_Start[Z_AXIS] >= boardTest.stepperSGR_Working[Z_AXIS][PLUS_DIR])
        {
            boardTest.stateOfStepper[Z_AXIS][PLUS_DIR] = false;            
        }
        
        boardTest.setWorkingStepper(E_AXIS);
        textStepperAxis(E_AXIS, -10);
        if(boardTest.stepperSGR_Start[E_AXIS] >= boardTest.stepperSGR_Working[E_AXIS][MINUS_DIR])
        {
            boardTest.stateOfStepper[E_AXIS][MINUS_DIR] = false;            
        }

        textStepperAxis(E_AXIS, 10);
        if(boardTest.stepperSGR_Start[E_AXIS] >= boardTest.stepperSGR_Working[E_AXIS][PLUS_DIR])
        {
            boardTest.stateOfStepper[E_AXIS][PLUS_DIR] = false;            
        }

        boardTest.setWorkingStepper(NO_AXIS);

        SERIAL_ECHO("SKRBT ");
        if(!boardTest.stateOfStepper[X_AXIS][MINUS_DIR])
        {
            SERIAL_ECHO("XMF ");
            //SERIAL_ECHO(boardTest.stepperSGR_Working[X_AXIS][MINUS_DIR]);
        }
        if(!boardTest.stateOfStepper[X_AXIS][PLUS_DIR])
        {
            SERIAL_ECHO("XPF ");
            //SERIAL_ECHO(boardTest.stepperSGR_Working[X_AXIS][PLUS_DIR]);
        }
        if(!isWorkSensorless[X_AXIS])
        {
            SERIAL_ECHO("XSNW ");
        }
        if(!boardTest.stateOfStepper[Y_AXIS][MINUS_DIR])
        {
            SERIAL_ECHO("YMF ");
            //SERIAL_ECHO(boardTest.stepperSGR_Working[Y_AXIS][MINUS_DIR]);
        }
        if(!boardTest.stateOfStepper[Y_AXIS][PLUS_DIR])
        {
            SERIAL_ECHO("YPF ");
            //SERIAL_ECHO(boardTest.stepperSGR_Working[Y_AXIS][PLUS_DIR]);
        }
        if(!isWorkSensorless[Y_AXIS])
        {
            SERIAL_ECHO("YSNW ");
        }
        if(!boardTest.stateOfStepper[Z_AXIS][MINUS_DIR])
        {
            SERIAL_ECHO("ZMF ");
            //SERIAL_ECHO(boardTest.stepperSGR_Working[Z_AXIS][MINUS_DIR]);
        }
        if(!boardTest.stateOfStepper[Z_AXIS][PLUS_DIR])
        {
            SERIAL_ECHO("ZPF ");
            //SERIAL_ECHO(boardTest.stepperSGR_Working[Z_AXIS][PLUS_DIR]);
        }
        if(!isWorkProbe)
        {
            SERIAL_ECHO("PNW ");
        }
        if(!boardTest.stateOfStepper[E_AXIS][MINUS_DIR])
        {
            SERIAL_ECHO("EMF ");
            //SERIAL_ECHO(boardTest.stepperSGR_Working[E_AXIS][MINUS_DIR]);
        }
        if(!boardTest.stateOfStepper[E_AXIS][PLUS_DIR])
        {
            SERIAL_ECHO("EPF ");
            //SERIAL_ECHO(boardTest.stepperSGR_Working[E_AXIS][PLUS_DIR]);
        }
        SERIAL_EOL();

        endstops.enable(false);
        endstops.finishCremakerTest();
    }
    else if(parser.seen('K'))
    {
        boardTest.isKnobTesting = true;
    }
    else if(parser.seen('N'))
    {
        boardTest.isKnobTesting = false;
    }
}

/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This p//rogram is free software: you can redistribute it and/or modify
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
 * cremaker Z-calibration and check timing belt function
 */
#include "../../inc/MarlinConfigPre.h"

#include "../../module/motion.h"
#include "../../module/stepper.h"
#include "../../module/endstops.h"
#include "../../module/planner.h"
#include "../../module/temperature.h"
#include "../../module/probe.h"

#include "../gcode.h"
#include "../queue.h"

#include "../../feature/tmc_util.h"

const feedRate_t real_fr_mm_s = homing_feedrate(X_AXIS);

abce_pos_t target   = planner.get_axis_positions_mm(); // 1st bound value
abce_pos_t target2B = planner.get_axis_positions_mm(); // 2nd bound value

// each axis distance to move for belt test
uint64_t xDistance     = MOVE_TEST_X_SIZE;
uint64_t yDistance     = MOVE_TEST_Y_SIZE;
uint64_t zDistance     = MOVE_TEST_Z_SIZE;
uint64_t zDistance_Bot = 40;

// start sensorless
sensorless_t stealth_states;

bool isItWithinRange(const float &Average, const AxisEnum AXIS)
{
  if (AXIS == X_AXIS)
  {
    if (Average > MOVE_TEST_X_AVG_VALUE - MOVE_TEST_GAP_VALUE)
    {
      if (Average < MOVE_TEST_X_AVG_VALUE + MOVE_TEST_GAP_VALUE)
      {
        return true;
      }
    }
  }
  else if(AXIS == Y_AXIS)
  {
    if (Average > MOVE_TEST_Y_AVG_VALUE - MOVE_TEST_GAP_VALUE)
    {
      if (Average < MOVE_TEST_Y_AVG_VALUE + MOVE_TEST_GAP_VALUE)
      {
        return true;
      }
    }
  }

  return false;
}

bool isItAbove(const float &Average, const AxisEnum AXIS)
{
  if(AXIS == X_AXIS)
  {
    if(Average > MOVE_TEST_X_AVG_VALUE + MOVE_TEST_GAP_VALUE)
    {
      return true;
    }
  }

  if(AXIS == Y_AXIS)
  {
    if(Average > MOVE_TEST_Y_AVG_VALUE + MOVE_TEST_GAP_VALUE)
    {
      return true;
    }
  }

  return false;
}

inline void home_z_safely() {
    //DEBUG_SECTION(log_G28, "home_z_safely", DEBUGGING(LEVELING));

    // Disallow Z homing if X or Y homing is needed
    if (homing_needed_error(_BV(X_AXIS) | _BV(Y_AXIS))) return;

    sync_plan_position();

    /**
     * Move the Z probe (or just the nozzle) to the safe homing point
     * (Z is already at the right height)
     */
    constexpr xy_float_t safe_homing_xy = { Z_SAFE_HOMING_X_POINT, Z_SAFE_HOMING_Y_POINT };
    destination.set(safe_homing_xy, current_position.z);

    TERN_(HOMING_Z_WITH_PROBE, destination -= probe.offset_xy);

    if (position_is_reachable(destination)) {

      //if (DEBUGGING(LEVELING)) DEBUG_POS("home_z_safely", destination);

      TERN_(SENSORLESS_HOMING, safe_delay(500)); // Short delay needed to settle

      do_blocking_move_to_xy(destination);
      //do_homing_move(Z_AXIS, 1.5f * max_length(Z_AXIS) * home_dir(Z_AXIS));
      homeaxis(Z_AXIS);
    }
    else {
      LCD_MESSAGEPGM(MSG_ZPROBE_OUT);
      SERIAL_ECHO_MSG(STR_ZPROBE_OUT_SER);
    }
  }

abce_pos_t setToMoveToDestinationForBeltTest(const AxisEnum AXIS, sensorless_t &stealth_states, abce_pos_t &target, const uint64_t &distance, const feedRate_t &real_fr_mm_s)
{
  char testStr[50];
  // before checking z axis, do homing xy
  if(AXIS==Z_AXIS)
  {
    homeaxis(X_AXIS);
    homeaxis(Y_AXIS);
    home_z_safely();
    SERIAL_ECHO(dtostrf(target[Z_AXIS], 3, 3, testStr));
    SERIAL_EOL();
  }
  else
  {
    homeaxis(AXIS);   
  }

  TERN_(SENSORLESS_HOMING, stealth_states = start_sensorless_homing_per_axis(AXIS));

  target[AXIS] = 0;
  planner.set_machine_position_mm(target);

  target[AXIS] = distance;
  planner.buffer_segment(target
  #if HAS_DIST_MM_ARG
    , cart_dist_mm
  #endif
    , real_fr_mm_s, active_extruder
  );

  planner.synchronize();
  
  TERN_(SENSORLESS_HOMING, end_sensorless_homing_per_axis(AXIS, stealth_states));

  return planner.get_axis_positions_mm(); 
}

// adjust Z level between X'rightside and X'leftside (z calibration)
void GcodeSuite::M416()
{
  endstops.enable(true);

  TERN_(SENSORLESS_HOMING, stealth_states = start_sensorless_homing_per_axis(X_AXIS));
  homeaxis(X_AXIS);  
  TERN_(SENSORLESS_HOMING, end_sensorless_homing_per_axis(X_AXIS, stealth_states));

  TERN_(SENSORLESS_HOMING, stealth_states = start_sensorless_homing_per_axis(Y_AXIS));
  homeaxis(Y_AXIS);
  TERN_(SENSORLESS_HOMING, end_sensorless_homing_per_axis(Y_AXIS, stealth_states));

  home_z_safely();

  endstops.enable(false);  

  target = planner.get_axis_positions_mm(); // 1st bound value
  
  // move to z=MOVE_TEST_Z_SIZE to finish testing
  target[Z_AXIS] = MOVE_TEST_Z_SIZE;

  planner.buffer_segment(target
  #if HAS_DIST_MM_ARG
    , cart_dist_mm
  #endif
    , real_fr_mm_s, active_extruder
  );

  planner.synchronize();
  
  planner.get_axis_positions_mm();  

  // move to z=10 to finish testing
  target[Z_AXIS] = zDistance_Bot;

  planner.buffer_segment(target
  #if HAS_DIST_MM_ARG
    , cart_dist_mm
  #endif
    , real_fr_mm_s, active_extruder
  );

  planner.synchronize();
  
  planner.get_axis_positions_mm();

  queue.is_othertesting = true;
  endstops.enable(true);

  home_z_safely();

  endstops.enable(false);
  queue.is_othertesting = false;
  
  set_current_from_steppers_for_axis(ALL_AXES);

  SERIAL_ECHO("ZHC FINISH");
  SERIAL_EOL();
}     

// check timimg belt using tmc2209
void GcodeSuite::M417()
{    
  char sendMsgTotal[40] = "belt test";
  char sendMsgX[20];
  char sendMsgY[20];

  float xAverage = 0;
  float yAverage = 0;

  bool xysenic = false;
  bool xylooseic = false;

  // enable sensorless endstop, start test
  endstops.enable(true);
  endstops.startCremakerTest();

  // move to x max for 1st belt test 
  target   = setToMoveToDestinationForBeltTest(X_AXIS,stealth_states,target,  xDistance, real_fr_mm_s);    

  // move to x max for 2nd belt test 
  target2B = setToMoveToDestinationForBeltTest(X_AXIS,stealth_states,target2B,xDistance, real_fr_mm_s);

  // move to y max for 1st belt test 
  target   = setToMoveToDestinationForBeltTest(Y_AXIS,stealth_states,target,  yDistance, real_fr_mm_s);

  // move to y max for 2nd belt test 
  target2B = setToMoveToDestinationForBeltTest(Y_AXIS,stealth_states,target2B,yDistance, real_fr_mm_s);
  
  // calculate average of each axis
  xAverage = (target[X_AXIS] + target2B[X_AXIS])/2;
  yAverage = (target[Y_AXIS] + target2B[Y_AXIS])/2;

  memcpy(sendMsgX, &target[X_AXIS], sizeof(target[X_AXIS]));
  SERIAL_ECHO("X-average1 value:");
  SERIAL_ECHO(dtostrf(target[X_AXIS], 3, 3, sendMsgX));

  memcpy(sendMsgY, &target[Y_AXIS], sizeof(target[Y_AXIS]));
  SERIAL_ECHO("   Y-average1 value:");
  SERIAL_ECHO(dtostrf(target[Y_AXIS], 3, 3, sendMsgY));
  SERIAL_EOL();

  memcpy(sendMsgX, &target2B[X_AXIS], sizeof(target2B[X_AXIS]));
  SERIAL_ECHO("X-average2 value:");
  SERIAL_ECHO(dtostrf(target2B[X_AXIS], 3, 3, sendMsgX));

  memcpy(sendMsgY, &target2B[Y_AXIS], sizeof(target2B[Y_AXIS]));
  SERIAL_ECHO("   Y-average2 value:");
  SERIAL_ECHO(dtostrf(target2B[Y_AXIS], 3, 3, sendMsgY));
  SERIAL_EOL();

  // echo distance of each axis 
  memcpy(sendMsgX, &xAverage, sizeof(xAverage));
  SERIAL_ECHO("X-average value:");
  SERIAL_ECHO(dtostrf(xAverage, 3, 3, sendMsgX));

  memcpy(sendMsgY, &yAverage, sizeof(yAverage));
  SERIAL_ECHO("   Y-average value:");
  SERIAL_ECHO(dtostrf(yAverage, 3, 3, sendMsgY));
  SERIAL_EOL();  

  // // move to z max for adjusting each z height 
  // target   = setToMoveToDestinationForBeltTest(Z_AXIS,stealth_states, target,  zDistance,     real_fr_mm_s);

  // // move to z=10 to finish testing
  // target[Z_AXIS] = zDistance_Bot;
  // planner.buffer_segment(target
  // #if HAS_DIST_MM_ARG
  //   , cart_dist_mm
  // #endif
  //   , real_fr_mm_s, active_extruder
  // );

  // planner.synchronize();
  
  // planner.get_axis_positions_mm();

  //home_z_safely();

  if(xAverage < MOVE_TEST_X_SIZE/2)
  {
    serialprintPGM(PSTR("SETPPER_SEN "));
    serialprintPGM(PSTR("X")); //incorrect sensitivity X
    xysenic = true;
  }

  if(yAverage < MOVE_TEST_Y_SIZE/2)
  {
    if(!xysenic)
      serialprintPGM(PSTR("SETPPER_SEN "));
    serialprintPGM(PSTR("Y")); //incorrect sensitivity Y    
    xysenic = true;
  }

  if(xysenic)
  {
    SERIAL_EOL(); 
    endstops.enable(false);
    endstops.finishCremakerTest();
    return;
  }

  // check stepper wiring between x and y , if loosed blet, send msg
  if(xAverage < yAverage)
  {
    serialprintPGM(PSTR("DIW XY")); //Detect Incorrect Wiring XY
    SERIAL_EOL();  
    endstops.enable(false);
    endstops.finishCremakerTest();
    return;
  }

  if(isItAbove(xAverage, X_AXIS))
  {
    serialprintPGM(PSTR("Blet loose ")); 
    serialprintPGM(PSTR("X"));           //Detect loose x blet
    serialprintPGM(PSTR(sendMsgX));
    xylooseic = true;
  }

  if(isItAbove(yAverage, Y_AXIS))
  {
    if (!xylooseic)
      serialprintPGM(PSTR("Blet loose"));
    serialprintPGM(PSTR(" Y")); //Detect loose y blet
    serialprintPGM(PSTR(sendMsgY));     
    xylooseic = true;
  }

  // if (!isItWithinRange(xAverage, X_AXIS))
  // {
  //   serialprintPGM(PSTR("Blet loose ")); //Detect loose x blet
  //   serialprintPGM(PSTR("X"));           //Detect loose x blet
  //   serialprintPGM(PSTR(sendMsgX));
  //   SERIAL_EOL();
  //   xylooseic = true;
  // }

  // if (!isItWithinRange(yAverage, Y_AXIS))
  // {
  //   if (!xylooseic)
  //     serialprintPGM(PSTR("Blet loose"));
  //   serialprintPGM(PSTR(" Y")); //Detect loose y blet
  //   serialprintPGM(PSTR(sendMsgY));
  //   SERIAL_EOL();
  //   xylooseic = true;
  // }

  if (xylooseic)
  {    
    SERIAL_EOL();
    endstops.enable(false);
    endstops.finishCremakerTest();
    return;
  }

  // echo measured value to TFT 35 
  strcat(sendMsgTotal," X");
  strcat(sendMsgTotal,sendMsgX);
  strcat(sendMsgTotal," Y");
  strcat(sendMsgTotal,sendMsgY);  
  strcat(sendMsgTotal,"\n");

  serialprintPGM(PSTR(sendMsgTotal));
  SERIAL_EOL();  

  // disable sensorless endstop, finish test
  endstops.enable(false);
  endstops.finishCremakerTest();  

  set_current_from_steppers_for_axis(ALL_AXES);
}

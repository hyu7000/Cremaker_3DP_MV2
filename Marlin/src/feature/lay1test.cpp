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
#include "lay1test.h"

uint8_t Lay1Test::numToLay1Test = 0;
uint32_t Lay1Test::lay1EncoderPosition = 0;

Lay1Test lay1Test;

static const char cmd_meander_0[] PROGMEM = "G28";
static const char cmd_meander_1[] PROGMEM = "G29";
static const char cmd_meander_2[] PROGMEM = "M83";
static const char cmd_meander_3[] PROGMEM = "G0 X40 Y40 Z0.3 F4000"; 
static const char cmd_meander_4[] PROGMEM = "G1 E1 F1000"; //start point

static const char cmd_meander_5[] PROGMEM = "G1 X60 Y40 E1 F1000"; //1st line row
static const char cmd_meander_6[] PROGMEM = "G1 X60 Y60 E1";
static const char cmd_meander_7[] PROGMEM = "G1 X80 Y60 E1";
static const char cmd_meander_8[] PROGMEM = "G1 X80 Y40 E1";
static const char cmd_meander_9[] PROGMEM = "G1 X100 Y40 E1";
static const char cmd_meander_10[] PROGMEM = "G1 X100 Y60 E1";
static const char cmd_meander_11[] PROGMEM = "G1 X120 Y60 E1";
static const char cmd_meander_12[] PROGMEM = "G1 X120 Y40 E1";
static const char cmd_meander_13[] PROGMEM = "G1 X140 Y40 E1";
static const char cmd_meander_14[] PROGMEM = "G1 X140 Y60 E1";
static const char cmd_meander_15[] PROGMEM = "G1 X160 Y60 E1";
static const char cmd_meander_16[] PROGMEM = "G1 X160 Y40 E1";
static const char cmd_meander_17[] PROGMEM = "G1 X180 Y40 E1";

static const char cmd_meander_18[] PROGMEM = "G1 X180 Y80 E2"; //2nd line row
static const char cmd_meander_19[] PROGMEM = "G1 X160 Y80 E1";
static const char cmd_meander_20[] PROGMEM = "G1 X160 Y100 E1";
static const char cmd_meander_21[] PROGMEM = "G1 X140 Y100 E1";
static const char cmd_meander_22[] PROGMEM = "G1 X140 Y80 E1";
static const char cmd_meander_23[] PROGMEM = "G1 X120 Y80 E1";
static const char cmd_meander_24[] PROGMEM = "G1 X120 Y100 E1";
static const char cmd_meander_25[] PROGMEM = "G1 X100 Y100 E1";
static const char cmd_meander_26[] PROGMEM = "G1 X100 Y80 E1";
static const char cmd_meander_27[] PROGMEM = "G1 X80 Y80 E1";
static const char cmd_meander_28[] PROGMEM = "G1 X80 Y100 E1";
static const char cmd_meander_29[] PROGMEM = "G1 X60 Y100 E1";
static const char cmd_meander_30[] PROGMEM = "G1 X60 Y80 E1";
static const char cmd_meander_31[] PROGMEM = "G1 X40 Y80 E1";

static const char cmd_meander_32[] PROGMEM = "G1 X40 Y120 E2"; //3rd line row
static const char cmd_meander_33[] PROGMEM = "G1 X60 Y120 E1";
static const char cmd_meander_34[] PROGMEM = "G1 X60 Y140 E1";
static const char cmd_meander_35[] PROGMEM = "G1 X80 Y140 E1";
static const char cmd_meander_36[] PROGMEM = "G1 X80 Y120 E1";
static const char cmd_meander_37[] PROGMEM = "G1 X100 Y120 E1";
static const char cmd_meander_38[] PROGMEM = "G1 X100 Y140 E1";
static const char cmd_meander_39[] PROGMEM = "G1 X120 Y140 E1";
static const char cmd_meander_40[] PROGMEM = "G1 X120 Y120 E1";
static const char cmd_meander_41[] PROGMEM = "G1 X140 Y120 E1";
static const char cmd_meander_42[] PROGMEM = "G1 X140 Y140 E1";
static const char cmd_meander_43[] PROGMEM = "G1 X160 Y140 E1";
static const char cmd_meander_44[] PROGMEM = "G1 X160 Y120 E1";
static const char cmd_meander_45[] PROGMEM = "G1 X180 Y120 E1";
static const char cmd_meander_46[] PROGMEM = "G1 X180 Y160 E2";

static const char cmd_meander_47[] PROGMEM = "G1 X40 Y160 E6"; // last line row
static const char cmd_meander_48[] PROGMEM = "G1 X40 Y170 E0.5";
static const char cmd_meander_49[] PROGMEM = "G1 X180 Y170 E6";
static const char cmd_meander_50[] PROGMEM = "G1 X180 Y180 E0.5";
static const char cmd_meander_51[] PROGMEM = "G1 X40 Y180 E6";
static const char cmd_meander_52[] PROGMEM = "G0 Z10 F6000";
static const char cmd_meander_53[] PROGMEM = "G28 X";
static const char cmd_meander_54[] PROGMEM = "M84"; //

static const char *const cmd_meander[] PROGMEM =
{
        cmd_meander_0, //start point
        cmd_meander_1,
        cmd_meander_2,
        cmd_meander_3,
        cmd_meander_4, 

        cmd_meander_5, //1st line row
        cmd_meander_6,
        cmd_meander_7,
        cmd_meander_8,
        cmd_meander_9,
        cmd_meander_10,
        cmd_meander_11,
        cmd_meander_12,
        cmd_meander_13,
        cmd_meander_14,
        cmd_meander_15,
        cmd_meander_16,
        cmd_meander_17, 
        
        cmd_meander_18, //2nd line row
        cmd_meander_19,
        cmd_meander_20,
        cmd_meander_21,
        cmd_meander_22,
        cmd_meander_23,
        cmd_meander_24,
        cmd_meander_25,
        cmd_meander_26,
        cmd_meander_27,
        cmd_meander_28,
        cmd_meander_29,
        cmd_meander_30,
        cmd_meander_31, 

        cmd_meander_32, //3rd line row
        cmd_meander_33,
        cmd_meander_34,
        cmd_meander_35,
        cmd_meander_36,
        cmd_meander_37,
        cmd_meander_38,
        cmd_meander_39,
        cmd_meander_40,
        cmd_meander_41,
        cmd_meander_42,
        cmd_meander_43,
        cmd_meander_44,
        cmd_meander_45,
        cmd_meander_46,

        cmd_meander_47, //last line row
        cmd_meander_48, 
        cmd_meander_49,
        cmd_meander_50,
        cmd_meander_51,
        cmd_meander_52,
        cmd_meander_53,
        cmd_meander_54,
};

void Lay1Test::lay1TestMove()
{
  if(numToLay1Test>TOTAL_LINE_NUM)
  {
    queue.is_lay1testing = false;
    return;
  }

  queue.enqueue_one_P(static_cast<char *>(pgm_read_ptr(&cmd_meander[numToLay1Test])));
  
  SERIAL_ECHO("ptr : ");
  SERIAL_ECHO(parser.command_ptr);
  SERIAL_EOL();

  numToLay1Test++;
}

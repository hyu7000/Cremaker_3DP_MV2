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

#include "../../feature/loadcheckfilament.h"

void GcodeSuite::M414()
{
    loadcheckfilament.isExtrudeTesting = false;
    loadcheckfilament.numberOfTimesFunction = 0;
}

void GcodeSuite::M415()
{
    loadcheckfilament.loadCheckFilament();
}
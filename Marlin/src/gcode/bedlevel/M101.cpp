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


#include "../../inc/MarlinConfig.h"

#include "../gcode.h"
#include "../queue.h"
#include "../../feature/bedlevel/bedlevel.h"
#include "../../module/planner.h"
#include "../../module/probe.h"
#include "../../module/motion.h"
#include "../../module/stepper.h"

#include "../../feature/probe_temp_comp.h"
#include "../../module/temperature.h"

#if ENABLED(EEPROM_SETTINGS)
  #include "../../module/settings.h"
#endif

#define CORNER_NUM 4
#define TOLERANCE_PROBING_M101 0.10

uint16_t corner_Progress_Num;
uint16_t x_index_measure_z, y_index_measure_z;

float corner_measured_z[CORNER_NUM];
float gird_xy_measurec_z[CORNER_NUM*CORNER_NUM][2];
float average_measured_z = 0;
float min_measured_z = 10;
float max_measured_z = -10;
float error_measured_z = 0;
float after_z_cal_apply_value = 0;
float get_m_value = 0;

bool isSeenI, isSeenJ, isSeenM;

xy_pos_t corner_xy[CORNER_NUM] = {{30,30}, {190,30}, {190,190}, {30,190}};
xy_pos_t grid_xy_point[CORNER_NUM*CORNER_NUM] = 
{
    {30,30},     {30,83.3},     {30, 136.6},    {30, 190}, 
    {83.3, 190}, {83.3, 136.6}, {83.3,83.3},    {83.3,30},
    {136.6,30},  {136.6,83.3},  {136.6, 136.6}, {136.6, 190},
    {190, 190},  {190, 136.6},  {190,83.3},     {190,30}                 
};

ProbePtRaise raise_after_probing = PROBE_PT_STOW;

float probingCaliAtPoint(uint16_t corner_num, xy_pos_t xy[])
{
    float temp_measured_z = probe.probe_at_point(xy[corner_num], raise_after_probing);

    #if ENABLED(PROBE_TEMP_COMPENSATION)
        temp_comp.compensate_measurement(TSI_BED, thermalManager.degBed(), temp_measured_z);
        if(temp_comp.isWinter)
        {
            temp_comp.compensate_measurement(TSI_PROBE_WINTER, thermalManager.degProbe(), temp_measured_z);
        }
        else
        {
            temp_comp.compensate_measurement(TSI_PROBE, thermalManager.degProbe(), temp_measured_z);
        }        
        TERN_(USE_TEMP_EXT_COMPENSATION, temp_comp.compensate_measurement(TSI_EXT, thermalManager.degHotend(), temp_measured_z));
    #endif

    idle_no_sleep();

    return temp_measured_z;
}

/**
 * @brief Bed coordinate
 * ※It was applied to apply z_values and added_z_value
 *             back
 *        ----------------
 *        |0,0 |1,0 |2,0 |
 *        ----------------
 *  left  |0,1 |1,1 |2,1 |  right
 *        ----------------
 *        |0,2 |1,2 |2,2 |
 *        ---------------- 
 *             front
 * 
 *   Z  check Bedlevel of 4 point, and calculate z offset average of 4 point. 
 * 
 *   H  Check specific areas in process Bedlevel
 * 
 *   T  ※disable this code
 * 
 *   I  x coordinate
 * 
 *   J  y coordinate
 * 
 *   M  Store value transmitted from the display to added_z_value
 * 
 *   R  reset all added_z_value
 * 
 *   A  adjust z_values by calculated get_m_value
 */

void GcodeSuite::M101()
{
    isSeenI = false;
    isSeenJ = false;
    isSeenM = false;

    if(parser.seen('Z'))
    {
        for(uint16_t i =0; i<CORNER_NUM; i++)
        {
            corner_measured_z[i] = probingCaliAtPoint(i, corner_xy);

            average_measured_z = average_measured_z + corner_measured_z[i];            
        }
        
        average_measured_z = average_measured_z / CORNER_NUM;
        SERIAL_ECHOLNPAIR("AVG_MEA_Z:",average_measured_z);
        SERIAL_ECHOLNPAIR("MEA_Z_LF:",corner_measured_z[0]);
        SERIAL_ECHOLNPAIR("MEA_Z_RF:",corner_measured_z[1]);
        SERIAL_ECHOLNPAIR("MEA_Z_RB:",corner_measured_z[2]);
        SERIAL_ECHOLNPAIR("MEA_Z_LB:",corner_measured_z[3]);

        for(uint16_t i =0; i<CORNER_NUM; i++)
        {
            error_measured_z = (average_measured_z - corner_measured_z[i])/average_measured_z;
            
            if(error_measured_z > TOLERANCE_PROBING_M101)
            {
                break;
            }

            if(i==(CORNER_NUM-1))
            {
                SERIAL_ECHO("TR_PASS");
                SERIAL_EOL();
            }
        }        

        average_measured_z= 0;
    }

    if(parser.seen('H'))
    {
        corner_Progress_Num = parser.value_linear_units();
        corner_measured_z[corner_Progress_Num] = probingCaliAtPoint(corner_Progress_Num, corner_xy);
        if(corner_Progress_Num==0)
        {            
            SERIAL_ECHOLNPAIR("MEA_Z_LF:",corner_measured_z[0]);
        }
        else if(corner_Progress_Num==1)
        {
            SERIAL_ECHOLNPAIR("MEA_Z_RF:",corner_measured_z[1]);
        }
        else if(corner_Progress_Num==2)
        {
            SERIAL_ECHOLNPAIR("MEA_Z_RB:",corner_measured_z[2]);
        }
        else if(corner_Progress_Num==3)
        {
            SERIAL_ECHOLNPAIR("MEA_Z_LB:",corner_measured_z[3]);
        }
    }

    if(parser.seen('T'))
    {
       for(uint16_t i = 0; i<(CORNER_NUM*CORNER_NUM); i++)
       {
          gird_xy_measurec_z[i][0] = probingCaliAtPoint(i, grid_xy_point);
          gird_xy_measurec_z[i][1] = stepper.position(Z_AXIS);
       }

       for(uint16_t i = 0; i<(CORNER_NUM*CORNER_NUM); i++)
       {
           SERIAL_ECHO(i+1);
           SERIAL_ECHOLNPAIR(" z-Offset: ", gird_xy_measurec_z[i][0], " steps: ", gird_xy_measurec_z[i][1]);           
       }
    }

    if(parser.seen('I'))
    {
        x_index_measure_z = parser.value_linear_units();
        isSeenI = true;
    }

    if(parser.seen('J'))
    {
        y_index_measure_z = parser.value_linear_units();
        isSeenJ = true;
    }

    if(parser.seen('M'))
    {
        if(isSeenI && isSeenJ)
        {
            get_m_value             = parser.value_linear_units();
            after_z_cal_apply_value = -(added_z_value[x_index_measure_z][y_index_measure_z] - get_m_value);

            added_z_value[x_index_measure_z][y_index_measure_z] = get_m_value;

            if(!isnan(added_z_value[x_index_measure_z][y_index_measure_z]))
            {
                SERIAL_ECHO("Save complete...");
                SERIAL_EOL();
            }        
        }

        isSeenM = true;
    }

    if(parser.seen('R'))
    {
        for(uint16_t j =0; j < GRID_MAX_POINTS_Y; j++)
        {
            for(uint16_t i =0; i < GRID_MAX_POINTS_X; i++)
            {
                added_z_value[i][j] = 0;
            }            
        }

        SERIAL_ECHO("added_z_value is Reset");
        SERIAL_EOL();
    }

    if(parser.seen('A'))
    {
        if(isSeenI && isSeenJ &&isSeenM)
        {
            z_values[x_index_measure_z][y_index_measure_z] = z_values[x_index_measure_z][y_index_measure_z] + after_z_cal_apply_value;
            
            isUpdated_Z_Value = true;

            extrapolate_unprobed_bed_level();
            refresh_bed_level();
            bilinear_z_offset(current_position);

            isUpdated_Z_Value = false;
        }
    }

    if(parser.seen('B'))
    {
        SERIAL_ECHOLNPAIR("Leveling active :",planner.leveling_active);        
    }

    SERIAL_ECHOLNPAIR("GMPX", GRID_MAX_POINTS_X);
    SERIAL_ECHOLNPAIR("GMPY", GRID_MAX_POINTS_Y);
    
    for (uint16_t j = 0; j < GRID_MAX_POINTS_Y; j++)
    {
        for (uint16_t i = 0; i < GRID_MAX_POINTS_X; i++)
        {
            SERIAL_ECHOLNPAIR("AD_VAL I", i, " J", j, " Z", added_z_value[i][j]);

            if (max_measured_z < added_z_value[i][j])
            {
                max_measured_z = added_z_value[i][j];
            }

            if (min_measured_z > added_z_value[i][j])
            {
                min_measured_z = added_z_value[i][j];
            }
        }
    }

    SERIAL_ECHOLNPAIR("AD_VAL MAX", max_measured_z);
    SERIAL_ECHOLNPAIR("AD_VAL MIN", min_measured_z);
}
/**
  ******************************************************************************
  * @file    quadruped_trainer_hardware.ino
  * @author  Marco, Roldan L.
  * @date    October 01, 2021
  * @brief   
  ******************************************************************************
  *
  * Copyright (C) 2021  Marco, Roldan L.
  * 
  * This program is free software: you can redistribute it and/or modify
  * it under the terms of the GNU General Public License as published by
  * the Free Software Foundation, either version 3 of the License, or
  * any later version.
  * 
  * This program is distributed in the hope that it will be useful,
  * but WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  * GNU General Public License for more details.
  * 
  * You should have received a copy of the GNU General Public License
  * along with this program.  If not, see https://www.gnu.org/licenses/gpl-3.0.en.html.
  * 
  * 
  * https://github.com/rmarco30
  * 
  ******************************************************************************
**/
#include "quadruped_trainer_hardware.h"



/**
 * @brief    Set legs' data to initial values
 * @param    fl: pointer to front left leg
 * @param    fr: pointer to front right leg
 * @param    bl: pointer to back left leg
 * @param    br: pointer to back right leg
 * @retval   none
 */
void legs_init( robot_leg_t* fl, robot_leg_t* fr, robot_leg_t* bl, robot_leg_t* br )
{
    robot_leg_t* legs[4] = { fl, fr, bl, br };
    float init_fl_data[3] = { FL_SHOULDER_ANGLE, FL_KNEE_ANGLE, FL_HEIGHT };
    float init_fr_data[3] = { FR_SHOULDER_ANGLE, FR_KNEE_ANGLE, FR_HEIGHT };
    float init_bl_data[3] = { BL_SHOULDER_ANGLE, BL_KNEE_ANGLE, BL_HEIGHT };
    float init_br_data[3] = { BR_SHOULDER_ANGLE, BR_KNEE_ANGLE, BR_HEIGHT };

    /* front left */
    legs[0]->leg_data.shoulder_a = init_fl_data[0];
    legs[0]->leg_data.knee_a = init_fl_data[1];
    legs[0]->leg_data.height = init_fl_data[2];
    legs[0]->leg_data.foot_ref = 0;
    updateLeg( legs[0], FRONT_LEFT );

    /* front right */
    legs[1]->leg_data.shoulder_a = init_fr_data[0];
    legs[1]->leg_data.knee_a = init_fr_data[1];
    legs[1]->leg_data.height = init_fr_data[2];
    legs[1]->leg_data.foot_ref = 0;
    updateLeg( legs[1], FRONT_RIGHT );

    /* back left */
    legs[2]->leg_data.shoulder_a = init_bl_data[0];
    legs[2]->leg_data.knee_a = init_bl_data[1];
    legs[2]->leg_data.height = init_bl_data[2];
    legs[2]->leg_data.foot_ref = 0;
    updateLeg( legs[2], BACK_LEFT );

    /* back right */
    legs[3]->leg_data.shoulder_a = init_br_data[0];
    legs[3]->leg_data.knee_a = init_br_data[1];
    legs[3]->leg_data.height = init_br_data[2];
    legs[3]->leg_data.foot_ref = 0;
    updateLeg( legs[3], BACK_RIGHT );
}


/**
 * @brief    Update servo motor position based on the newly calculated leg data values
 * @param    leg: pointer to robot_leg_t type to be updated
 * @retval   none
 */
void updateLeg(robot_leg_t* leg, location_t leg_location )
{
    int shoulder_us;
    int knee_us;

    switch( leg_location )
    {
        case FRONT_LEFT:
            /* convert angles to microseconds for higher resolution */
            shoulder_us = toMicroSeconds( leg->leg_data.shoulder_a );
            knee_us = toMicroSeconds( map_val( leg->leg_data.knee_a, 117, 140, 70, 15 ) );
            break;

        case FRONT_RIGHT:
            /* convert angles to microseconds for higher resolution */
            shoulder_us = toMicroSeconds( leg->leg_data.shoulder_a );
            knee_us = toMicroSeconds( map_val( leg->leg_data.knee_a, 115, 140, 105, 160 ) );
            break;

        case BACK_LEFT:
            /* convert angles to microseconds for higher resolution */
            shoulder_us = toMicroSeconds( leg->leg_data.shoulder_a );
            knee_us = toMicroSeconds( map_val( leg->leg_data.knee_a, 105, 135, 70, 143 ) );
            break;

        case BACK_RIGHT:
            /* convert angles to microseconds for higher resolution */
            shoulder_us = toMicroSeconds( leg->leg_data.shoulder_a );
            knee_us = toMicroSeconds( map_val( leg->leg_data.knee_a, 112, 140, 95, 37 ) );
            break;
        
        default:
            break;
    }

    /* update the servo positions */
    leg->shoulder.write( shoulder_us );
    leg->knee.write( knee_us );
}
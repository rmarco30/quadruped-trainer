/**
  ******************************************************************************
  * @file    quadruped_trainer_hardware.h
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
  * along with this program.
  * If not, see https://www.gnu.org/licenses/gpl-3.0.en.html.
  * 
  * 
  * https://github.com/rmarco30
  * 
  ******************************************************************************
**/

#ifndef __QUADRUPED_TRAINER_HARDWARE_H
#define __QUADRUPED_TRAINER_HARDWARE_H

#include <Arduino.h>
#include <Servo.h>
#include "quadruped_trainer_logic.h"


/* TYPEDEFS ----------------------------------------------------------------- */


/**
 * @brief    Structure to create a leg
 * @param    shoulder: servo motor in shoulder joint
 * @param    knee: servo motor in knee joint
 * @param    leg_data: holds the leg data of the leg
 */
typedef struct
{
    Servo shoulder;
    Servo knee;
    current_leg_data_t leg_data;
} robot_leg_t;


/* FUNCTION APIs ------------------------------------------------------------ */

/**
 * @brief    Set legs' data to initial values
 * @param    fl: pointer to front left leg
 * @param    fr: pointer to front right leg
 * @param    bl: pointer to back left leg
 * @param    br: pointer to back right leg
 * @retval   none
 */
void legs_init( robot_leg_t* fl, robot_leg_t* fr, robot_leg_t* bl, robot_leg_t* br );


/**
 * @brief    Update servo motor position based on the newly calculated leg data values
 * @param    leg: pointer to robot_leg_t type to be updated
 * @retval   none
 */
void updateLeg(robot_leg_t* leg, location_t leg_location );


#endif /* __QUADRUPED_TRAINER_HARDWARE_H */
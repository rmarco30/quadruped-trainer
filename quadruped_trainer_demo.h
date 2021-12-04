/**
  ******************************************************************************
  * @file    quadruped_trainer_functions.h
  * @author  Marco, Roldan L.
  * @version v1.0
  * @date    October 04, 2021
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

#ifndef __QUADRUPED_TRAINER_DEMO_H
#define __QUADRUPED_TRAINER_DEMO_H

#include "quadruped_trainer_hardware.h"


/* DEMO MOVEMENT FUNCTION APIs ---------------------------------------------- */


/**
 * @brief    Demo function that loops the leg in z-axis
 * @param    fl: pointer to front left leg
 * @param    fr: pointer to front right leg
 * @param    bl: pointer to front left leg
 * @param    br: pointer to back right leg
 * @param    del: time delay for each iteration
 * @retval   none
 * @note     this function uses hardcoded minimum values of each leg
 */
void z_translation_loop( robot_leg_t* fl, robot_leg_t* fr, robot_leg_t* bl, robot_leg_t* br, int del );


/**
 * @brief    Demo function that loops the leg in x-axis
 * @param    fl: pointer to front left leg
 * @param    fr: pointer to front right leg
 * @param    bl: pointer to front left leg
 * @param    br: pointer to back right leg
 * @param    del: time delay for each iteration
 * @retval   none
 * @note     this function uses hardcoded minimum values of each leg
 */
void x_translation_loop( robot_leg_t* fl, robot_leg_t* fr, robot_leg_t* bl, robot_leg_t* br, int del );


/**
 * @brief    Demo function that simulates stepping of the quadruped creature using
 *           purely inverse kinematics calculation.
 * @param    leg: pointer to a leg
 * @param    leg_location: location of the leg
 * @param    step_len: desired length of each step
 * @param    del: time delay for each iteration
 * @retval   none
 * @note     this function operates only on one leg at a time.
 */
void step( robot_leg_t* leg, location_t leg_location, int step_len, int del );


/**
 * @brief    Demo function that simulates walking of the quadruped creature using
 *           purely inverse kinematics calculation.
 * @param    front: pointer to front leg
 * @param    fl_loc: front leg location
 * @param    back: pointer to back leg
 * @param    bl_loc: back leg location
 * @param    step_len: desired length of each step
 * @param    del: time delay for each iteration
 * @retval   none
 * @note     This function operates on diagonally paired legs. Additionally, this function assumes that both pair of legs
 *           could achieve the same step len, but will still work if not.
 */
void walk_pair_v1( robot_leg_t* front, location_t fl_loc, robot_leg_t* back, location_t bl_loc, int step_len, int del );


/**
 * @brief    Demo function that simulates walking of the quadruped robot using
 *           purely inverse kinematics calculation.
 * @param    front: pointer to front leg
 * @param    back: pointer to back leg
 * @param    bl_loc: back leg location
 * @param    step_len: desired length of each step
 * @param    del: time delay for each iteration
 * @retval   none
 * @note     This function operates on diagonally paired legs. Additionally, 
 *           the front legs are not using inverse kinematics to move, it is hard
 *           coded to its respective values to achieve a much noticeable movement.
 *           This is done due to its tight limitations.
 */
void walk_pair_v2( robot_leg_t* front, robot_leg_t* back, location_t bl_loc, int step_len, int del );


/**
 * @brief    Demo function to simulate a forward step of front legs using hard
 *           coded values.
 * @param    leg: pointer to one of the front legs
 * @param    log_location: location of front leg
 * @retval   none
 */
void front_walk( robot_leg_t* leg, location_t leg_location );


#endif /* __QUADRUPED_TRAINER_DEMO_H */
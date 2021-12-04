/**
  ******************************************************************************
  * @file    quadruped_trainer.h
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

#ifndef __QUADRUPED_TRAINER_H
#define __QUADRUPED_TRAINER_H

#include <math.h>


/**
 *                ROBOT'S TORSO  
 * ---------------------------------------------
 *                   /| <- SHOULDER_ANGLE
 *                  /_|
 *       LEG_UPPER /  |
 *                /   |
 * KNEE_ANGLE -> /|   | HEIGHT
 *               \|   |
 *                \   |
 *       LEG_LOWER \  |
 *                  \ |
 *                   \|
 * ----------------------------------------------
 *                 GROUND
 */


/* DEFINES ------------------------------------------------------------------ */


/* Leg parameters */

#define FLEG_UPPER                ( 95.0  )       /* upper leg length in mm */
#define FLEG_LOWER                ( 110.0 )       /* lower leg length in mm */
#define BLEG_UPPER                ( 118.0 )       /* upper back leg length in mm */
#define BLEG_LOWER                ( 118.0 )       /* lower back leg length in mm */

/* Error corrections */
/* this values are used to calibrate the servo motor whereas its 90 degrees is perpendicular as possible to shoulder pivot */
#define FL_SHOULDER_ERROR         ( -5.0  )
#define FR_SHOULDER_ERROR         (  13.5 )
#define BL_SHOULDER_ERROR         ( -10.0 )
#define BR_SHOULDER_ERROR         (  5.0  )


/* Angles and height initialization */
/* Please refer to the documentation how this values are acquired */

/* front left leg values */
#define FL_SHOULDER_ANGLE         ( 61.6 + FL_SHOULDER_ERROR )
#define FL_KNEE_ANGLE             ( 127.3 )
#define FL_HEIGHT                 ( 183.84 )

/* front right leg values */
#define FR_SHOULDER_ANGLE         ( 119.1 + FR_SHOULDER_ERROR)
#define FR_KNEE_ANGLE             ( 126.1 )
#define FR_HEIGHT                 ( 182.9 )

/* back left leg values */
#define BL_SHOULDER_ANGLE         ( 120.8 + BL_SHOULDER_ERROR )
#define BL_KNEE_ANGLE             ( 118.3 )
#define BL_HEIGHT                 ( 202.63 )

/* back right leg values */
#define BR_SHOULDER_ANGLE         ( 62.2 + BR_SHOULDER_ERROR )
#define BR_KNEE_ANGLE             ( 124.3 )
#define BR_HEIGHT                 ( 208.71 )


/* Do not modify anything beyond here --------------------------------------- */

/* Calculation constants */
#define FLEG_U_SQRD               ( FLEG_UPPER * FLEG_UPPER )
#define FLEG_L_SQRD               ( FLEG_LOWER * FLEG_LOWER )
#define FLEGU_FLEGL_2             ( 2 * FLEG_UPPER * FLEG_LOWER )
#define BLEG_U_SQRD               ( BLEG_UPPER * BLEG_UPPER )
#define BLEG_L_SQRD               ( BLEG_UPPER * BLEG_UPPER )
#define BLEGU_BLEGL_2             ( 2 * BLEG_UPPER * BLEG_LOWER )


/* TYPEDEFS ----------------------------------------------------------------- */


/**
 * @brief    Enumerated structure containing the constants that represents each leg location
 * @param    members: self explanatory
 * @retval   none
 */
typedef enum
{
    FRONT_LEFT  = 0U,
    FRONT_RIGHT,
    BACK_LEFT,
    BACK_RIGHT
} location_t;


/**
 * @brief    Structure that holds the data of a leg. Call the legs_init()
 *           function to initialize this structure.
 * @param    shoulder_a: holds the current shoulder angle
 * @param    knee_a: holds the current knee_angle
 * @param    height: holds the current height
 * @param    foot_ref: holds the current foot reference
 * @note     shoulder_a holds its current servo position that is needed to form
 *           the angle inside the calculated triangle. it is:
 *           calculated_angle + 90 degrees + error
 */
typedef struct
{
    float shoulder_a;
    float knee_a;
    float height;
    int foot_ref;

} current_leg_data_t;


/* FUNCTION APIs ------------------------------------------------------------ */

/**
 * @brief    Move the leg in z-axis
 * @param    leg: pointer to robot_leg_t object type to modify
 * @param    height: desired height of the leg along the z-axis
 *           Possible values: min - max
 *           fr = 173.08 - 192.71
 *           fl = 174.97 - 192.71
 *           br = 195.65 - 221.77
 *           bl = 187.23 - 218.04
 * @retval   none
 */
void z_translation( current_leg_data_t* leg_data, float height, location_t leg_location );


/**
 * @brief    Move the leg in x-axis
 * @param    leg: pointer to robot_leg_t object type to modify
 * @param    goto_x: desired location of foot along the x-axis
 *           Possible values: max_forward to max_backward
 *           fr = -66 to  60
 *           fl = -64 to  59
 *           br =  75 to -75
 *           bl =  82 to -82
 * @retval   none
 */
void x_translation( current_leg_data_t* leg_data, int goto_x, location_t leg_location);


/**
 * @brief    Function that calculates y point of the top half of the circle
 * @param    dia: diameter of circle
 * @param    x: x value of the (x,y) coordinate in circle ranging from 0 to dia
 * @retval   y value of the (x,y) coordinate
 */
float circle_path(int dia, int x);


/**
 * @brief    Function that calculates y point of the top half of an ellipse
 * @param    a_2: length of major axis
 * @param    x: x value of the (x,y) coordinate in ellipse ranging from 0 to a_2
 * @retval   y value of the (x,y) coordinate
 */
float ellipse_path( int a_2, int x );


/**
 * @brief    Maps a range of values to another range of values.
 * @param    val: value to be mapped
 * @param    from_min: minimum value to be mapped
 * @param    from_max: maximum value to be mapped
 * @param    to_min: desired minimum value
 * @param    to_max: desired maximum value
 * @retval   mapped value
 */
long map_val( long val, long from_min, long from_max, long to_min, long to_max );


/**
 * @brief    Converts degrees to microseconds
 * @param    angle: angle in degrees
 * @retval   mapped value of degrees to microseconds
 */
int toMicroSeconds(float angle);


/**
 * @brief    Converts radians to degrees
 * @param    rad: radian value
 * @retval   converted radian value to degrees
 */
float toDegrees(float rad);


#if 1 /* If using a standard C compiler, make this value 1 to enable the code */
      /* and also its definition */
/**
 * @brief    A function that squares a number
 * @param    val: value to be squared
 * @retval   squared value
 */
double squared( double val );
#endif


#endif /* __QUADRUPED_TRAINER_H */
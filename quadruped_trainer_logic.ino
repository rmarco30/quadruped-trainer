/**
  ******************************************************************************
  * @file    quadruped_trainer_logic.ino
  * @author  Marco, Roldan L.
  * @date    October 01, 2021
  * @brief   This file contains the implementations of the quadruped robot logic
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

#include "quadruped_trainer_logic.h"


/**
 * @brief    Move the leg in z-axis
 * @param    leg: pointer to robot_leg_t object type to modify
 * @param    height: desired height of the leg along the z-axis
 *           Possible values: min - max
 *           fl = 174.97 - 192.71
 *           fr = 173.08 - 192.71
 *           bl = 187.23 - 218.04
 *           br = 195.65 - 221.77
 * @retval   none
 */
void z_translation( current_leg_data_t* leg_data, float height, location_t leg_location )
{
    float new_shoulder_angle;
    float new_knee_angle;

    switch ( leg_location )
    {
        case FRONT_LEFT:

            /* check if desired height is in range of minimum and maximum possible values */
            if( height < 174.0 )
            {
                height = 174.0;
            }
            if( height > 192.0 )
            {
                height = 192.0;
            }

            /* inverse kinematics calculation */
            new_shoulder_angle = acos( ( FLEG_U_SQRD + squared(height) - FLEG_L_SQRD ) / ( 2 * FLEG_UPPER * height ) );
            new_shoulder_angle = toDegrees( new_shoulder_angle );

            new_knee_angle = acos( ( FLEG_U_SQRD + FLEG_L_SQRD - squared(height) ) / ( FLEGU_FLEGL_2 ) );
            new_knee_angle = toDegrees( new_knee_angle );

            /* save the new leg data */
            leg_data->shoulder_a = 90.0 - new_shoulder_angle + FL_SHOULDER_ERROR;
            leg_data->knee_a = new_knee_angle;
            leg_data->height = height;

            break;
        
        case FRONT_RIGHT:

            /* check if desired height is in range of minimum and maximum possible values */
            if( height < 173.0 )
            {
                height = 173.0;
            }
            if( height > 192.0 )
            {
                height = 192.0;
            }

            /* inverse kinematics calculation */
            new_shoulder_angle = acos( ( FLEG_U_SQRD + squared(height) - FLEG_L_SQRD ) / ( 2 * FLEG_UPPER * height ) );
            new_shoulder_angle = toDegrees( new_shoulder_angle );

            new_knee_angle = acos( ( FLEG_U_SQRD + FLEG_L_SQRD - squared(height) ) / ( FLEGU_FLEGL_2 ) );
            new_knee_angle = toDegrees( new_knee_angle );

            /* save the new leg data */
            leg_data->shoulder_a = 90.0 + new_shoulder_angle + FR_SHOULDER_ERROR;
            leg_data->knee_a = new_knee_angle;
            leg_data->height = height;

            break;

        case BACK_LEFT:

            /* check if desired height is in range of minimum and maximum possible values */
            if( height < 187.0 )
            {
                height = 187.0;
            }
            if( height > 218.0 )
            {
                height = 218.0;
            }

            /* inverse kinematics calculation */
            new_shoulder_angle = acos( ( BLEG_U_SQRD + squared(height) - BLEG_L_SQRD ) / ( 2 * BLEG_UPPER * height ) );
            new_shoulder_angle = toDegrees( new_shoulder_angle );

            new_knee_angle = acos( ( BLEG_U_SQRD + BLEG_L_SQRD - squared(height) ) / ( BLEGU_BLEGL_2 ) );
            new_knee_angle = toDegrees( new_knee_angle );

            /* save the new leg data */
            leg_data->shoulder_a = 90.0 + new_shoulder_angle + BL_SHOULDER_ERROR;
            leg_data->knee_a = new_knee_angle;
            leg_data->height = height;

            break;
        
        case BACK_RIGHT:
            
            /* check if desired height is in range of minimum and maximum possible values */
            if( height < 195.0 )
            {
                height = 195.0;
            }
            if( height > 221.0 )
            {
                height = 221.0;
            }

            /* inverse kinematics calculation */
            new_shoulder_angle = acos( ( BLEG_U_SQRD + squared(height) - BLEG_L_SQRD ) / ( 2 * BLEG_UPPER * height ) );
            new_shoulder_angle = toDegrees( new_shoulder_angle );

            new_knee_angle = acos( ( BLEG_U_SQRD + BLEG_L_SQRD - squared(height) ) / ( BLEGU_BLEGL_2 ) );
            new_knee_angle = toDegrees( new_knee_angle );

            /* save the new leg data */
            leg_data->shoulder_a = 90.0 - new_shoulder_angle + BR_SHOULDER_ERROR;
            leg_data->knee_a = new_knee_angle;
            leg_data->height = height;

            break;
        
        default:
            break;
    }
}


/**
 * @brief    Move the leg in x-axis
 * @param    leg: pointer to robot_leg_t object type to modify
 * @param    goto_x: desired location of foot along the x-axis
 *           Possible values: max_forward to max_backward
 *           fl = -64 to  59
 *           fr = -66 to  60
 *           bl =  82 to -82
 *           br =  75 to -75
 * @retval   none
 */
void x_translation( current_leg_data_t* leg_data, int goto_x, location_t leg_location)
{
    float new_shoulder_angle;
    float new_knee_angle;
    float new_height;
    float theta_rad;

    switch( leg_location )
    {
        case FRONT_LEFT:

            /* check goto_x value  */
            if( leg_data->foot_ref == goto_x )
            {
                /* current foot location is already on the desired location */
                /* nothing to do here, exit immediately */
                return;
            }

            /* check x-axis travel distance */
            if( goto_x < -64 )
            {
                /* forward max limit */
                goto_x = -64;
            }
            else if( goto_x > 59 )
            {
                /* backward max limit */
                goto_x = 59;
            }

            /* inverse kinematics calculation */
            theta_rad = atan( (float)goto_x / leg_data->height );

            new_height = leg_data->height / cos( theta_rad );

            theta_rad = toDegrees( theta_rad );

            new_shoulder_angle = acos( ( FLEG_U_SQRD + squared( new_height ) - FLEG_L_SQRD ) / ( 2 * FLEG_UPPER * new_height ) );
            new_shoulder_angle = toDegrees( new_shoulder_angle );

            new_knee_angle = acos( ( FLEG_U_SQRD + FLEG_L_SQRD - squared( new_height ) ) / ( FLEGU_FLEGL_2 ) );
            new_knee_angle = toDegrees( new_knee_angle );

            /* knee servo limit */
            if( new_knee_angle > 140.0 )
            {
                new_knee_angle = 140.0;
            }

            leg_data->shoulder_a = 90.0 - ( new_shoulder_angle + theta_rad )  + FL_SHOULDER_ERROR;
            leg_data->knee_a = new_knee_angle;
            leg_data->foot_ref = goto_x;
            break;
        
        case FRONT_RIGHT:

            /* check goto_x value  */
            if( leg_data->foot_ref == goto_x )
            {
                /* current foot location is already on the desired location */
                /* nothing to do here, exit immediately */
                return;
            }

            /* check x-axis travel distance */
            if( goto_x < -66 )
            {
                /* forward max limit */
                goto_x = -66;
            }
            else if( goto_x > 60 )
            {
                /* backward max limit */
                goto_x = 60;
            }

            theta_rad = atan( (float)goto_x / leg_data->height );

            new_height = leg_data->height / cos( theta_rad );

            theta_rad = toDegrees( theta_rad );

            new_shoulder_angle = acos( ( FLEG_U_SQRD + squared( new_height ) - FLEG_L_SQRD ) / ( 2 * FLEG_UPPER * new_height ) );
            new_shoulder_angle = toDegrees( new_shoulder_angle );

            new_knee_angle = acos( ( FLEG_U_SQRD + FLEG_L_SQRD - squared( new_height ) ) / ( FLEGU_FLEGL_2 ) );
            new_knee_angle = toDegrees( new_knee_angle );

            /* knee servo limit */
            if( new_knee_angle > 140.0 )
            {
                new_knee_angle = 140.0;
            }

            leg_data->shoulder_a = 90.0 + ( new_shoulder_angle + theta_rad ) + FR_SHOULDER_ERROR;
            leg_data->knee_a = new_knee_angle;
            leg_data->foot_ref = goto_x;
            break;

        case BACK_LEFT:

            /* check goto_x value  */
            if( leg_data->foot_ref == goto_x )
            {
                /* current foot location is already on the desired location */
                /* nothing to do here, exit immediately */
                return;
            }

            /* check x-axis travel distance */
            if( goto_x > 82 )
            {
                /* forward max limit */
                goto_x = 82;
            }
            else if( goto_x < -82 )
            {
                /* backward max limit */
                goto_x = -82;
            }

            /* inverse kinematics calculation */
            theta_rad = atan( (float)goto_x / leg_data->height );

            new_height = leg_data->height / cos( theta_rad );

            theta_rad = toDegrees( theta_rad );

            new_shoulder_angle = acos( ( BLEG_U_SQRD + squared( new_height ) - BLEG_L_SQRD ) / ( 2 * BLEG_UPPER * new_height ) );
            new_shoulder_angle = toDegrees( new_shoulder_angle );

            new_knee_angle = acos( ( BLEG_U_SQRD + BLEG_L_SQRD - squared( new_height ) ) / ( BLEGU_BLEGL_2 ) );
            new_knee_angle = toDegrees( new_knee_angle );

            /* knee servo limit */
            if( new_knee_angle > 140.0 )
            {
                new_knee_angle = 140.0;
            }

            /* save the new values */
            leg_data->shoulder_a = 90.0 + ( new_shoulder_angle + theta_rad ) + BL_SHOULDER_ERROR;
            leg_data->knee_a = new_knee_angle;
            leg_data->foot_ref = goto_x;
            break;
        
        case BACK_RIGHT:

            /* check goto_x value  */
            if( leg_data->foot_ref == goto_x )
            {
                /* current foot location is already on the desired location */
                /* nothing to do here, exit immediately */
                return;
            }

            /* check x-axis travel distance */
            if( goto_x > 75 )
            {
                /* forward max limit */
                goto_x = 75;
            }
            else if( goto_x < -75 )
            {
                /* backward max limit */
                goto_x = -75;
            }

            /* inverse kinematics calculation */
            theta_rad = atan( (float)goto_x / leg_data->height );

            new_height = leg_data->height / cos( theta_rad );

            theta_rad = toDegrees( theta_rad );

            new_shoulder_angle = acos( ( BLEG_U_SQRD + squared( new_height ) - BLEG_L_SQRD ) / ( 2 * BLEG_UPPER * new_height ) );
            new_shoulder_angle = toDegrees( new_shoulder_angle );

            new_knee_angle = acos( ( BLEG_U_SQRD + BLEG_L_SQRD - squared( new_height ) ) / ( BLEGU_BLEGL_2 ) );
            new_knee_angle = toDegrees( new_knee_angle );

            /* knee servo limit */
            if( new_knee_angle > 140.0 )
            {
                new_knee_angle = 140.0;
            }

            /* save the new values */
            leg_data->shoulder_a = 90.0 - ( theta_rad + new_shoulder_angle ) + BR_SHOULDER_ERROR;
            leg_data->knee_a = new_knee_angle;
            leg_data->foot_ref = goto_x;
            break;

        default:
            break;
    }
}


/**
 * @brief    Function that calculates y point of the top half of the circle
 * @param    dia: diameter of circle
 * @param    x: x value of the (x,y) coordinate in circle ranging from 0 to dia
 * @retval   y value of the (x,y) coordinate
 */
float circle_path(int dia, int x)
{
    float rad = dia / 2.0;
    float y;

    y = sqrt( pow( rad, 2 ) - pow( ((float)x - rad), 2) );

    return y;
}


/**
 * @brief    Function that calculates y point of the top half of an ellipse
 * @param    a_2: length of major axis
 * @param    x: x value of the (x,y) coordinate in ellipse ranging from 0 to a_2
 * @retval   y value of the (x,y) coordinate
 */
float ellipse_path( int a_2, int x )
{
    float a = (float)a_2 / 2.0;
    float h = a;
    float b = a / 2.0;
    float y;
    
    y = sqrt( ( 1.0 - ( pow( ( (float)x - h ), 2) ) / pow(a, 2) ) * pow(b, 2) );

    return y;
}


/**
 * @brief    Maps a range of values to another range of values.
 * @param    val: value to be mapped
 * @param    from_min: minimum value to be mapped
 * @param    from_max: maximum value to be mapped
 * @param    to_min: desired minimum value
 * @param    to_max: desired maximum value
 * @retval   mapped value
 */
long map_val( long val, long from_min, long from_max, long to_min, long to_max )
{
    return ( (long)(round( val )) - from_min ) * ( to_max - to_min ) / ( from_max - from_min ) + to_min;
}


/**
 * @brief    Converts degrees to microseconds
 * @param    angle: angle in degrees
 * @retval   mapped value of degrees to microseconds
 */
int toMicroSeconds( float angle )
{
    return map_val( angle , 0, 180, 500, 2500 );
}


/**
 * @brief    Converts radians to degrees
 * @param    rad: radian value
 * @retval   converted radian value to degrees
 */
float toDegrees( float rad )
{
    return ( rad * (180.0 / 3.1415926535897932384626433832795 ) );
}


#if 1
/**
 * @brief    A function that squares a number
 * @param    val: value to be squared
 * @retval   squared value
 */
double squared( double val )
{
    return ( val * val );
}
#endif
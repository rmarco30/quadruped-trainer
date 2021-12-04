/**
  ******************************************************************************
  * @file    quadruped_trainer_functions.ino
  * @author  Marco, Roldan L.
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

#include "quadruped_trainer_demo.h"
#include <Arduino.h>


/* DEMO MOVEMENT FUNCTION DEFINITIONS --------------------------------------- */


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
void z_translation_loop( robot_leg_t* fl, robot_leg_t* fr, robot_leg_t* bl, robot_leg_t* br, int del )
{
    /* this min and max values will not be reached by all the legs 
       it is used so that the legs will appear moving at the same time */
    float max_height = 221.0;
    float min_height = 173.0;

    for(float i = min_height; i <= max_height; i++)
    {
        z_translation( &(*fl).leg_data, i, FRONT_LEFT );
        z_translation( &(*fr).leg_data, i, FRONT_RIGHT );
        z_translation( &(*bl).leg_data, i, BACK_LEFT );
        z_translation( &(*br).leg_data, i, BACK_RIGHT );

        updateLeg( &(*fl), FRONT_LEFT );
        updateLeg( &(*fr), FRONT_RIGHT );
        updateLeg( &(*bl), BACK_LEFT );
        updateLeg( &(*br), BACK_RIGHT );
        delay( del );
    }
    delay( 500 );

    for(float i = max_height; i >= min_height; i--)
    {
        z_translation( &(*fl).leg_data, i, FRONT_LEFT );
        z_translation( &(*fr).leg_data, i, FRONT_RIGHT );
        z_translation( &(*bl).leg_data, i, BACK_LEFT );
        z_translation( &(*br).leg_data, i, BACK_RIGHT );

        updateLeg( &(*fl), FRONT_LEFT );
        updateLeg( &(*fr), FRONT_RIGHT );
        updateLeg( &(*bl), BACK_LEFT );
        updateLeg( &(*br), BACK_RIGHT );
        delay( del );
    }
    delay( 500 );
}


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
void x_translation_loop( robot_leg_t* fl, robot_leg_t* fr, robot_leg_t* bl, robot_leg_t* br, int del )
{
    /* this min and max values will not be reached by all the legs 
       it is used so that the legs will appear moving at the same time */
    int max_forward  = 82;
    int max_backward = -59;

    updateLeg( &(*fl), FRONT_LEFT );
    updateLeg( &(*fr), FRONT_RIGHT );
    updateLeg( &(*bl), BACK_LEFT );
    updateLeg( &(*br), BACK_RIGHT );


    for( int i = 0; i <= max_forward; i++ )
    {
        x_translation( &(*fl).leg_data, ( i * (-1) ), FRONT_LEFT );
        x_translation( &(*fr).leg_data, ( i * (-1) ), FRONT_RIGHT );
        x_translation( &(*bl).leg_data, i, BACK_LEFT );
        x_translation( &(*br).leg_data, i, BACK_RIGHT );

        updateLeg( &(*fl), FRONT_LEFT );
        updateLeg( &(*fr), FRONT_RIGHT );
        updateLeg( &(*bl), BACK_LEFT );
        updateLeg( &(*br), BACK_RIGHT );
        delay( del );
    }
    delay(500);

    for( int i = max_forward; i >= 0; i-- )
    {
        x_translation( &(*fl).leg_data, ( i * (-1) ), FRONT_LEFT );
        x_translation( &(*fr).leg_data, ( i * (-1) ), FRONT_RIGHT );
        x_translation( &(*bl).leg_data, i, BACK_LEFT );
        x_translation( &(*br).leg_data, i, BACK_RIGHT );

        updateLeg( &(*fl), FRONT_LEFT );
        updateLeg( &(*fr), FRONT_RIGHT );
        updateLeg( &(*bl), BACK_LEFT );
        updateLeg( &(*br), BACK_RIGHT );
        delay( del );
    }
    delay(500);

    for( int i = 0; i >= max_backward - 1; i-- )
    {
        x_translation( &(*fl).leg_data, ( i * (-1) ), FRONT_LEFT );
        x_translation( &(*fr).leg_data, ( i * (-1) ), FRONT_RIGHT );
        x_translation( &(*bl).leg_data, i, BACK_LEFT );
        x_translation( &(*br).leg_data, i, BACK_RIGHT );

        updateLeg( &(*fl), FRONT_LEFT );
        updateLeg( &(*fr), FRONT_RIGHT );
        updateLeg( &(*bl), BACK_LEFT );
        updateLeg( &(*br), BACK_RIGHT );
        delay( del );
    }
    delay(500);

    for( int i = max_backward; i <= 0; i++)
    {
        x_translation( &(*fl).leg_data, ( i * (-1) ), FRONT_LEFT );
        x_translation( &(*fr).leg_data, ( i * (-1) ), FRONT_RIGHT );
        x_translation( &(*bl).leg_data, i, BACK_LEFT );
        x_translation( &(*br).leg_data, i, BACK_RIGHT );

        updateLeg( &(*fl), FRONT_LEFT );
        updateLeg( &(*fr), FRONT_RIGHT );
        updateLeg( &(*bl), BACK_LEFT );
        updateLeg( &(*br), BACK_RIGHT );
        delay( del );
    }
    delay(500);
}


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
void step( robot_leg_t* leg, location_t leg_location, int step_len, int del )
{
    float z;
    float initial_height;
    int initial_foot_ref;

    switch( leg_location )
    {
        case FRONT_LEFT:
            initial_height = FL_HEIGHT;
            initial_foot_ref = leg->leg_data.foot_ref;
            step_len = step_len * (-1);
            break;
        
        case FRONT_RIGHT:
            initial_height = FR_HEIGHT;
            initial_foot_ref = leg->leg_data.foot_ref;
            step_len = step_len * (-1);
            break;
        
        case BACK_LEFT:
            initial_height = BL_HEIGHT;
            initial_foot_ref = leg->leg_data.foot_ref;
            break;
        
        case BACK_RIGHT:
            initial_height = BR_HEIGHT;
            initial_foot_ref = leg->leg_data.foot_ref;
            break;
        
        default:
            break;
    }

    /* check the step direction */
    if( step_len < 0 )
    {
        step_len = step_len * (-1);
        /* move foot to desired trajectory */
        for( int i = 0; i < step_len; i++ )
        {
            /* simulate walking following a semi circular path */
            // z = circle_path(step_len, i);

            /* simulate walking following a semi ellipse path */
            z = ellipse_path(step_len, i);

            z_translation( &(*leg).leg_data, ( initial_height - z ), leg_location );
            x_translation( &(*leg).leg_data, ( i * (-1) ), leg_location );
            updateLeg( &(*leg), leg_location );
            delay( del + 2 );
        }

        /* pull back the foot to initial position */
        for( int i = ( step_len * -1 ); i <= 0; i++ )
        {
            x_translation( &(*leg).leg_data, i, leg_location );
            updateLeg( &(*leg), leg_location );
            delay( del );
        }
    }
    else
    {
        /* move foot to desired trajectory */
        for( int i = 0; i <= step_len; i++ )
        {
            /* simulate walking following a semi circular path */
            // z = circle_path(step_len, i);

            /* simulate walking following a semi ellipse path */
            z = ellipse_path(step_len, i);

            z_translation( &(*leg).leg_data, ( initial_height - z ), leg_location );
            x_translation( &(*leg).leg_data, i, leg_location );
            updateLeg( &(*leg), leg_location );
            delay( del + 2 );
        }

        /* pull back the foot to initial position */
        for( int i = step_len; i >= 0; i-- )
        {
            x_translation( &(*leg).leg_data, i, leg_location );
            updateLeg( &(*leg), leg_location );
            delay( del );
        }
    }
}


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
void walk_pair_v1( robot_leg_t* front, location_t fl_loc, robot_leg_t* back, location_t bl_loc, int step_len, int del )
{
    float z;
    float fl_initial_height;
    float bl_initial_height;

    switch( fl_loc )
    {
        case FRONT_LEFT:
            fl_initial_height = FL_HEIGHT;
            // fl_step_len = fl_step_len * (-1);
            break;
        
        case FRONT_RIGHT:
            fl_initial_height = FR_HEIGHT;
            // fl_step_len = fl_step_len * (-1);
            break;
        
        default:
            break;
    }

    switch( bl_loc )
    {
        case BACK_LEFT:
            bl_initial_height = BL_HEIGHT;
            break;
        
        case BACK_RIGHT:
            bl_initial_height = BR_HEIGHT;
            break;
        
        default:
            break;
    }

    /* check step direction */
    if( step_len < 0 )
    {
        step_len = step_len * (-1);
        /* move foot to desired trajectory */
        for( int i = 0; i < step_len; i++ )
        {
            /* simulate walking following a semi circular path */
            // z = circle_path(step_len, i);

            /* simulate walking following a semi ellipse path */
            z = ellipse_path(step_len, i);

            z_translation( &(*back).leg_data, ( bl_initial_height - z ), bl_loc );
            x_translation( &(*back).leg_data, ( i * (-1) ), bl_loc );

            z_translation( &(*front).leg_data, ( fl_initial_height - z ), fl_loc );
            x_translation( &(*front).leg_data, i, fl_loc );

            updateLeg( &(*back), bl_loc );
            updateLeg( &(*front), fl_loc );
            delay( del + 2 );
        }

        /* pull back the foot to initial position */
        for( int i = step_len; i >= 0; i-- )
        {
            x_translation( &(*back).leg_data, ( 0 - i ), bl_loc );
            x_translation( &(*front).leg_data, i, fl_loc );

            updateLeg( &(*back), bl_loc );
            updateLeg( &(*front), fl_loc );
            delay( del );
        }
    }
    else
    {
        /* move foot to desired trajectory */
        for( int i = 0; i < step_len; i++ )
        {
            /* simulate walking following a semi circular path */
            // z = circle_path(step_len, i);

            /* simulate walking following a semi ellipse path */
            z = ellipse_path(step_len, i);

            z_translation( &(*front).leg_data, ( fl_initial_height - z ), fl_loc );
            x_translation( &(*front).leg_data, ( i * (-1) ), fl_loc );

            z_translation( &(*back).leg_data, ( bl_initial_height - z ), bl_loc );
            x_translation( &(*back).leg_data, i, bl_loc );

            updateLeg( &(*front), fl_loc );
            updateLeg( &(*back), bl_loc );
            delay( del + 2 );
        }

        /* pull back the foot to initial position */
        for( int i = step_len; i >= 0; i-- )
        {
            x_translation( &(*front).leg_data, ( 0 - i ), fl_loc );
            x_translation( &(*back).leg_data, i, bl_loc );

            updateLeg( &(*front), fl_loc );
            updateLeg( &(*back), bl_loc );
            delay( del );
        }
    }
}


/**
 * @brief    Demo function that simulates walking of the quadruped creature using
 *           purely inverse kinematics for back legs, and hard coded values for
 *           front legs.
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
void walk_pair_v2( robot_leg_t* front, robot_leg_t* back, location_t bl_loc, int step_len, int del )
{
    float z;
    int back_step = 0;

    if( bl_loc == BACK_RIGHT )
    {
        for( float i = 192; i >= 174.0; i-- )
        {
            z = ellipse_path( step_len, back_step );
            z_translation( &(*back).leg_data, ( BR_HEIGHT - z ), BACK_RIGHT );
            x_translation( &(*back).leg_data, back_step, BACK_RIGHT );

            z_translation( &(*front).leg_data, i, FRONT_LEFT );

            updateLeg( &(*back), BACK_RIGHT );
            updateLeg( &(*front), FRONT_LEFT );
            back_step += 1;
            delay( del + 2 );
        }

        for( int i = 50; i <= 80; i++ )
        {   
            z = ellipse_path( step_len, back_step );
            z_translation( &(*back).leg_data, ( BR_HEIGHT - z ), BACK_RIGHT );
            x_translation( &(*back).leg_data, back_step, BACK_RIGHT );
            updateLeg( &(*back), BACK_RIGHT );
            front->shoulder.write( i );

            back_step += 1;
            delay( del + 2 );
        }

        for( int i = 70; i >= 15; i-- )
        {
            if( back_step < 75 )
            {
                z = ellipse_path( step_len, back_step );
                z_translation( &(*back).leg_data, ( BR_HEIGHT - z ), BACK_RIGHT );
                x_translation( &(*back).leg_data, back_step, BACK_RIGHT );
                back_step += 1;
            }
            updateLeg( &(*back), BACK_RIGHT );
            front->knee.write( i );
            delay( del + 2 );
        }

        for( int i = 80; i >= 63; i-- )
        {
            x_translation( &(*back).leg_data, back_step, BACK_RIGHT );
            updateLeg( &(*back), BACK_RIGHT );
            front->shoulder.write( i );
            back_step -= 2;
            delay( del + 2 );
        }

        for( int i = back_step; i >= 0; i-- )
        {
            x_translation( &(*back).leg_data, back_step, BACK_RIGHT );
            updateLeg( &(*back), BACK_RIGHT );
            back_step -= 1;
            delay( 2 );
        }
    }
    else
    {
        for( float i = 192; i >= 173.0; i-- )
        {
            z = ellipse_path( step_len, back_step );
            z_translation( &(*back).leg_data, ( BL_HEIGHT - z ), BACK_LEFT );
            x_translation( &(*back).leg_data, back_step, BACK_LEFT );

            z_translation( &(*front).leg_data, i, FRONT_RIGHT );

            updateLeg( &(*back), BACK_LEFT );
            updateLeg( &(*front), FRONT_RIGHT );
            back_step += 1;
            delay( del + 2 );
        }
        /* 20 */

        for( int i = 138; i >= 108; i-- )
        {   
            z = ellipse_path( step_len, back_step );
            z_translation( &(*back).leg_data, ( BL_HEIGHT - z ), BACK_LEFT );
            x_translation( &(*back).leg_data, back_step, BACK_LEFT );
            updateLeg( &(*back), BACK_LEFT );
            front->shoulder.write( i );

            back_step += 1;
            delay( del + 2 );
        }

        for( int i = 105; i <= 160; i++ )
        {
            if( back_step < 82 )
            {
                z = ellipse_path( step_len, back_step );
                z_translation( &(*back).leg_data, ( BL_HEIGHT - z ), BACK_LEFT );
                x_translation( &(*back).leg_data, back_step, BACK_LEFT );
                back_step += 1;
            }
            updateLeg( &(*back), BACK_LEFT );
            front->knee.write( i );
            delay( del + 2 );
        }

        for( int i = 108; i <= 125; i++ )
        {
            x_translation( &(*back).leg_data, back_step, BACK_LEFT );
            updateLeg( &(*back), BACK_LEFT );
            front->shoulder.write( i );
            back_step -= 2;
            delay( del + 2 );
        }

        for( int i = back_step; i >= 0; i-- )
        {
            x_translation( &(*back).leg_data, back_step, BACK_LEFT );
            updateLeg( &(*back), BACK_LEFT );
            back_step -= 1;
            delay( 2 );
        }
    }
}


/**
 * @brief    Demo function to simulate a forward step of front legs using hard
 *           coded values.
 * @param    leg: pointer to one of the front legs
 * @param    log_location: location of front leg
 * @retval   none
 */
void front_walk( robot_leg_t* leg, location_t leg_location )
{
    int delay_k = 5;
    int delay_s = 15;
    int delay_all = 5;

    switch( leg_location )
    {
        case FRONT_LEFT:

            for( float i = 192.0; i >= 174.0; i-- )
            {
                z_translation( &(*leg).leg_data, i, FRONT_LEFT );
                updateLeg( &(*leg), FRONT_LEFT );
                delay( 20 );
            }
            delay( delay_all );

            for( int i = 50; i <= 80; i++ )
            {
                leg->shoulder.write( i );
                delay( delay_s );
            }
            delay( delay_all );

            for( int i = 70; i >= 15; i-- )
            {
                leg->knee.write( i );
                delay( delay_k );
            }
            delay( delay_all );

            for( int i = 80; i >= 63; i-- )
            {
                leg->shoulder.write( i );
                delay( delay_s );
            }
            delay( delay_all );
            break;
        
        case FRONT_RIGHT:
            
            for( float i = 192.0; i >= 173.0; i-- )
            {
                z_translation( &(*leg).leg_data, i, FRONT_RIGHT );
                updateLeg( &(*leg), FRONT_RIGHT );
                delay( 20 );
            }
            delay( delay_all );

            for( int i = 138; i >= 108; i-- )
            {
                leg->shoulder.write( i );
                delay( delay_s );
            }
            delay( delay_all );

            for( int i = 105; i <= 160; i++ )
            {
                leg->knee.write( i );
                delay( delay_k );
            }
            delay( delay_all );

            for( int i = 108; i <= 125; i++ )
            {
                leg->shoulder.write( i );
                delay( delay_s );
            }
            delay( delay_all );
            break;
        
        default:
            break;
    }
}
/**
  ******************************************************************************
  * @file    quadruped_trainer_main.ino
  * @author  Marco, Roldan L.
  * @date    October 01, 2021
  * @brief   Sample main file of quadruped robot
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
#include "quadruped_trainer_hardware.h"
#include "quadruped_trainer_demo.h"

/* create a leg object */
robot_leg_t frontLeft;
robot_leg_t frontRight;
robot_leg_t backLeft;
robot_leg_t backRight;

void initialize_for( int leg_function );

void setup()
{
    /* Serial monitor for debugging */
    Serial.begin(9600);

    /* attach servo motors to gpios */
    frontLeft.shoulder.attach( 13 );
    frontLeft.knee.attach( 12 );

    frontRight.shoulder.attach( 8 );
    frontRight.knee.attach( 9 );

    backLeft.shoulder.attach( 2 );
    backLeft.knee.attach( 3 );

    backRight.shoulder.attach( 7 );
    backRight.knee.attach( 6 );

    /* set legs' data to initial values */
    legs_init( &frontLeft, &frontRight, &backLeft, &backRight );
    delay(2000);
}


void loop()
{
    /* Loop the legs 5 times in Z-axis to its maximum and minimum height */
    /* Change this to 1 to include this code */
    #if 1
    initialize_for('z');
    for( int i = 0; i < 5; i++ )
    {
        z_translation_loop( &frontLeft, &frontRight, &backLeft, &backRight, 5 );
    }
    delay(500);
    #endif



    /* Loop the legs 5 times in X-axis to its maximum and minimum distance */
    /* Change this to 1 to include this code */
    #if 1
    initialize_for('x');
    for( int i = 0; i < 5; i++ )
    {
        x_translation_loop( &frontLeft, &frontRight, &backLeft, &backRight, 5 );
    }
    delay(500);
    #endif



    /* Simulate stepping 5 times for back legs */
    /* Change this to 1 to include this code */
    #if 0
    initialize_for('w1');
    for( int i = 0; i < 5; i++ )
    {
        step( &backLeft, BACK_LEFT, 82, 5 );
        delay(500);
        step( &backRight, BACK_RIGHT, 75, 5 );
        delay(500);
    }
    delay(500);
    #endif



    /* Simulate stepping 5 times for front legs (inverse-kinematics) */
    /* Change this to 1 to include this code */
    #if 0
    initialize_for('w1');
    for( int i = 0; i < 5; i++ )
    {
        step( &frontLeft, FRONT_LEFT, 82, 5 );
        delay(500);
        step( &frontRight, FRONT_RIGHT, 75, 5 );
        delay(500);
    }
    delay(500);
    #endif



    /* Simulate stepping 5 times for front legs (hardcoded) */
    /* Change this to 1 to include this code */
    #if 0
    initialize_for('w2');
    delay(500);
    for( int i = 0; i < 5; i++ )
    {
        front_walk( &frontLeft, FRONT_LEFT );
        delay(500);
        front_walk( &frontRight, FRONT_RIGHT );
        delay(500);
    }
    delay(500);
    #endif



    /* Simulate walking 5 times using inverse kinematics */
    /* Change this to 1 to include this code */
    #if 0
    initialize_for('w1');
    for( int i = 0; i < 5; i++ )
    {
        walk_pair_v1( &frontLeft, FRONT_LEFT, &backRight, BACK_RIGHT, 75, 5 );
        delay(200);
        walk_pair_v1( &frontRight, FRONT_RIGHT, &backLeft, BACK_LEFT, 82, 5 );
        delay(200);
    }
    delay(500);
    #endif



    /* Simulate walking 5 times using hardcoded approach */
    /* Change this to 1 to include this code */
    #if 1
    int speed = 5;
    initialize_for('w2');
    for( int i = 1; i <= 15; i++ )
    {
        walk_pair_v2( &frontLeft, &backRight, BACK_RIGHT, 75, speed );
        walk_pair_v2( &frontRight, &backLeft, BACK_LEFT, 82, speed );
        
        if( (i % 5) == 0 )
        {
            /* decrease the delay to speed up the loop */
            speed = speed - 1;
        }
    }
    #endif



    /* Reset the legs to its default position */
    /* Change this to 1 to include this code */
    #if 0
    legs_init( &frontLeft, &frontRight, &backLeft, &backRight );
    #endif

    delay(2000);
}



/**
 * @brief    Function to set refresh legs' data to values where it could
 *           maximize its full range of motion.
 * @param    leg_function: 'z' for z-translation, 'x' for x-translation
 *           'w' for walk.
 * @retval   none
 * @note     This function is not really important. This is for demonstration only.
 */
void initialize_for( int leg_function )
{
    switch( leg_function )
    {
        case 'z':
          /* initialization to set the quadruped to its minimum height to achive
             full-range of motion in z-translation */
          frontLeft.leg_data.shoulder_a = 55.9 + FL_SHOULDER_ERROR;
          frontLeft.leg_data.knee_a = 117.0;
          frontLeft.leg_data.height = 174.97;
          frontLeft.leg_data.foot_ref = 0;

          frontRight.leg_data.shoulder_a = 125.2 + FR_SHOULDER_ERROR;
          frontRight.leg_data.knee_a = 115.0;
          frontRight.leg_data.height = 173.08;
          frontRight.leg_data.foot_ref = 0;

          backLeft.leg_data.shoulder_a = 127.5 + BL_SHOULDER_ERROR;
          backLeft.leg_data.knee_a = 105.0;
          backLeft.leg_data.height = 187.23;
          backLeft.leg_data.foot_ref = 0;

          backRight.leg_data.shoulder_a = 56.0 + BR_SHOULDER_ERROR;
          backRight.leg_data.knee_a = 112.0;
          backRight.leg_data.height = 195.65;
          backRight.leg_data.foot_ref = 0;
          break;
        
        case 'x':
          /* initialization to set quadruped between its minimum and maximum
             height to achieve full-range of motion in x-translation */
          frontLeft.leg_data.shoulder_a = 55.9 + FL_SHOULDER_ERROR;
          frontLeft.leg_data.knee_a = 117.0;
          frontLeft.leg_data.height = 174.97;
          frontLeft.leg_data.foot_ref = 0;

          frontRight.leg_data.shoulder_a = 125.2 + FR_SHOULDER_ERROR;
          frontRight.leg_data.knee_a = 115.0;
          frontRight.leg_data.height = 173.08;
          frontRight.leg_data.foot_ref = 0;

          backLeft.leg_data.shoulder_a = 120.8 + BL_SHOULDER_ERROR;
          backLeft.leg_data.knee_a = 118.3;
          backLeft.leg_data.height = 202.63;
          backLeft.leg_data.foot_ref = 0;

          backRight.leg_data.shoulder_a = 62.2 + BR_SHOULDER_ERROR;
          backRight.leg_data.knee_a = 124.3;
          backRight.leg_data.height = 208.71;
          backRight.leg_data.foot_ref = 0;
          break;

        case 'w1':
          /* initialization for walking using pure math calculation */
          frontLeft.leg_data.shoulder_a = 61.6 + FL_SHOULDER_ERROR;
          frontLeft.leg_data.knee_a = 127.3;
          frontLeft.leg_data.height = 183.84;
          frontLeft.leg_data.foot_ref = 0;

          frontRight.leg_data.shoulder_a = 119.1 + FR_SHOULDER_ERROR;
          frontRight.leg_data.knee_a = 126.1;
          frontRight.leg_data.height = 182.9;
          frontRight.leg_data.foot_ref = 0;

          backLeft.leg_data.shoulder_a = BL_SHOULDER_ANGLE;
          backLeft.leg_data.knee_a = BL_KNEE_ANGLE;
          backLeft.leg_data.height = BL_HEIGHT;
          backLeft.leg_data.foot_ref = 0;

          backRight.leg_data.shoulder_a = BR_SHOULDER_ANGLE;
          backRight.leg_data.knee_a = BR_KNEE_ANGLE;
          backRight.leg_data.height = BR_HEIGHT;
          backRight.leg_data.foot_ref = 0;
          break;
        
        case 'w2':
          /* initialization for walking using hard coded approach */
          frontLeft.leg_data.shoulder_a = 68.5 + FL_SHOULDER_ERROR;
          frontLeft.leg_data.knee_a = 140.0;
          frontLeft.leg_data.height = 192.71;
          frontLeft.leg_data.foot_ref = 0;

          frontRight.leg_data.shoulder_a = 111.5 + FR_SHOULDER_ERROR;
          frontRight.leg_data.knee_a = 140.0;
          frontRight.leg_data.height = 192.71;
          frontRight.leg_data.foot_ref = 0;

          backLeft.leg_data.shoulder_a = BL_SHOULDER_ANGLE;
          backLeft.leg_data.knee_a = BL_KNEE_ANGLE;
          backLeft.leg_data.height = BL_HEIGHT;
          backLeft.leg_data.foot_ref = 0;

          backRight.leg_data.shoulder_a = BR_SHOULDER_ANGLE;
          backRight.leg_data.knee_a = BR_KNEE_ANGLE;
          backRight.leg_data.height = BR_HEIGHT;
          backRight.leg_data.foot_ref = 0;
          break;
        
        default:
          break;
    }
}
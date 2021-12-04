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
    /* Simulate walking 5 times using inverse kinematics */
    /* Change this to 1 to include this code */

    for( int i = 0; i < 5; i++ )
    {
        walk_pair( &frontLeft, FRONT_LEFT, &backRight, BACK_RIGHT, 75, 5 );
        delay(200);
        walk_pair( &frontRight, FRONT_RIGHT, &backLeft, BACK_LEFT, 82, 5 );
        delay(200);
    }
    delay(500);
}
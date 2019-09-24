/*
 * Volbot2D_simulator.cpp
 * This is the program with the main function in it
 * It will do the actual calculations for each object per cycle and update the velocity and
 * position of each object. The velocity will be used internally and applied to the
 * CLM (Conservation of Linear Momentum) in the next iteration, and the position for each new
 * iteration will be passed to the graphics side of things. Will write a separate function to
 * handle the graphics part using openCV
 *
 * V.imp: The velocity of each ball will be given by a temp velocity vector
 *
 * The render function in the simulator class will render draw each object on an Matrix
 *  Created on: Oct 30, 2018
 *      Author: dev
 */
#include<iostream>
#include "volbot_classes.h"
using namespace std;
using namespace VolBot2D;
float velo_1[2] = {1., 0}; // Just use this to set the velocity of the robots
float velo_2[2] = {-2, 0};
float temp_pos[DIM];
float temp_velo[DIM];
int main(int argc, char **argv)
{

	VolBot Balls[NUMBER_OF_BALLS];  // Declare an array of balls
	Container Box; // The container
	Simulator S(ACTUAL_HEIGHT, ACTUAL_WIDTH, PIX_HEIGHT, PIX_WIDTH);

		S.run_simulation(Balls, Box, S);


  return 0;
}



/*
 * volbot_simulator.cpp
 *
 *  Created on: Nov 8, 2018
 *      Author: dev
 */

#include<iostream>
#include "volbot_classes.h"
using namespace std;
using namespace VolBot2D;

int main(int argc, char **argv)
{

	VolBot Balls[NUMBER_OF_BALLS];  // Declare an array of balls
	Container Box; // The container
	Simulator S(L, L, PIX_HEIGHT, PIX_WIDTH, SIGMA, EPSILON, 4, 3);

		S.run_simulation(Balls, Box, S);


  return 0;
}



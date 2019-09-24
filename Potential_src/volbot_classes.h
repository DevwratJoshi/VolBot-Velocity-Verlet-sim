/*
 * volbot_classes.h
 *
 *  Created on: Nov 8, 2018
 *      Author: dev
 */

#ifndef VOLBOT_CLASSES_H_
#define VOLBOT_CLASSES_H_

#include<iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#define NUMBER_OF_BALLS 9
#define GRAVITATIONAL_ACC -9.8 // gravitational acceleration in the downward direction
#define COEFFICIENT_E 1 // The coefficient of restitution
#define FPS 300 // Frames to simulate per second
#define DIM 2 // 2D world being considered

#define ACTUAL_HEIGHT 30.
#define ACTUAL_WIDTH 20.
#define PIX_HEIGHT 500.
#define PIX_WIDTH 500.
#define NUMBER_OF_FRAMES 1
#define RENDER_DELAY 100
#define FORCE_COEFF 15 // The numerator of the force calculation equation (F = FORCE_COEFF/(r^FORCE_POWER - ball_radius^FORCE_POWER)
#define FORCE_POWER 2
#define RADIUS 2
#define L 30
#define K 30
#define BOX_MIN_X 0.
#define BOX_MIN_Y 0.
#define BOX_MAX_X L
#define BOX_MAX_Y L
#define SIGMA 2*RADIUS*0.75
#define EPSILON 1 // Check equation 2.1
// L is the side length of the cell

namespace VolBot2D
{
	class VolBot
	{
		public:
			VolBot();
			float* set_position(float*);
			float set_radius(float);
			float set_mass(float);
			float* set_velocity(float*);
			float* set_force(float*);
			float* give_position(void);
			float* give_velocity(void);
			float give_radius(void);

			float temp_velocity[2];
			float mass;
			float position[2]; // a position array
			float velocity[2]; // The velocity of the ball
			float force[2]; // The force on the ball
			float temp_force[2];
			float last_force[2]; // Need the last force to calculate the next velocity
			float radius;
		private:




	};

	class Container // The container for the robots
	{
		public:
			Container();
			float* set_position(float*, float*); // The function to set the position of the bottom and top coordinates of the box.
			float* set_velocity(float*); // The function to set the velocity vector of the box
			float* give_velocity(void); // The function to return the velocity vector of the box
			float* give_min(void);  // Returns the position of the bottom left corner
			float* give_max(void);  // Returns the position of the top right corner


			float mass; // mass of the box
			float min[2]; // The coordinates for the bottom left corner
			float max[2]; // The coordinates for the top right corner
			float velocity[2]; // The velocity vector for the box
		private:


	};


class Simulator
	{
		public:
			Simulator(float, float, float, float, float, float, float, float); // The initializing function. Takes actual width and height as arguments and
											// uses them to calculate size ratio to render at
			bool BotvsBot(VolBot*, VolBot*, float*); // Function to check for collisions between balls
			char BotvsWall(VolBot*, Container); // The arguments are: Object, Wall, and array to hold collision normal
			int run_simulation(VolBot*, Container, Simulator); // The function that will run everything
			float calc_force(VolBot*, VolBot*, bool); // Will return the potential between two balls. Will use this to get total pot.energy
			//float calc_energy(VolBot*); // Potential calculator specific to each simulator
			float calc_force_balls(VolBot*, VolBot*);
			float calc_force_wall(VolBot*, Container);
			float calc_force_gravity(VolBot*, Container);

			float sigma;
			float epsilon;
			int LJP_bigger_exponent;
			int LJP_smaller_exponent; // The exponents of r in the lennard jones potential equation
			float size_ratio;

	};

}





#endif /* VOLBOT_CLASSES_H_ */

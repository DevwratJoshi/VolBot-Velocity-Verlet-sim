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
	Simulator S;



float pos_1[2] = {3., 0};
float pos_2[2] = {20., 0};
	//cout <<  "Current x position for 0 = " << *Balls[0].set_position(pos_1) << endl;
	//cout << "Current x position for 1 = " << *Balls[1].set_position(pos_2) << endl;

	Balls[0].set_position(pos_1);
	Balls[1].set_position(pos_2);

	Balls[0].set_velocity(velo_1);
	Balls[1].set_velocity(velo_2);
	static int count = 0;
while(count < 100)
{

  count ++;
	//cout << "instant = " << count++ << endl;
	int i = 0;
	for(i = 0; i < NUMBER_OF_BALLS-1; i++) // Check for collisions and edit temporary velocity vector
	{
		for(int j = i+1; j < NUMBER_OF_BALLS; j++)
		{

			float collision_direction[2]; // The vector normal to the collision.
			if(S.BotvsBot(&Balls[i], &Balls[j], collision_direction))
			{
				cout << "Cooooooollllliiiiiiiissssionnnnnnnn!" << endl;
				cout << count << endl ;
			}// If there is a collision detected, edit temp velos
										 // for both of them

			else
				cout << "didn't collide" << endl;


/*
			if(S.BotvsWall(Balls[i], Box, collision_direction)) // Collision between robot and wall
			{
				//////// Write this bit later

			}
*/
		}
/*
 * Next to change the position and velocity of each ball we're done with
 */

		for(int m = 0; m < DIM; m++)
				{

					temp_velo[m] = (Balls[i].velocity[m] + Balls[i].temp_velocity[m]);
					Balls[i].temp_velocity[m] = 0.; // reset temp_velocity of the object to 0
				}
		Balls[i].set_velocity(temp_velo);
		// Ball velocity now set for next frame


		float* current_pos = Balls[i].give_position();
		for(int m = 0; m < DIM; m++)
			{
				temp_pos[m] = Balls[i].position[m] + Balls[i].velocity[m] * (1.0/FPS);
			}

			Balls[i].set_position(temp_pos);
			cout << "x coordinate of the ball " << i << " = " << Balls[i].position[0] << endl;
	}
 // At this point, i = DIM-1

			for(int m = 0; m < DIM; m++)
					{

						temp_velo[m] = (Balls[i].velocity[m] + Balls[i].temp_velocity[m]);
						Balls[i].temp_velocity[m] = 0.; // reset temp_velocity of the object to 0
					}
			Balls[i].set_velocity(temp_velo);
			// Ball velocity now set for next frame



			for(int m = 0; m < DIM; m++)
				{
					temp_pos[m] = Balls[i].position[m] + Balls[i].velocity[m] * (1.0/FPS);
				}

				Balls[i].set_position(temp_pos);
		cout << "x coordinate of the ball " << i << " = " << Balls[i].position[0] << endl << endl;


}

//	while(1) // The simulation will run forever
	//{


	//}
	return 0;
}



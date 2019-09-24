/*
 * volbot_classes.cpp
 * The force on the robot will edit the velocity, which will edit the position
 * The functions will be run for all the particles, and the new values will edit the new frame
 * of the simulation
 *  Created on: Oct 29, 2018
 *      Author: dev
 */
#include "volbot_classes.h"
#include<iostream>
#include<cmath>


using namespace std;
using namespace VolBot2D;

using namespace cv;
float modulus(float a)
{
	if(a < 0)
	{
		 a = -a;
	}

	return a;
}

int render(Mat image, VolBot* Balls, Container Box, Simulator S)
{
	int i = 0;
	static char wndname[] = "VolBot";
	Scalar robot_colour (0, 0, 220);
	Scalar container_colour (220, 0, 0);
	Point center, pt1, pt2, pt3, pt4; // The center of a particular ball and four corners of the container

	 //* The first corner of the rectangle is considered to be the top left corner. The next point is on position forward in the
	 //* counter clockwise direction

	for(i = 0; i < NUMBER_OF_BALLS; i++)
	{
		center.x = Balls[i].position[0] * S.size_ratio;
		center.y = Balls[i].position[1]* S.size_ratio;

		circle(image, center, (int)Balls[i].give_radius()* S.size_ratio, robot_colour, -1, LINE_AA);
	}

	pt1.x = (int)Box.min[0]* S.size_ratio;
	pt1.y = (int)Box.max[1]* S.size_ratio;
	pt2.x = (int)Box.min[0]* S.size_ratio;
	pt2.y = (int)Box.min[1]* S.size_ratio;
	pt3.x = (int)Box.max[0]* S.size_ratio;
	pt3.y = (int)Box.min[1]* S.size_ratio;
	pt4.x = (int)Box.max[0]* S.size_ratio;
	pt4.y = (int)Box.max[1]* S.size_ratio;

	line( image, pt1, pt2, container_colour, 2, LINE_AA );
	line( image, pt2, pt3, container_colour, 2, LINE_AA );
	line( image, pt3, pt4, container_colour, 2, LINE_AA );

	flip(image, image, 0);
	imshow(wndname, image);
	waitKey(1000/FPS);

	return 0;
}


float dot_product(int length, float* a, float* b)
{
		float product = 0.;
		for(int i = 0; i < length; i++)
		{
			product += (*a)*(*b);
			a++;
			b++;
		}

		return product;

}
/*
 * The next function will calculate relative velocity after collision using coeff of restitution

float relative_velocity(float e, float vel_1, float vel_2)
{

	return (modulus((vel_2-vel_1))*e); // Return a positive scalar. The direction will be figured out later
}
*/
VolBot::VolBot()
{
	mass = 1;
	radius = RADIUS;
	temp_velocity[0] = 0.;
	temp_velocity[1] = 0.;
	force[0] = 0.;
	force[1] = 0.;
	temp_force[0] = 0.;
	temp_force[1] = GRAVITATIONAL_ACC;
}

Container::Container()
{
	min[0] = BOX_MIN_X;
	min[1] = BOX_MIN_Y;
	max[0] = BOX_MAX_X;
	max[1] = BOX_MAX_Y;
	mass = HUGE_VALF;
	velocity[0] = 0.;
	velocity[1] = 0.;

}
Simulator::Simulator(float act_height, float act_width, float pix_height, float pix_width) // act_height and act_width in meters
{
	size_ratio = (float)pix_height/act_height;
}

float* VolBot::set_position(float* new_position)
{
	float* current_pos = new_position;

	position[0] = *current_pos;
	//cout << "New position x is = " << position[0] << endl;
	current_pos++;
	position[1]= *current_pos;
	//cout << "New position y is = " << position[1] << endl;
	return position;
}

float* VolBot::set_velocity(float* new_velocity)
{
	float* curr_velocity = new_velocity;

	velocity[0] = *curr_velocity;
	//cout << "New velocity x is = " << velocity[0] << endl;
	curr_velocity++;
	velocity[1] = *curr_velocity;
	//cout << "New velocity y is = " << velocity[1] << endl;
	return velocity;

}

float* VolBot::set_force(float* new_force) // This is planned to be just gravity for now.
{										// The velocity will be calculated using conservation
										// momentum laws.
	float* current_force = force;
	current_force[0] = new_force[0];
	current_force[1] = new_force[1];
	return current_force;

}

float* VolBot::give_position(void)
{
	return position;
}

float* VolBot::give_velocity(void)
{

	return velocity;

}
float VolBot::give_radius(void)
{
	return radius;
}

// And now for the container functions

float* Container::set_position(float* new_min, float* new_max)
{


	min[0] = new_min[0];
	min[1] = new_min[1];

	max[0] = new_max[0];
	max[1] = new_max[1];
	return new_min; // Picked a random value to return.
}

float* Container::set_velocity(float* new_velocity)
{
	velocity[0] = new_velocity[0];
	velocity[1] = new_velocity[1];

	return velocity;

}

float* Container::give_velocity(void)
{

	return velocity;

}

float* Container::give_min(void)
{

	return min;

}

float* Container::give_max(void)
{

	return max;

}

bool Simulator::BotvsBot(VolBot* Focus, VolBot* Other, float* collision_direction) // Focus is the robot currently being considered,
{											// and Other is the other one
	float* Focus_pos = Focus->give_position(); // argument 3 is the vector in the direction of
	float* Other_pos = Other->give_position(); //velocity added to Focus
	float* Focus_velo = Focus->give_velocity();
	float* Other_velo = Other->give_velocity();
	float* point_1 = collision_direction;
	float* point_2 = ++collision_direction;
	float r = Focus->give_radius() + Other->give_radius();
	float rel_velo; // Magnitude of the relative velocity after collision
	float distance = sqrt((double)(Focus->position[0]-Other->position[0]) * (Focus->position[0]-Other->position[0])) + ((Focus->position[1]-Other->position[1]) * (Focus->position[1]-Other->position[1]));
	collision_direction = point_1; // Restore collision_direction to its original position

	if (r >= distance) // There has been a collision
	{ // The collision vector will be in the correct direction for Focus
		cout << "sum of radii = " << r << endl;
		cout << "distance = " << distance << endl;
		*point_1 = Other_pos[0] - Focus_pos[0];
		*point_2 = Other_pos[1] - Focus_pos[1];


		r = sqrt((double)((*point_1 * (*point_1)) + (*point_2) * (*point_2)));
		collision_direction[0] = *point_1/r;
		collision_direction[1] = *point_2/r;
		/* Collision detection holds the unit vector in the direction of the collision.
		 * (in the direction from Focus to Other)
		 * Now to resolve the velocities.
		 */

		//rel_velo = relative_velocity((float)COEFFICIENT_E, dot_product((int)DIM, Focus.give_velocity(), collision_direction), dot_product((int)DIM, Other.give_velocity(), collision_direction));

		if(FORCE_DISTANCE)
		{
			float radius_difference = Focus->give_radius() + Other->give_radius();
			Focus->position[0] -= ((radius_difference - distance)/2) * collision_direction[0];
			Focus->position[1] -= ((radius_difference - distance)/2) * collision_direction[1];
			Other->position[0] += ((radius_difference - distance)/2) * collision_direction[0];
			Other->position[1] += ((radius_difference - distance)/2) * collision_direction[1];
		}

		for(int i = 0; i < DIM; i++)
		{
			float v1, v2;

			//v = dot_product((int)DIM, Focus->velocity, collision_direction) - dot_product((int)DIM, Other->velocity, collision_direction);
			v1 = dot_product((int)DIM, Focus->velocity, collision_direction);
			v2 = dot_product((int)DIM, Other->velocity, collision_direction);
					Focus->temp_velocity[i] -= collision_direction[i] * (v1 - v2 * COEFFICIENT_E);
						Other->temp_velocity[i] += collision_direction[i] * (-v2 + v1 * COEFFICIENT_E);
		}

		/*
		 * We now have the relative velocity after collision
		 *  The individual velocities after collision in the direction of the collision vector will be
		 *  calculated using conservation of linear momentum
		 */
		return true; // Return TRUE for a collision
	}


    return false;

}

char Simulator::BotvsWall(VolBot* Bot, Container Wall) // Focus is the robot currently being considered,
{											// and Other is the other one
	float* Bot_pos = Bot->give_position();
	float* Wall_min = Wall.give_min();
	float* Wall_max = Wall.give_max();
	float collision_direction[2]; // Direction of collision away from the center of the robot
	float v; // Velocity after collision
	// Use r for the distance to the wall
		float r = Bot->position[0] - Wall_min[0]; // The distance to the wall


		if(r < (Bot->give_radius()) && Bot->position[1] <= Wall_max[1]) // The ball collided with the left side
		{
			collision_direction[0] = -1.0;
			collision_direction[1] = 0.;
			v = dot_product((int)DIM, Bot->velocity, collision_direction);
			Bot->temp_velocity[0] -=  v * (1 + COEFFICIENT_E) * collision_direction[0];
			Bot->force[1] = GRAVITATIONAL_ACC; // normal force
			//return 'l'; // Returns l for left wall when the ball touches the left wall
		}

		r = Wall_max[0] - Bot_pos[0];


		if(r < (Bot->give_radius())&& Bot->position[1] <= Wall_max[1]) // The ball collided with the left side
			{
				collision_direction[0] = 1.0;
				collision_direction[1] = 0.;
				v = dot_product((int)DIM, Bot->velocity, collision_direction);
				Bot->temp_velocity[0] -=  v * (1 + COEFFICIENT_E) * collision_direction[0];
				Bot->force[1] = GRAVITATIONAL_ACC;
				//return 'r'; // Returns r for right wall when the ball touches the right wall
			}

		r = Bot_pos[1] - Wall_min[1]; // The y coordinate of the bottom line

		if(r < Bot->give_radius() && Bot->position[1] <= Wall_max[1])
		{
			collision_direction[0] = 0.;
			collision_direction[1] = -1.0;
			v = dot_product((int)DIM, Bot->velocity, collision_direction);
			Bot->temp_velocity[1] -=  v * (1 + COEFFICIENT_E) * collision_direction[1];
			Bot->force[1] = 0.; // normal force
			return 'b'; // Returns b for bottom if the robot touches the bottom
		}
    // Return TRUE for a collision
		Bot->force[1] = GRAVITATIONAL_ACC;
		return 'n'; // Nothing for no collision
}

int Simulator::run_simulation(VolBot* Balls, Container Box, Simulator S )
{

	float temp_pos[DIM];
	float temp_velo[DIM];

	float pos[2];
	float vel[2] = {0., 0.};
	pos[0] = 4;
	pos[1] = 4;
	for(int row = 0; row < 3; row++)
	{
		static int i = 0;
		for(int column = 0; column < 3; column++)
		{
			pos[0] += 5*column;
			Balls[i].set_position(pos);
			pos[0] -= 5*column;
			Balls[i].set_velocity(vel);
			i++;
		}
			pos[1] += 6;
			if(row%2 == 0)
				pos[0] = 3;

			else
				pos[0] = 4;

	}

		//waitKey(0);
		//cout <<  "Current x position for 0 = " << *Balls[0].set_position(pos_1) << endl;
		//cout << "Current x position for 1 = " << *Balls[1].set_position(pos_2) << endl;


		int count = 0;
	while(1)
	{
		Mat image = Mat::zeros(PIX_HEIGHT, PIX_WIDTH, CV_8UC3);
	  //count ++;
		//cout << "instant = " << count++ << endl;
		int i = 0;
		for(i = 0; i < NUMBER_OF_BALLS; i++) // Check for collisions and edit temporary velocity vector
		{
			for(int j = i+1; j < NUMBER_OF_BALLS; j++)
			{

				float collision_direction[2]; // The vector normal to the collision.
				if(S.BotvsBot(&Balls[i], &Balls[j], collision_direction))
				{
					cout << "Collision between robots " << i << " and " << j << "!" << endl;
					cout << count << endl ;
				}// If there is a collision detected, edit temp velos
											 // for both of them

				//else
					//cout << "Robots didn't collide this time" << endl;

			}
/*
 * Now to check for collision between the Robot i and the container
 */
			//cout << " Checking for collision of i with wall = " << i << endl;
			char a =  S.BotvsWall(&Balls[i], Box);
			 // Collision between robot and wall

								switch (a)
								{
									case 'r':
										cout << "The robot " << i << " collided with the right wall!" << endl;
										break;

									case 'l':
										cout << "The robot " << i << " collided with the left wall!" << endl;
										break;

									case 'b':
										cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>The robot " << i << " collided with the floor!" << endl;
										break;

									case 'n':
										cout << "No collision with the wall container for robot " << i << endl;
										break;

									default:
										break;
								}




			/*
			 * Next to change the position and velocity of each ball we're done with
			 */
			for(int m = 0; m < DIM; m++)
					{

						temp_velo[m] = (Balls[i].velocity[m] + Balls[i].temp_velocity[m]);
						if(m == 1) // the y direction
							temp_velo[m] += Balls[i].force[1] * (1.0/FPS);

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
				//cout << "x coordinate of the ball " << i << " = " << Balls[i].position[0] << endl;
				//cout << "y coordinate of the ball " << i << " = " << Balls[i].position[1] << endl;
				if(i == DIM-1)
					cout << endl;
		}


			render(image, Balls, Box, S);



	}


		return 0;
}
/*
int Simulator::render(Mat image, VolBot* Balls, Container Box, Simulator S)
{
	int i = 0;
	Scalar robot_colour (0, 0, 220);
	Scalar container_colour (220, 0, 0);
	Point center, pt1, pt2, pt3, pt4; // The center of a particular ball and four corners of the container

	 //* The first corner of the rectangle is considered to be the top left corner. The next point is on position forward in the
	 //* counter clockwise direction

	for(i = 0; i < NUMBER_OF_BALLS; i++)
	{
		center.x = Balls[i].position[0] * S.size_ratio;
		center.y = Balls[i].position[1]* S.size_ratio;

		circle(image, center, (int)Balls[i].give_radius()* S.size_ratio, robot_colour, -1, LINE_AA);
	}

	pt1.x = (int)Box.min[0]* S.size_ratio;
	pt1.y = (int)Box.max[1]* S.size_ratio;
	pt2.x = (int)Box.min[0]* S.size_ratio;
	pt2.y = (int)Box.min[1]* S.size_ratio;
	pt3.x = (int)Box.max[0]* S.size_ratio;
	pt3.y = (int)Box.min[1]* S.size_ratio;
	pt4.x = (int)Box.max[0]* S.size_ratio;
	pt4.y = (int)Box.max[1]* S.size_ratio;

	line( image, pt1, pt2, container_colour, 2, LINE_AA );
	line( image, pt2, pt3, container_colour, 2, LINE_AA );
	line( image, pt3, pt4, container_colour, 2, LINE_AA );


	//waitKey(0);

	return 0;
}
*/



/*
 * test.cpp
 *
 *  Created on: Nov 19, 2018
 *      Author: dev
 */


#include "volbot_classes.h"
#include<iostream>
#include<cmath>
#include<cstdlib>

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

/*
 * The following is a function to return the nth power of the input number
 */
float power(float number, int n)
{
	float num = 1;

	if(n < 0)
	{
		number = 1/number;
		n = -1*n;
	}

	if(n == 0)
	{
		return 1.0;
	}
	for(int i = 1; i <= n; i++)
	{
		num *= number;
	}

	return num;
}


static Scalar randomColor(RNG& rng)
{
    int icolor = (unsigned)rng;
    return Scalar(icolor&255, (icolor>>8)&255, (icolor>>16)&255);
}


int render(Mat image, VolBot* Balls, Container Box, Simulator S)
{
	int i = 0;
	static char wndname[] = "VolBot";
	RNG rng(0xFFFFFFFF);
	Scalar line_colour (0, 0, 220);
	Scalar container_colour (220, 0, 0);
	Point center, pt1, pt2, pt3, pt4; // The center of a particular ball and four corners of the container

	 //* The first corner of the rectangle is considered to be the top left corner. The next point is on position forward in the
	 //* counter clockwise direction


	for(i = 0; i < NUMBER_OF_BALLS; i++)
	{
		center.x = Balls[i].position[0] * S.size_ratio;
		center.y = Balls[i].position[1] * S.size_ratio;
		pt1 = center;

		float force_mag = sqrt((double)Balls[i].force[0]*Balls[i].force[0] + Balls[i].force[1]*Balls[i].force[1]);

		pt2.x = center.x + 2*(Balls[i].force[0]/force_mag)*S.size_ratio;
		pt2.y = center.y + 2*(Balls[i].force[1]/force_mag)*S.size_ratio;

		circle(image, center, (int)Balls[i].give_radius()* S.size_ratio, randomColor(rng), -1, LINE_AA);

		arrowedLine(image, pt1, pt2, line_colour, 2, LINE_AA);
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
	mass = 2;
	radius = RADIUS;

	for(int i = 0; i < DIM; i++)
	{
		temp_velocity[i] = 0.;
		force[i] = 0.;
		temp_force[i] = 0.;
		last_force[i] = 0.;
	}
	//temp_force[1] = GRAVITATIONAL_ACC*mass;

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


Simulator::Simulator(float act_height, float act_width, float pix_height, float pix_width, float new_sigma, float new_epsilon, float big, float small) // act_height and act_width in meters
{
	size_ratio = (float)pix_height/act_height;
	sigma = new_sigma;
	epsilon = new_epsilon;
	LJP_bigger_exponent = big;
	LJP_smaller_exponent = small;
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

// And now for the container function

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


/*
 * The following function will calculate the potential of one robot due to the presence of another robot
 */
/*
float Simulator::calc_energy(VolBot* Balls)
{

	float potential;
	float r = (((focus_pos[0] - focus_pos[0])*(focus_pos[0] - focus_pos[0])) + ((focus_pos[1] - focus_pos[1])*(focus_pos[1] - focus_pos[1])));
	potential = 4 * epsilon * ((1/power((sigma/r), LJP_bigger_exponent)) - 1/power((sigma/r), LJP_smaller_exponent));

	return potential;

}
*/

/*
 * The calc_force function will calculate the force exerted between robots.
 * The Balls variable
 * The flag variable is to tell if the other robot is a real robot or an image robot.
 * Will make a temporary Balls object to send to this function to act as an image robot
 * The flag variable is to tell if the robot is one from the fundamental cell or if its from an image cell
 * Flag is true for a fundamental robot and false for an image robot
 * The function will return the potential energy added to the system due to the two robots
 * Will return potential energy of one robot x 2 for two real robots and pot energy of one robot if one is fake
 */
float Simulator::calc_force_balls(VolBot* Focus, VolBot* Other)
{
	float potential = 0.;
	float* focus_pos  = Focus->position;
	float* other_pos = Other->position;

	double r = sqrt((double)((focus_pos[0] - other_pos[0])*(focus_pos[0] - other_pos[0])) + ((focus_pos[1] - other_pos[1])*(focus_pos[1] - other_pos[1])));
	float dist = (Focus->radius + Other->radius);
	/*
	 * The following is the expression for differential of potential
	 */
	//if(flag)
	//cout << "r = " << r << endl;
	//if(r < (dist*2)) // The distance must be less than the distance between the two centers
	{
		/*
		 * Initially the temp_force due to the contact between two balls will be considered
		 */
		cout << "the distance between the balls is " << r << endl;
		float diff_of_potential_by_r = FORCE_COEFF/((power(r, FORCE_POWER)-power(dist/2, FORCE_POWER))*r);

			for(int i = 0; i < DIM; i++)
			{
				Focus->temp_force[i] += diff_of_potential_by_r*(focus_pos[i]-other_pos[i]);
				potential += -FORCE_COEFF*(log((r - dist)/(r + dist)))/(2*dist);
				Other->temp_force[i] += diff_of_potential_by_r*(other_pos[i]-focus_pos[i]);

				//cout << "The temp force on one of the balls is " << -diff_of_potential_by_r*(focus_pos[i]-other_pos[i]);
			}

			cout << "The temp force on one of the balls is " << -diff_of_potential_by_r*(focus_pos[1]-other_pos[1]) << endl;
			cout << endl;
	}

		return potential*2; // Energy of both balls to be considered


}

float Simulator::calc_force_wall(VolBot* Focus, Container Wall)
{
	float* Wall_min = Wall.give_min();
	float* Wall_max = Wall.give_max();
	float* focus_pos  = Focus->position;
	float potential = 0.;
	float r;
	float diff_of_potential_by_r;


	r = focus_pos[0] - Wall_min[0]; // The distance to the wall
	diff_of_potential_by_r = 4*epsilon*power(sigma, LJP_smaller_exponent)*(LJP_smaller_exponent*power(r, -(LJP_smaller_exponent+2)) - (power(sigma, LJP_bigger_exponent-LJP_smaller_exponent)*LJP_bigger_exponent*power(r, -(LJP_bigger_exponent+2))));
	//diff_of_potential_by_r = K;
	if(r < (Focus->give_radius()) && Focus->position[1] <= Wall_max[1]) // The ball collided with the left side
	{
		Focus->temp_force[0] += -diff_of_potential_by_r*(Wall_max[0]-focus_pos[0]);
		potential += 4 * epsilon * ((power((sigma/r), LJP_bigger_exponent)) - power((sigma/r), LJP_smaller_exponent));
	}


	r = Wall_max[0] - focus_pos[0];
	diff_of_potential_by_r = 4*epsilon*power(sigma, LJP_smaller_exponent)*(LJP_smaller_exponent*power(r, -(LJP_smaller_exponent+2)) - (power(sigma, LJP_bigger_exponent-LJP_smaller_exponent)*LJP_bigger_exponent*power(r, -(LJP_bigger_exponent+2))));
	//diff_of_potential_by_r = K;
	if(r < (Focus->give_radius())&& Focus->position[1] <= Wall_max[1]) // The ball collided with the right side
	{
		Focus->temp_force[0] += -diff_of_potential_by_r*(focus_pos[0]-Wall_max[0]);
		potential += 4 * epsilon * ((power((sigma/r), LJP_bigger_exponent)) - power((sigma/r), LJP_smaller_exponent));
	}


	r = focus_pos[1] - Wall_min[1]; // The y coordinate of the bottom line
	diff_of_potential_by_r = 4*epsilon*power(sigma, LJP_smaller_exponent)*(LJP_smaller_exponent*power(r, -(LJP_smaller_exponent+2)) - (power(sigma, LJP_bigger_exponent-LJP_smaller_exponent)*LJP_bigger_exponent*power(r, -(LJP_bigger_exponent+2))));
	//diff_of_potential_by_r = K;
	if(r < Focus->give_radius() && Focus->position[1] <= Wall_max[1])
	{
		Focus->temp_force[1] += -diff_of_potential_by_r*(focus_pos[1]-Wall_min[1]);
		potential += 4 * epsilon * ((power((sigma/r), LJP_bigger_exponent)) - power((sigma/r), LJP_smaller_exponent));
	}

				return potential;
}

float Simulator::calc_force_gravity(VolBot* Focus, Container Box)
{
	//float potential = 0;
	float r = Focus->position[1] - Box.min[1];

	r = Focus->position[1] - Box.min[1];

	Focus->temp_force[1] += GRAVITATIONAL_ACC*Focus->mass;
	return 1.;
}

int Simulator::run_simulation(VolBot* Balls, Container Box, Simulator S )
{
	float temp_pos[DIM];
	float temp_velo[DIM];
	float pos[2];

	/*
	 * Now to create a data file to store the energy data
	 */

	ofstream pot, kin, ene;

	pot.open("pot_energy.dat"); // opens the file
	   if( !pot ) { // file couldn't be opened
	      cerr << "Error: file could not be opened" << endl;
	      exit(1);
	   }

	kin.open("kin_energy.dat"); // opens the file
	   if( !kin ) { // file couldn't be opened
		  cerr << "Error: file could not be opened" << endl;
		  exit(1);
	   }

	ene.open("energy.dat"); // opens the file
	   if( !ene ) { // file couldn't be opened
		  cerr << "Error: file could not be opened" << endl;
		  exit(1);
	   }


	float pos1[2] = {10., 10.};
	float pos2[2] = {11., 15.};
	float pos3[2] = {18., 18.};
	float pos4[2] = {17., 24.};
	float vel[2] = {0., 0.};

	Balls[0].set_position(pos1);
	Balls[1].set_position(pos2);
	//Balls[2].set_position(pos3);

	Balls[0].set_velocity(vel);
	Balls[1].set_velocity(vel);
	//Balls[2].set_velocity(vel);
	pos[0] = 3;
	pos[1] = 5;
	/*
	 * Setting the initial position and velocity of the robots
	 */



	for(int row = 0; row < 3; row++)
	{
		static int i = 0;
		for(int column = 0; column < 3; column++)
		{
			pos[0] += 8*column;
			Balls[i].set_position(pos);
			pos[0] -= 8*column;
			Balls[i].set_velocity(vel);
			i++;
		}
			pos[1] += 8;
			if(row%2 == 0)
				pos[0] = 3;

			else
				pos[0] = 4;

	}

		/*
		 * The main while loop
		 */

	long unsigned int count = 0; // will keep count of the number of cycles the simulator has been through

	while(1)
	{

		Mat image = Mat::zeros(PIX_HEIGHT, PIX_WIDTH, CV_8UC3);
		float potential = 0.; // Initialize the potential energy of the system
		float kinetic = 0.;   // Initialize the kinetic energy of the system
		/*
		 * Actually calculating the positions and velocities of the robots
		 */

		int i = 0;
		for(i = 0; i < NUMBER_OF_BALLS; i++)
		{

/*
 * The following loop will cycle through the remaining fundamental robots and check the potential between the focus robot
 * other (fundamental) robots
 */
			for(int j = i+1; j < NUMBER_OF_BALLS; j++)
			{
				potential += calc_force_balls(&Balls[i], &Balls[j]);



			} // End of inner for loop. Cycling through the Other robots



			potential += calc_force_wall(&Balls[i], Box);
			//calc_force_gravity(&Balls[i], Box);
/*
 * For the first cycle, the force must be saved in both the last_force and the force variables.
 * other than that, the last_force variable should be updated with the value in force and then force should
 * be updated with the value in temp_force. Thus, the values of both current force and last force are
 * saved in order to calculate the next velocity.
 *
 * The position of the balls cannot be edited until after the i for loop
 * This is because the position of each ball AND its image balls will alter the potential of many other balls
 * therefore, only the force on a ball will be altered within this loop
 */


				for (int p = 0; p < DIM; p++)
				{

					if(count == 0)
					{
						Balls[i].last_force[p] =  0.;//Balls[i].temp_force[p];

					}

					else
					{
						Balls[i].last_force[p] = Balls[i].force[p];
					}

					Balls[i].force[p] = Balls[i].temp_force[p];

					Balls[i].temp_force[p] = 0.;


				}



			} // end of outer for loop. (cycling through the focus robots)


		/*
		 * Velocities and positions will be edited here
		 *
		 * Before anything is calculated, the robots will be rendered.
		 *
		 * Velocity calculation sequence will come first, but new velocity will only be calculated if this is not the first time
		 * the loop is running.
		 * New position will be calculated After.
		 *
		 *
		 */

			render(image, Balls, Box, S); // Edit this later to draw only the balls

			// Now calculating velocity in accordance with Velocity Verlet algorithm

			for (int i = 0; i < NUMBER_OF_BALLS; i++)
			{
				if(count != 0)
				{
					float vel_t[DIM];

					for(int m = 0; m < DIM; m++)
					{
						vel_t[m] = Balls[i].velocity[m];
					}

					for(int m = 0; m < DIM; m++)
					{
						Balls[i].velocity[m] = (vel_t[m] + (Balls[i].last_force[m] + Balls[i].force[m])/(2*Balls[i].mass * FPS))*COEFFICIENT_E;

						kinetic += Balls[i].mass*Balls[i].velocity[m]*Balls[i].velocity[m]; // Calculating the velocity^2 part of kinetic energy
						//cout <<"Velocity for " << i << "th robot in the " << m  << "direction is = "<< Balls[i].velocity[m] << endl;

					}



				}


				// Now calculating position in accordance with Velocity Verlet algorithm

				float pos_t[DIM];
				for(int m = 0; m < DIM; m++)
				{
					pos_t[m] = Balls[i].position[m];
				}

				for(int m = 0; m < DIM; m++)
				{
					Balls[i].position[m] = pos_t[m] + (Balls[i].velocity[m] + (Balls[i].force[m])/(2*Balls[i].mass * FPS))/FPS;

					potential += Balls[i].mass * Balls[i].position[1] * (-GRAVITATIONAL_ACC);

					/*
					 * The following if else sequence corrects the position of the balls if they go beyond the boundaries of the box
					 */
			/*
					int check = 0;
					while (temp > L)
					{
						temp = temp - L;
						if(check == 10)
							cout << "SSSSSSSSSSSOOOOOOOOOOOMMMMMMMMEEEEETTTTTTTHHHHHHIIIIIIIINNNNNNNNNNGGGGGGG WWWWWWWWWRRRRRRRRROOOOOOOOONNNNNNNGGGGG";
					}

					check = 0;
					while (temp < 0)
					{
						temp = temp + L;
						if(check == 10)
						    cout << "SSSSSSSSSSSOOOOOOOOOOOMMMMMMMMEEEEETTTTTTTHHHHHHIIIIIIIINNNNNNNNNNGGGGGGG WWWWWWWWWRRRRRRRRROOOOOOOOONNNNNNNGGGGG";
					}

			*/


				}
			}


		count++;

		kinetic = kinetic/2.;

		/*
		 * At this point, kinetic and potential contain the potential energy and kinetic energy of the system respectively
		 * The values will then be added and written to a dat file to be
		 */

		if(count != 0)
		{
			pot << count <<"	" << potential << endl; // writing to the file
			kin << count << "	" << kinetic << endl;
			ene << count << "	" << potential << "	" << kinetic << "	" << potential + kinetic << endl;
		}

	} // End of while loop




		return 0;
}



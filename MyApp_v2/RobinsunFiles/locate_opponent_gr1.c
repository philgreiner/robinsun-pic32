/*!
* \file robot_detect_gr1.cc
* \brief File description
*/

#include "locate_opponent_gr1.h"

NAMESPACE_INIT(ctrlGr1); // where X should be replaced by your group number

void locate_opponent(CtrlStruct *cvs)
{
	/*	LOCATE_OPPONENT:
	 *		Given the distances extracted from the 6 sonars and the number of opponents (encoded with LT24),
	 *		this function updates the position(s) of the opponent(s).
	 *		
	 *		Assumptions : sonars 1,2,3 are on the front side (clamps side) from left to right
	 *					  sonars 4,5,6 are on the rear side (microswitches side) from left to right
	 */

	//============================================
	// DECLARATION OF VARIABLES
	//============================================

	int i;

	CtrlIn *ivs;
    ivs = cvs->inputs;

 	int nb_opponents = ivs->nb_opponents;
	int opponents_detected = 0;
	double opponent_radius = 0.15;		// max distance from a point of the opponent and its center
	double opponents_xpos[2];			// vector of size 2 since {opponents_detected <= 2}
	double opponents_ypos[2];	

	// Position of Robinsun
	double x_robinsun 	  = cvs->state->position[0];
	double y_robinsun 	  = cvs->state->position[1];
	double theta_robinsun = cvs->state->position[2];

	// Position of each sonar and their respective readings
	double sonar_xdist = 0.12; // distance from the center of Robinsun along the x-axis of the mobile frame 															
	double sonar_ydist = 0.07; // distance from the center of Robinsun along the y-axis of the mobile frame															

	double x_sonars[6] = {	x_robinsun + sonar_xdist*cos(theta_robinsun) - sonar_ydist*sin(theta_robinsun),
							x_robinsun + sonar_xdist*cos(theta_robinsun),
							x_robinsun + sonar_xdist*cos(theta_robinsun) + sonar_ydist*sin(theta_robinsun),
							x_robinsun - sonar_xdist*cos(theta_robinsun) - sonar_ydist*sin(theta_robinsun),
							x_robinsun - sonar_xdist*cos(theta_robinsun),
							x_robinsun - sonar_xdist*cos(theta_robinsun) + sonar_ydist*sin(theta_robinsun)};															
	double y_sonars[6] = {	y_robinsun + sonar_xdist*sin(theta_robinsun) + sonar_ydist*cos(theta_robinsun),
							y_robinsun + sonar_xdist*sin(theta_robinsun),
							y_robinsun + sonar_xdist*sin(theta_robinsun) - sonar_ydist*cos(theta_robinsun),
							y_robinsun - sonar_xdist*sin(theta_robinsun) + sonar_ydist*cos(theta_robinsun),
							y_robinsun - sonar_xdist*sin(theta_robinsun),
							y_robinsun - sonar_xdist*sin(theta_robinsun) - sonar_ydist*cos(theta_robinsun)};												
	
	double dist[6];
	for (i=0; i<6; i=i+1)
		dist[i] = cvs->state->obstacle_dist[i];

	double sign[2] = {1.0,-1.0};  // used to adapt the contributions of front and rear sonars

	// Conditions on the measurements
	double max_dist = 0.8;	// maximum measurable/significant distance					// TO ADAPT
	double min_dist = 0.05; // maximum measurable/significant distance 					// TO ADAPT

	// Intermediate variables
	double xpos = 0.0;
	double ypos = 0.0;

	//============================================
	// LOCALIZATION ALGORITHM
	//============================================

	/*  If Robinsun is in front of an external wall (and close to it), the values extracted from the 
	 *  front sonars are not useful because the opponent(s) can not be in front of them.
	 *  The same idea is used when it has its back to the external wall.
	 */

	for(i=0; (i<6 && opponents_detected<2); i=i+3)
	{ // for : i=0 --> front side / i=3 --> rear side

		if (dist[i] < max_dist)
		{ // left

			if (dist[i+1] < max_dist)
			{ // left and center

				if (dist[i+2] < max_dist)
				{ // left, center and right --> opponent in front of the center sonar

					xpos = x_sonars[i+1] + sign[i]*(dist[i+1] + opponent_radius)*cos(theta_robinsun);
					ypos = y_sonars[i+1] + sign[i]*(dist[i+1] + opponent_radius)*sin(theta_robinsun); 

				} // left, center and right
				else
				{ // left and center only --> opponent in front of the left sonar

					xpos = x_sonars[i] + sign[i]*((dist[i]+dist[i+1])/2.0 + opponent_radius)*cos(theta_robinsun);
					ypos = y_sonars[i] + sign[i]*((dist[i]+dist[i+1])/2.0 + opponent_radius)*sin(theta_robinsun); 

				} // left and center only

				if (isMeasureRelevant(xpos,ypos))
				{
					opponents_xpos[opponents_detected-1] = xpos;
					opponents_ypos[opponents_detected-1] = ypos;
					opponents_detected += 1;
				}

			} // left and center
			
			else if (dist[i+2] < max_dist)
			{ // left and right --> 2 opponents : one on the left of the left sonar, 
			  //								  the other on the right of the right sonar

				xpos = x_sonars[i] + sign[i]*dist[i]*cos(theta_robinsun) - opponent_radius*sin(theta_robinsun);
				ypos = y_sonars[i] + sign[i]*dist[i]*sin(theta_robinsun) + opponent_radius*cos(theta_robinsun); 

				if (isMeasureRelevant(xpos,ypos))
				{
					opponents_xpos[0] = xpos;
					opponents_ypos[0] = ypos;
					opponents_detected += 1;
				}

				xpos = x_sonars[i+2] + sign[i]*dist[i+2]*cos(theta_robinsun) + opponent_radius*sin(theta_robinsun);
				ypos = y_sonars[i+2] + sign[i]*dist[i+2]*sin(theta_robinsun) - opponent_radius*cos(theta_robinsun); 

				if (isMeasureRelevant(xpos,ypos))
				{
					opponents_xpos[1] = xpos;
					opponents_ypos[1] = ypos;
					opponents_detected += 1;
				}

			} // left and right --> 2 opponents ...

			else
			{ // left only --> opponent on the left of the left sonar

				xpos = x_sonars[i] + sign[i]*dist[i]*cos(theta_robinsun) - opponent_radius*sin(theta_robinsun);
				ypos = y_sonars[i] + sign[i]*dist[i]*sin(theta_robinsun) + opponent_radius*cos(theta_robinsun);

				if (isMeasureRelevant(xpos,ypos))
				{
					opponents_xpos[opponents_detected-1] = xpos;
					opponents_ypos[opponents_detected-1] = ypos;
					opponents_detected += 1;
				}

			} // left only

		} // left

		else if (dist[i+2] < max_dist)
		{ // right

			opponents_detected += 1;

			if (dist[i+1] < max_dist)
			{ // right and center

				xpos = x_sonars[i] + sign[i]*((dist[i+1]+dist[i+2])/2.0 + opponent_radius)*cos(theta_robinsun);
				ypos = y_sonars[i] + sign[i]*((dist[i+1]+dist[i+2])/2.0 + opponent_radius)*sin(theta_robinsun); 

				if (isMeasureRelevant(xpos,ypos))
				{
					opponents_xpos[opponents_detected-1] = xpos;
					opponents_ypos[opponents_detected-1] = ypos;
					opponents_detected += 1;
				}
				
			} // right and center
			
			else
			{ // right only

				xpos = x_sonars[i+2] + sign[i]*dist[i+2]*cos(theta_robinsun) + opponent_radius*sin(theta_robinsun);
				ypos = y_sonars[i+2] + sign[i]*dist[i+2]*sin(theta_robinsun) - opponent_radius*cos(theta_robinsun); 

				if (isMeasureRelevant(xpos,ypos))
				{
					opponents_xpos[opponents_detected-1] = xpos;
					opponents_ypos[opponents_detected-1] = ypos;
					opponents_detected += 1;
				}

			} // right only

		} // right

		else if (dist[i+1] < max_dist)
		{ // center only --> opponent in front of the center sonar

			xpos = x_sonars[i+1] + sign[i]*(dist[i+1] + opponent_radius)*cos(theta_robinsun);
			ypos = y_sonars[i+1] + sign[i]*(dist[i+1] + opponent_radius)*sin(theta_robinsun);
			 
			if (isMeasureRelevant(xpos,ypos))
			{
				opponents_xpos[opponents_detected-1] = xpos;
				opponents_ypos[opponents_detected-1] = ypos;
				opponents_detected += 1;
			}

		} // center only
	
	} // for : i=0 --> front side / i=3 --> rear side


	//============================================
	// UPDATE THE STATE OF THE STRUCTURE
	//============================================

	if (opponents_detected == 1)
	{
		if (fabs(opponents_xpos[0]) <= 1.0)
			cvs->state->opponent_position[0] = opponents_xpos[0];
		if (fabs(opponents_ypos[0]) <= 1.5)
			cvs->state->opponent_position[1] = opponents_ypos[0];
	}
	else if ((opponents_detected == 2) && (nb_opponents == 2))				// ATTENTION NEED TO BE INITIALIZED !!
	{
		// Position of the first opponent
		if (fabs(opponents_xpos[0]) <= 1.0)
			cvs->state->opponent_position[0] = opponents_xpos[0];
		if (fabs(opponents_ypos[0]) <= 1.5)
			cvs->state->opponent_position[1] = opponents_ypos[0];

		// Position of the second opponent
		if (fabs(opponents_xpos[1]) <= 1.0)
			cvs->state->opponent_position[2] = opponents_xpos[1];
		if (fabs(opponents_ypos[1]) <= 1.5)
			cvs->state->opponent_position[3] = opponents_ypos[1];
	}	

}


int isMeasureRelevant(double xFound, double yFound)
{
	/*	iSMEASURERELEVANT:
	 *		Return 1 if the position found is relevant, given the actual robot pose and the sonars 
	 *      side (front: side = 0, or rear: side = 3), otherwise return 0.
	 */
	
	// Characteristic lengths of the playing area
	double external_walls_max_x = 1.0;
	double external_walls_max_y = 1.5;

	double central_wall_min_x = -0.2;
	double central_wall_max_x = 0.5;
	double central_wall_max_y = 0.1;

	 if ((fabs(xFound) > external_walls_max_x) || (fabs(yFound) > external_walls_max_y))
	 { // if the obstacle detected is located behind the external walls
	 	return 0;
	 } 
	 else if (((xFound < central_wall_max_x) && (xFound > central_wall_min_x)) && (fabs(yFound) < central_wall_max_y))
	 { // if the obstacle detected is the central separation wall
	 	return 0;
	 } 
	 else
	 { // the obstacle detected is an opponent
	 	return 1;
	 } 
}

NAMESPACE_CLOSE();
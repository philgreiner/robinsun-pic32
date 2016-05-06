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
 	//int nb_opponents = ivs->nb_opponents;
	int opponents_detected = 0, dirsonar = 0;
	double opponent_radius = 0.00;        // max distance from a point of the opponent and its center
	double opponents_xpos[4] = {-42.0,-42.0,-42.0,-42.0}; // vector of size 2 since {opponents_detected <= 2}
	double opponents_ypos[4] = {-42.0,-42.0,-42.0,-42.0};	

	// Position of Robinsun
	double x_robinsun 	  = cvs->state->position[0];
	double y_robinsun 	  = cvs->state->position[1];
	double tr             = cvs->state->position[2];
    double phi = 15*M_PI/180.0, cr = cos(tr), sr = sin(tr);

	// Position of each sonar and their respective readings
	double xdistf = 0.1; // distance from the center of Robinsun along the x-axis of the mobile frame 
    double xdistb = 0.14;
	double ydist = 0.075; // distance from the center of Robinsun along the y-axis of the mobile frame
											
	double x_sonars[6] = {x_robinsun + xdistf*cr - ydist*sr, x_robinsun + xdistf*cr,
                          x_robinsun + xdistf*cr + ydist*sr, x_robinsun - xdistb*cr - ydist*sr,
                          x_robinsun - xdistb*cr, x_robinsun - xdistb*cr + ydist*sr};
    
    double y_sonars[6] = {y_robinsun + xdistf*sr + ydist*cr, y_robinsun + xdistf*sr, 
                          y_robinsun + xdistf*sr - ydist*cr, y_robinsun - xdistb*sr + ydist*cr,
                          y_robinsun - xdistb*sr, y_robinsun - xdistb*sr - ydist*cr};
    
	double dist[6];
	for (i=0; i<6; i=i+1)
		dist[i] = cvs->state->avSonar[i];

	double sign[4] = {1.0,1.0,1.0,-1.0};  // used to adapt the contributions of front and rear sonars

	// Condition on the measurements
	double max_dist = 90;	// maximum measurable/significant distance					// TO ADAPT

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
    
	for(i=0; i<6; i=i+3)
	{ // for : i=0 --> front side / i=3 --> rear side
        if (dist[i] < max_dist)
		{ // left
			if (dist[i+1] < max_dist)
			{ // left and center
				if (dist[i+2] < max_dist)
				{ // left, center and right --> opponent in front of the center sonar
					xpos = x_sonars[i+1] + sign[i]*(min(min(dist[i],dist[i+1]),dist[i+2])/100.0 + opponent_radius)*cos(tr);
					ypos = y_sonars[i+1] + sign[i]*(min(min(dist[i],dist[i+1]),dist[i+2])/100.0 + opponent_radius)*sin(tr);
				} // left, center and right
				else
				{ // left and center only --> opponent in front of the left sonar
					xpos = x_sonars[i] + sign[i]*(min(dist[i],dist[i+1])/100.0 + opponent_radius)*cos(tr);
					ypos = y_sonars[i] + sign[i]*(min(dist[i],dist[i+1])/100.0 + opponent_radius)*sin(tr); 
				} // left and center only

				if (isMeasureRelevant(cvs, xpos,ypos))
				{
					opponents_xpos[dirsonar] = xpos;
					opponents_ypos[dirsonar] = ypos;
					opponents_detected += 1;
				}
			} // left and center
			
			else if (dist[i+2] < max_dist)
			{ // left and right --> 2 opponents : one on the left of the left sonar, 
			  //								  the other on the right of the right sonar
				xpos = x_sonars[i] + sign[i]*(dist[i]/100.0 + opponent_radius)*cos(tr+sign[i]*phi) - opponent_radius*sin(tr+sign[i]*phi);
				ypos = y_sonars[i] + sign[i]*(dist[i]/100.0 + opponent_radius)*sin(tr+sign[i]*phi) + opponent_radius*cos(tr+sign[i]*phi); 
				if (isMeasureRelevant(cvs, xpos,ypos))
				{
					opponents_xpos[dirsonar] = xpos;
					opponents_ypos[dirsonar] = ypos;
					opponents_detected += 1;
				}

				xpos = x_sonars[i+2] + sign[i]*(dist[i+2]/100.0 + opponent_radius)*cos(tr-sign[i]*phi) + opponent_radius*sin(tr-sign[i]*phi);
				ypos = y_sonars[i+2] + sign[i]*(dist[i+2]/100.0 + opponent_radius)*sin(tr-sign[i]*phi) - opponent_radius*cos(tr-sign[i]*phi); 

				if (isMeasureRelevant(cvs, xpos,ypos))
				{
					opponents_xpos[dirsonar+1] = xpos;
					opponents_ypos[dirsonar+1] = ypos;
					opponents_detected += 1;
				}
			} // left and right --> 2 opponents ...

			else
			{ // left only --> opponent on the left of the left sonar

				xpos = x_sonars[i] + sign[i]*(dist[i]/100.0 + opponent_radius)*cos(tr+sign[i]*phi) - opponent_radius*sin(tr+sign[i]*phi);
				ypos = y_sonars[i] + sign[i]*(dist[i]/100.0 + opponent_radius)*sin(tr+sign[i]*phi) + opponent_radius*cos(tr+sign[i]*phi);

				if (isMeasureRelevant(cvs, xpos,ypos))
				{
					opponents_xpos[dirsonar] = xpos;
					opponents_ypos[dirsonar] = ypos;
					opponents_detected += 1;
				}
			} // left only
		} // left

		else if (dist[i+2] < max_dist)
		{ // right
			if (dist[i+1] < max_dist)
			{ // right and center
				xpos = x_sonars[i+2] + sign[i]*(min(dist[i+1],dist[i+2])/100.0 + opponent_radius)*cos(tr);
				ypos = y_sonars[i+2] + sign[i]*(min(dist[i+1],dist[i+2])/100.0 + opponent_radius)*sin(tr); 

				if (isMeasureRelevant(cvs, xpos,ypos))
				{
					opponents_xpos[dirsonar] = xpos;
					opponents_ypos[dirsonar] = ypos;
					opponents_detected += 1;
				}
			} // right and center
			
			else
			{ // right only
				xpos = x_sonars[i+2] + sign[i]*(dist[i+2]/100.0 + opponent_radius)*cos(tr-sign[i]*phi) + opponent_radius*sin(tr-sign[i]*phi);
				ypos = y_sonars[i+2] + sign[i]*(dist[i+2]/100.0 + opponent_radius)*sin(tr-sign[i]*phi) - opponent_radius*cos(tr-sign[i]*phi); 

				if (isMeasureRelevant(cvs, xpos,ypos))
				{
					opponents_xpos[dirsonar] = xpos;
					opponents_ypos[dirsonar] = ypos;
					opponents_detected += 1;
				}
			} // right only
		} // right

		else if (dist[i+1] < max_dist)
		{ // center only --> opponent in front of the center sonar
			xpos = x_sonars[i+1] + sign[i]*(dist[i+1]/100.0 + opponent_radius)*cos(tr);
			ypos = y_sonars[i+1] + sign[i]*(dist[i+1]/100.0 + opponent_radius)*sin(tr);
			 
			if (isMeasureRelevant(cvs, xpos,ypos))
			{
				opponents_xpos[dirsonar] = xpos;
				opponents_ypos[dirsonar] = ypos;
				opponents_detected += 1;
			}
		} // center only
        dirsonar = 2;
	} // for : i=0 --> front side / i=3 --> rear side


	//============================================
	// UPDATE THE STATE OF THE STRUCTURE
	//============================================

    cvs->state->nb_opponents_detected = opponents_detected;
    cvs->state->opponent_position[0] = opponents_xpos[0];
    cvs->state->opponent_position[1] = opponents_ypos[0];
    cvs->state->opponent_position[2] = opponents_xpos[1];
    cvs->state->opponent_position[3] = opponents_ypos[1];
    cvs->state->opponent_position[4] = opponents_xpos[2];
    cvs->state->opponent_position[5] = opponents_ypos[2];
    cvs->state->opponent_position[6] = opponents_xpos[3];
    cvs->state->opponent_position[7] = opponents_ypos[3];
}

int isMeasureRelevant(CtrlStruct *cvs, double xFound, double yFound)
{
	/*	iSMEASURERELEVANT:
	 *		Return 1 if the position found is relevant, given the actual robot pose and the sonars 
	 *      side (front: side = 0, or rear: side = 3), otherwise return 0.
	 */
	
	// Characteristic lengths of the playing area
	double external_walls_max_x = 0.9;
	double external_walls_max_y = 1.4;

	double central_wall_min_x = -0.3;
	double central_wall_max_x = 0.4;
	double central_wall_max_y = 0.15;

    if ((fabs(xFound) > external_walls_max_x) || (fabs(yFound) > external_walls_max_y)) // if the obstacle detected is located beyond the external walls
       return 0;
    else if (((xFound < central_wall_max_x) && (xFound > central_wall_min_x)) && (fabs(yFound) < central_wall_max_y)) // if the obstacle detected is the central separation wall
       return 0;
    else if((cvs->state->position[1]/fabs(cvs->state->position[1]) != yFound/fabs(yFound)) && xFound > -0.3 && xFound < 0.4) // don't look through the central wall
       return 0;
//    else if(((cvs->state->position[0]+0.2)/fabs(cvs->state->position[1]+0.25) != (xFound+0.25)/fabs(xFound+0.25)) && fabs(yFound) < 0.6) // don't look through the central bar
//        return 0;
    else if(/*fabs(yFound)<1.4 && fabs(yFound) > 0.75 && */xFound < -0.8) // ignore cabins
       return 0;
    else // the obstacle detected is an opponent
       return 1;
}

NAMESPACE_CLOSE();
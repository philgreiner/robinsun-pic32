/*!
* \file Astar_gr1.cc
* \brief Astar Path planning
*/

#include "Astar_gr1.h"

NAMESPACE_INIT(ctrlGr1);

/*
* Get Path planning
* \param[in] cvs controller main structure ;
* \param[out]
*/
void Astar_get_path(CtrlStruct *cvs)
{
#ifdef ASTAR
	// Set the reference frame to the lower left corner; and multiply to get it in 5cm
	int x = (int)(cvs->state->position[0] * 20 + 21);                       // to adapt the cvs->state->position with reference to the center of the table to the one with reference to the left lower corner (corresponding to the game_map)
	int y = (int)(cvs->state->position[1] * 20 + 31);	                    // shift of reference to lower left corner

	int starting_node;                                                      // The number of the starting node. Will be used for step 3

	// Goal position
	double goal_x_init = cvs->state->goal_position[0] * 20;  // -0.6 *20;   // cvs->state->goal_position[0]*20 ;
	int goal_x = (int)(goal_x_init + 21);                                   // shift of reference to lower left corner

	double goal_y_init = cvs->state->goal_position[1] * 20;  // 0*20;       // cvs->state->goal_position[0]*20 ;

	int goal_y = (int)(goal_y_init + 31);                                   // shift of reference to lower left corner
	int goal_node = goal_x + (goal_y * 42);

    // used for loops
	int xi, yi;

	int G = 10;                         //General value of the cost of a single movement

	int empty = -42;
	int astar_parent_table[43][63];     //Keeps the parent of each node
	int astar_closed_list[43][63];      // Keeps the chosen nodes
	int astar_open_list[43][63];        // Keeps the value of G (the cost from parent node to actual node)

	for (xi = 0; xi < 43; xi++)         // initialise table to empty value
		for (yi = 0; yi < 63; yi++) {
			astar_parent_table[xi][yi] = empty;
			astar_closed_list[xi][yi] = empty;
			astar_open_list[xi][yi] = empty;
		}

	// Step 1 : Add the initial node to the closed list
	starting_node = (x + y * 42);
	astar_closed_list[x][y] = starting_node; // save in the closed list as the node number

	/* Step 2 : Loop while until final cvs->state->position is not in the closed_list
	*		step 2.1 : calculate G for adjacent nodes and add them in the open list
	*		step 2.2 : calculate H for adjacent nodes
	*		step 2.3 : calculate F for adjacent nodes
	*		step 2.4 : save the mother node for the adjacent ones
	*		step 2.5 : Choose lowest F node
	*		step 2.6 : save the chosen node in the closed list and change x and y value with new node
	*/
	while (astar_closed_list[goal_x][goal_y] != goal_node)
	{
		int actual_node = x + y * 42;

		// used to save astar data and reset all when jumping to a new loop
		int G_parent;
		if (actual_node == starting_node) G_parent = 0;
		else G_parent = astar_open_list[x][y];	// Value of G of the parent node (actual node)

		int H;
		int F[3][3]; // from lower left corner with the actual node(x,y) = [1][1]

		for (xi = 0; xi < 3; xi++)
			for (yi = 0; yi < 3; yi++)
				F[xi][yi] = empty;

		for (xi = (x - 1); xi <= (x + 1); xi++)
		{
			for (yi = (y - 1); yi <= (y + 1); yi++) 
			{
				if (xi == x && yi == y);                            // if it is the actual node (disregard)
				else if (cvs->param->game_map[xi][yi] == 0);		// if it is an obstacle (disregard
				else
				{
					// Check old value of G for this adjacent node and compare it to see if an overwritte is needed
					int G_old = astar_open_list[xi][yi];
					int G_new;
					int Replace_G_old = 0;

					if (G_old != empty)									//The node is already in the open list
						if (G_old <= (G_parent + G)) Replace_G_old = 0;		// Check for smallest G value
						else Replace_G_old = 1;
					else Replace_G_old = 1;									//The node is not yet in the open list

					if (Replace_G_old == 1)					// Step 2.1 Start the replacement of G value if needed + Step 2.4 Mother Node
					{
						G_new = G + G_parent;
						astar_open_list[xi][yi] = G_new;			// Add the node to the open list
						astar_parent_table[xi][yi] = actual_node;	// save the parent node number in the table at the child cvs->state->position
					}

					//Step 2.2 H
					H = abs(xi - goal_x)*G + abs(yi - goal_y) *G;		// Manhattan method

					//Step 2.3 F
					F[(xi - x + 1)][(yi - y + 1)] = H + astar_open_list[xi][yi];        // F= H+G
				}// end else
			}// end for yi
		}// end for xi

		// Step 2.5 search the minimum F
		int x_f_min = x - 1;
		int y_f_min = y - 1;
		int minimum = INT_MAX;

		// 2.5.2 Looking through F to find a new minimum if there is one
		int xj, yj, counter = 0;

		for (xj = 0; xj < 3; xj++)
		{
			for (yj = 0; yj < 3; yj++) 
			{
				// Problem when all the node are already in the closed list
				if (astar_closed_list[(x + xj - 1)][(yj + y - 1)] != empty) counter++;				// Avoid to choose its parent node
				else
					if (F[xj][yj] != empty && minimum >= F[xj][yj]) 
					{
						minimum = F[xj][yj];
						x_f_min = xj + x - 1;
						y_f_min = yj + y - 1;
					}//end if
			}//end for yj
		}//end for xj

        // when all the adjacent node are in the closed_list, take the one that has the lowest F
		if (counter == 9)
		{
            #ifdef SIMU_PROJECT
                printf("fail : One of the node you want to reach is in the obstacle zone !\n");
            #endif // SIMU_PROJECT
		}

        // Step 2.6 Change value of x and y and save the chosen node value in close list
		x = x_f_min;
		y = y_f_min;
		astar_closed_list[x][y] = x + (42 * y);
	} // end While loop

/*
* Step 3 : Path is found, now time to extract the path and to put it in a stack
*
*	3.1 Start from cvs->state->goal_position, save it in the stack and check mother node
*	3.2 Save the mother node in the stack and check for his mother node until reaching starting node
*
*/
	int check_node = goal_node;     // = top of the stack
	int parent_node, x_check_node, y_check_node;
	cvs->param->index_path = 0;

	while (check_node != starting_node)
	{
		cvs->param->path[cvs->param->index_path] = check_node;		// save the check_node in the Stack
		x_check_node = check_node % 42;
		y_check_node = (check_node - x_check_node) / 42;

		check_node = astar_parent_table[x_check_node][y_check_node]; // new node to check is the mother node
		cvs->param->index_path++;
	}
	cvs->param->path[cvs->param->index_path] = check_node;		// put the intial starting_node in the path array

	cvs->param->index_path -= 5;
#endif
}//end function

/*
* Function : (Step 4) Read the stack and extract the wheels motors commands (has to be called every cycle)
*
*	4 Loop to extract next top stack until reach goal cvs->state->position
*	4.1 Save x and y coordinates from top stack node and read actual cvs->state->position
*	4.2 While Loop to compare actual cvs->state->position with the next target cvs->state->position go to next (! actual cvs->state->position has not the same reference)
*		4.3.1 Actuate motors as needed
*		4.3.2 Read new actual cvs->state->position
*	End of 4. Stop motors
*/

void Astar_read_path(CtrlStruct *cvs)  // Should be read at each cycle
{
#ifdef ASTAR
	double omega, vlin;

	/* A. Goal data */
	int actual_step = cvs->param->path[cvs->param->index_path];
	int actual_step_x = actual_step % 42;
	int actual_step_y = (actual_step - actual_step_x) / 42;

	int x = cvs->state->position[0] * 20;
	x = (int)x + 21;				// to adapt the cvs->state->position with reference to the center of the table to the one with reference to the left lower corner (corresponding to the game_map)
	int y = cvs->state->position[1] * 20;
	y = (int)y + 31;				// shift of reference to lower left corner
	double theta = cvs->state->position[2];

	double delta_x = actual_step_x - x;
	double delta_y = actual_step_y - y;

	int PRECISION;
	if (cvs->param->index_path == 0) PRECISION = 0;
	else PRECISION = 1;
	double dist = sqrt((delta_x*delta_x) + (delta_y*delta_y));

	/* B. Check if on position and act accordingly  */

	// 1. ***** ON POSITION *****
	if (fabs(dist) <= PRECISION) {
		if (cvs->param->index_path == 0)				// at final destination
		{
			cvs->state->omegaref[R_ID] = 0.0;
			cvs->state->omegaref[L_ID] = 0.0;

			cvs->param->Astar_path_active = 0;
			cvs->param->ready_start_astar = 0;
		}
		else
		{
			cvs->param->index_path = cvs->param->index_path - 3;
			if (cvs->param->index_path < 0) { cvs->param->index_path = 0; }
		} // end else
	} // enf if ************ ON POSITION ****************

	// 2. NOT YET ON POSITION
	else 
	{
		double arcos = acos(delta_x / dist);	// Calculate the angle between the 0rad reference angle and the angle to go to the next point
		double theta_required = (delta_y >= 0) ? arcos : -arcos;
		double delta_theta = theta_required - theta;

		if (delta_theta > M_PI) delta_theta -= 2 * M_PI;
		if (delta_theta < -M_PI) delta_theta += 2 * M_PI;

		omega = -12 * delta_theta;
		vlin = 13;

		if (cvs->param->index_path == 0) vlin = dist * 3;
		if (fabs(delta_theta) < M_PI_4) 
		{
			vlin = (vlin > 13.0) ? (13.0) : (vlin);
			vlin = (vlin < -13.0) ? (-13.0) : (vlin);
			if (fabs(vlin) <= 3) vlin = (vlin / (fabs(vlin)))*2.5;
		}
		else vlin = 0;

		cvs->state->omegaref[R_ID] = vlin - 1.3*omega;
		cvs->state->omegaref[L_ID] = vlin + 1.3*omega;
	} // end else (not on position)
#endif

}
NAMESPACE_CLOSE();

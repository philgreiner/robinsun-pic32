/*!
* \file Astar_gr1.cc
* \brief Astar Path planning
*/

#include "namespace_ctrl.h"
#include "Astar_gr1.h"
#include <stdio.h>
#include "math.h"

#ifdef ASTAR

NAMESPACE_INIT(ctrlGr1);

 /*
 * Get Path planning
 * \param[in] cvs controller main structure ; initial cvs->state->position, target cvs->state->position
 * \param[out]
 */

void Astar_get_path(CtrlStruct *cvs)
{
	// *** Preparation of the variables ***

	double x_init =  cvs->state->position[0] *100;  //  -0.225 *100;
	int x =(int) x_init + 106;				        // to adapt the cvs->state->position with reference to the center of the table to the one with reference to the left lower corner (corresponding to the game_map)
	double  y_init = cvs->state->position[1]*100;   // -1.15 *100;
	int y = (int) y_init + 156;	                    // shift of reference to lower left corner

	int starting_node;			                    // The number of the starting node. Will be used for step 3

	double goal_x_init = -0.6 *100;                 // cvs->state->goal_position[0]*100 ;
	int goal_x = (int) goal_x_init + 106;		    // shift of reference to lower left corner
	double goal_y_init =  0 *100;                   // cvs->state->goal_position[0] * 100;  // 0 *100;					//cvs->state->goal_position[2] *100;
	int goal_y = (int) goal_y_init + 156;		    // shift of reference to lower left corner
	int goal_node = goal_x + (goal_y * 212);

#ifdef DEBUG
	printf("xinit, yinit= %d,%d\n", x, y);
	printf("actual x = %d,actual y = %d\n", x, y);
	printf("goal x = %d,goal y = %d\n", goal_x, goal_y);
#endif // DEBUG

		// used for loops
	int xi;
	int yi;

	int G = 10;	                        //General value of the cost of a single movement

	int null = -858993460;              // 0 for release, - 8588993460 for debugger
	int astar_parent_table[213][313];   //Keeps the parent of each node
	int astar_closed_list[213][313];    // Keeps the chosen nodes
	int astar_open_list[213][313];      // Keeps the value of G (the cost from parent node to actual node)

	for (xi = 0; xi < 213; xi++)	    // initialise table to null value
	{
		for (yi = 0; yi < 313; yi++) {
			astar_parent_table[xi][yi] = null;
			astar_closed_list[xi][yi] = null;
			astar_open_list[xi][yi] = null;
		}
	}

#ifdef DEBUG
	printf("null: parent =%d, closed = %d, open =%d\n", astar_parent_table[0][0], astar_closed_list[0][0], astar_open_list[0][0]);

#endif // DEBUG



	// *** Application of the Astar system *** //

	 // step 1 : Add the initial node to the closed list
		starting_node = (x + y * 212);
		astar_closed_list[x][y] = starting_node; // save in the closed list as the node number

#ifdef DEBUG
		printf("initial node = %d, goal node =%d \n", starting_node, goal_node);
#endif // DEBUG


	 /* step 2 : Loop while until final cvs->state->position is not in the closed_list
	 *		step 2.1 : calculate G for adjacent nodes and add them in the open list
	 *		step 2.2 : calculate H for adjacent nodes
	 *		step 2.3 : calculate F for adjacent nodes
	 *		step 2.4 : save the mother node for the adjacent ones
	 *		step 2.5 : Choose lowest F node
	 *		step 2.6 : save the chosen node in the closed list and change x and y value with new node
	 */
		while ( astar_closed_list[goal_x][goal_y] != goal_node)
		{
			//printf("while");
			// Define variables

				// used for loops
					int xi;
					int yi;
					int actual_node = x + y * 212;

				// used to save astar data and resetted all when jumping to a new loop
					int G_parent;
					if (actual_node == starting_node) { G_parent = 0; }
					else { G_parent = astar_open_list[x][y]; }	// Value of G of the parent node (actual node)

					int H;
					int F[3][3]; // from lower left corner with the actual node(x,y) = [1][1]

			// Check all the cases for the actual cvs->state->position on the map

			if (x == 0) {
				if (y == 0) {		//*** lower left corner ***

					for (xi = x; xi <= (x+1); xi++) {
						for (yi = y; yi <= (y+1); yi++) {

							if (xi == x && yi ==y){}							// if it is the actual node (disregard)
							else if (cvs->param->game_map[xi][yi] == 0) {}		// if it is an obstacle (disregard
							else {

								// Check old value of G for this adjacent node and compare it to see if an overwritte is needed
								int G_old = astar_open_list[xi][yi];
								int G_new;
								int Replace_G_old = 0;

								if (G_old != null) {										//The node is already in the open list
									if (G_old <= (G_parent + G)) { Replace_G_old = 0; }		// Check for smallest G value
									else { Replace_G_old = 1; }
								}
								else { Replace_G_old = 1;}									//The node is not yet in the open list

								// Step 2.1 Start the replacement of G value if needed + Step 2.4 Mother Node
								if (Replace_G_old == 1) {
									G_new = G + G_parent;
									astar_open_list[xi][yi] = G_new;			// Add the node to the open list
									astar_parent_table[xi][yi] = actual_node;	// save the parent node number in the table at the child cvs->state->position
								}
								//else {}

								//Step 2.2 H
								H = fabs((float) xi - goal_x)*G + fabs((float) yi - goal_y) *G;				// Manhattan method

								//Step 2.3 F
								F[(xi - x + 1)][(yi - y + 1)] = H + astar_open_list[xi][yi]; // F= H+G


							}// end else
						}// end for yi

					}// end for xi


				} // end lower left corner

				else if (y == 312) {	//*** upper left corner ***

					for (xi = x; xi <= (x + 1); xi++) {
						for (yi = (y-1) ; yi <= y; yi++) {

							if (xi == x && yi == 312) {}							// if it is the actual node (disregard)
							else if (cvs->param->game_map[xi][yi] == 0) {}		// if it is an obstacle (disregard
							else {

								// Check old value of G for this adjacent node and compare it to see if an overwritte is needed
								int G_old = astar_open_list[xi][yi];
								int G_new;
								int Replace_G_old = 0;

								if (G_old != null) {										//The node is already in the open list
									if (G_old <= (G_parent + G)) { Replace_G_old = 0; }		// Check for smallest G value
									else { Replace_G_old = 1; }
								}
								else { Replace_G_old = 1; }									//The node is not yet in the open list

																							// Step 2.1 Start the replacement of G value if needed + Step 2.4 Mother Node
								if (Replace_G_old == 1) {
									G_new = G + G_parent;
									astar_open_list[xi][yi] = G_new;			// Add the node to the open list
									astar_parent_table[xi][yi] = actual_node;	// save the parent node number in the table at the child cvs->state->position
								}

								//Step 2.2 H
								H = fabs((float) xi - goal_x) *G + fabs((float) yi - goal_y) *G;				// Manhattan method

								//Step 2.3 F
								F[(xi - x + 1)][(yi - y + 1)] = H + astar_open_list[xi][yi]; // F= H+G


							}// end else
						}// end for yi

					}// end for xi

				}// end upper left corner

				else {					// *** left border ***

					for (xi = x; xi <= (x + 1); xi++) {
						for (yi = (y - 1); yi <= (y + 1) ; yi++) {
							if (xi == x && yi == y) {}							// if it is the actual node (disregard)
							else if (cvs->param->game_map[xi][yi] == 0) {}		// if it is an obstacle (disregard
							else {
								// Check old value of G for this adjacent node and compare it to see if an overwritte is needed
								int G_old = astar_open_list[xi][yi];
								int G_new;
								int Replace_G_old = 0;

								if (G_old != null) {										//The node is already in the open list
									if (G_old <= (G_parent + G)) { Replace_G_old = 0; }		// Check for smallest G value
									else { Replace_G_old = 1; }
								}
								else { Replace_G_old = 1; }									//The node is not yet in the open list


																							// Step 2.1 Start the replacement of G value if needed + Step 2.4 Mother Node
								if (Replace_G_old == 1) {
									G_new = G + G_parent;
									astar_open_list[xi][yi] = G_new;			// Add the node to the open list
									astar_parent_table[xi][yi] = actual_node;	// save the parent node number in the table at the child cvs->state->position
								}

								//Step 2.2 H
								H = fabs((float) xi - goal_x)*G + fabs((float) yi - goal_y) *G;				// Manhattan method

																							//Step 2.3 F
								F[(xi - x + 1)][(yi - y + 1)] = H + astar_open_list[xi][yi]; // F= H+G


							}// end else
						}// end for yi

					}// end for xi


				} // end left border
			} //end if (x ==0)

	//***************** end x ==0  *********************

			else if (x == 212) {
				if (y == 0){			// ***  lower right corner ***

					for (xi = (x-1); xi <= x ; xi++) {
						for (yi = y; yi <= (y + 1); yi++) {
							if (xi == x && yi == y) {}							// if it is the actual node (disregard)
							else if (cvs->param->game_map[xi][yi] == 0) {}		// if it is an obstacle (disregard
							else {

								// Check old value of G for this adjacent node and compare it to see if an overwritte is needed
								int G_old = astar_open_list[xi][yi];
								int G_new;
								int Replace_G_old = 0;

								if (G_old != null) {										//The node is already in the open list
									if (G_old <= (G_parent + G)) { Replace_G_old = 0; }		// Check for smallest G value
									else { Replace_G_old = 1; }
								}
								else { Replace_G_old = 1; }									//The node is not yet in the open list

																							// Step 2.1 Start the replacement of G value if needed + Step 2.4 Mother Node
								if (Replace_G_old == 1) {
									G_new = G + G_parent;
									astar_open_list[xi][yi] = G_new;			// Add the node to the open list
									astar_parent_table[xi][yi] = actual_node;	// save the parent node number in the table at the child cvs->state->position
								}

								//Step 2.2 H
								H = fabs((float) xi - goal_x)*G + fabs((float) yi - goal_y) *G;				// Manhattan method

								//Step 2.3 F
								F[(xi - x + 1)][(yi - y + 1)] = H + astar_open_list[xi][yi]; // F= H+G


							}// end else
						}// end for yi

					}// end for xi

				} // end lower right corner


				else if (y == 312){	// ***  upper right corner ***

					for (xi = (x -1); xi <= x; xi++) {
						for (yi = (y - 1); yi <= y; yi++) {
							if (xi == x && yi == y) {}							// if it is the actual node (disregard)
							else if (cvs->param->game_map[xi][yi] == 0) {}		// if it is an obstacle (disregard
							else {

								// Check old value of G for this adjacent node and compare it to see if an overwritte is needed
								int G_old = astar_open_list[xi][yi];
								int G_new;
								int Replace_G_old = 0;

								if (G_old != null) {										//The node is already in the open list
									if (G_old <= (G_parent + G)) { Replace_G_old = 0; }		// Check for smallest G value
									else { Replace_G_old = 1; }
								}
								else { Replace_G_old = 1; }									//The node is not yet in the open list

																							// Step 2.1 Start the replacement of G value if needed + Step 2.4 Mother Node
								if (Replace_G_old == 1) {
									G_new = G + G_parent;
									astar_open_list[xi][yi] = G_new;			// Add the node to the open list
									astar_parent_table[xi][yi] = actual_node;	// save the parent node number in the table at the child cvs->state->position
								}

								//Step 2.2 H
								H = fabs((float) xi - goal_x)*G + fabs((float) yi - goal_y) *G;				// Manhattan method

																							//Step 2.3 F
								F[(xi - x + 1)][(yi - y + 1)] = H + astar_open_list[xi][yi]; // F= H+G


							}// end else
						}// end for yi

					}// end for xi

				} // end upper right corner


				else{					// ***  right border ***

					for (xi = (x-1); xi <= x; xi++) {
						for (yi = (y - 1); yi <= (y + 1); yi++) {
							if (xi == x && yi == y) {}							// if it is the actual node (disregard)
							else if (cvs->param->game_map[xi][yi] == 0) {}		// if it is an obstacle (disregard
							else {

								// Check old value of G for this adjacent node and compare it to see if an overwritte is needed
								int G_old = astar_open_list[xi][yi];
								int G_new;
								int Replace_G_old = 0;

								if (G_old != null) {										//The node is already in the open list
									if (G_old <= (G_parent + G)) { Replace_G_old = 0; }		// Check for smallest G value
									else { Replace_G_old = 1; }
								}
								else { Replace_G_old = 1; }									//The node is not yet in the open list

																							// Step 2.1 Start the replacement of G value if needed + Step 2.4 Mother Node
								if (Replace_G_old == 1) {
									G_new = G + G_parent;
									astar_open_list[xi][yi] = G_new;			// Add the node to the open list
									astar_parent_table[xi][yi] = actual_node;	// save the parent node number in the table at the child cvs->state->position
								}

								//Step 2.2 H
								H = fabs((float) xi - goal_x)*G + fabs((float) yi - goal_y) *G;				// Manhattan method

																							//Step 2.3 F
								F[(xi - x + 1)][(yi - y + 1)] = H + astar_open_list[xi][yi]; // F= H+G


							}// end else
						}// end for yi

					}// end for xi


				} // end right border

			} // end (x ==212)

	//***************** end x == 212 *********************

			else {						// case (x!=0 && x!=212)
				if (y == 0) {			// *** lower border ***

					for (xi = (x - 1) ; xi <= (x + 1); xi++) {
						for (yi = y; yi <= (y + 1); yi++) {
							if (xi == x && yi == y) {}							// if it is the actual node (disregard)
							else if (cvs->param->game_map[xi][yi] == 0) {}		// if it is an obstacle (disregard
							else {
								// Check old value of G for this adjacent node and compare it to see if an overwritte is needed
								int G_old = astar_open_list[xi][yi];
								int G_new;
								int Replace_G_old = 0;

								if (G_old != null) {										//The node is already in the open list
									if (G_old <= (G_parent + G)) { Replace_G_old = 0; }		// Check for smallest G value
									else { Replace_G_old = 1; }
								}
								else { Replace_G_old = 1; }									//The node is not yet in the open list

																							// Step 2.1 Start the replacement of G value if needed + Step 2.4 Mother Node
								if (Replace_G_old == 1) {
									G_new = G + G_parent;
									astar_open_list[xi][yi] = G_new;			// Add the node to the open list
									astar_parent_table[xi][yi] = actual_node;	// save the parent node number in the table at the child cvs->state->position
								}

								//Step 2.2 H
								H = fabs((float) xi - goal_x)*G + fabs((float) yi - goal_y) *G;				// Manhattan method

																							//Step 2.3 F
								F[(xi - x + 1)][(yi - y + 1)] = H + astar_open_list[xi][yi]; // F= H+G


							}// end else
						}// end for yi

					}// end for xi

				} // end lower border


				else if (y==312){		// ***  upper border ***

					for (xi = (x - 1); xi <= (x + 1); xi++) {
						for (yi = (y - 1); yi <= y; yi++) {
							if (xi == x && yi == y) {}							// if it is the actual node (disregard)
							else if (cvs->param->game_map[xi][yi] == 0) {}		// if it is an obstacle (disregard
							else {
								// Check old value of G for this adjacent node and compare it to see if an overwritte is needed
								int G_old = astar_open_list[xi][yi];
								int G_new;
								int Replace_G_old = 0;

								if (G_old != null) {										//The node is already in the open list
									if (G_old <= (G_parent + G)) { Replace_G_old = 0; }		// Check for smallest G value
									else { Replace_G_old = 1; }
								}
								else { Replace_G_old = 1; }									//The node is not yet in the open list

																							// Step 2.1 Start the replacement of G value if needed + Step 2.4 Mother Node
								if (Replace_G_old == 1) {
									G_new = G + G_parent;
									astar_open_list[xi][yi] = G_new;			// Add the node to the open list
									astar_parent_table[xi][yi] = actual_node;	// save the parent node number in the table at the child cvs->state->position
								}

								//Step 2.2 H
								H = fabs((float) xi - goal_x)*G + fabs((float) yi - goal_y) *G;				// Manhattan method

																							//Step 2.3 F
								F[(xi - x + 1)][(yi - y + 1)] = H + astar_open_list[xi][yi]; // F= H+G


							}// end else
						}// end for yi

					}// end for xi

				} // end upper border


				else {					// ***  in the grid (no border, no corner) ***

					for (xi = (x - 1); xi <= (x + 1); xi++) {
						for (yi = (y - 1); yi <= (y+1) ; yi++) {
							if (xi == x && yi == y) {
							}							// if it is the actual node (disregard)
							else if (cvs->param->game_map[xi][yi] == 0) {}		// if it is an obstacle (disregard
							else {
								// Check old value of G for this adjacent node and compare it to see if an overwritte is needed
								int G_old = astar_open_list[xi][yi];
								int G_new;
								int Replace_G_old = 0;

								if (G_old != null) {										//The node is already in the open list
									if (G_old <= (G_parent + G)) { Replace_G_old = 0; }		// Check for smallest G value
									else { Replace_G_old = 1;}
								}
								else { Replace_G_old = 1; }									//The node is not yet in the open list

																// Step 2.1 Start the replacement of G value if needed + Step 2.4 Mother Node
								if (Replace_G_old == 1) {
									G_new = G + G_parent;
									astar_open_list[xi][yi] = G_new;			// Add the node to the open list
									astar_parent_table[xi][yi] = actual_node;	// save the parent node number in the table at the child cvs->state->position
								}
								else{}
								//Step 2.2 H
								H = fabs((float) xi - goal_x)*G + fabs((float) yi - goal_y) *G;				// Manhattan method

								//Step 2.3 F
								F[(xi - x + 1)][(yi - y + 1)] = H + astar_open_list[xi][yi]; // F= H+G
#ifdef DEBUG
								printf("node = %d, F = %d\n", xi + yi * 212, F[(xi - x + 1)][(yi - y + 1)]);

#endif // DEBUG

							}// end else
						}// end for yi

					}// end for xi

				}// end in the grid

			}//end (x!=0 & x!=212)

	 //***************** end x!=0 & x!=212 *********************
	 //** end of all cases


			 //2.5 search the minimum F
			int x_f_min;
			int y_f_min;
			int minimum;

			// 2.5.1 Starting point of the search in F depending on the corners values
			if(F[0][0] != null  && astar_closed_list[x-1][y-1] == null){
				minimum = F[0][0];
				x_f_min = 0 + x - 1;
				y_f_min = 0 + y - 1;
			}		// case if the lower left corner of the actual node was calculated (not obstacle)
			else if (F[0][1] != null && astar_closed_list[x - 1][y] == null) {
				minimum = F[0][1];
				x_f_min = 0 + x - 1;
				y_f_min = 1 + y - 1;
			}		// take the left position as reference
			else if (F[0][2] != null && astar_closed_list[x - 1][y + 1] == null) {
				minimum = F[0][2];
				x_f_min = 0 + x - 1;
				y_f_min = 2 + y - 1;
			}		// take the upper left corner as reference
			else if (F[1][2] != null && astar_closed_list[x][y + 1] == null) {
				minimum = F[1][2];
				x_f_min = 1 + x - 1;
				y_f_min = 2 + y - 1;
			}		// take the upper position as reference
			else if(F[2][2] != null && astar_closed_list[x +1][y +1] == null) {
				minimum = F[2][2];
				x_f_min = 2 + x - 1;
				y_f_min = 2 + y - 1;
			}		 // take upper right corner as reference
			else if (F[2][1] != null && astar_closed_list[x + 1][y] == null) {
				minimum = F[2][1];
				x_f_min = 2 + x - 1;
				y_f_min = 1 + y - 1;
			}		// take right position as reference
			else if (F[2][0] != null && astar_closed_list[x + 1][y-1] == null) {
				minimum = F[2][0];
				x_f_min = 2 + x - 1;
				y_f_min = 0 + y - 1;
			}		//  take lower right corner as reference
			else {
				minimum = F[1][0];
				x_f_min = 1 + x - 1;
				y_f_min = 0 + y - 1;
			}		// take lower position as reference

#ifdef DEBUG
			printf("line 519 : x_fmin = %d, y_fmin = %d\n, node = %d\n", x_f_min, y_f_min, x_f_min + y_f_min * 212);
#endif // DEBUG

			// 2.5.2 Looking through F to find a new minimum if there is one

			int xj;
			int yj;

			for (xj = 0; xj < 3; xj++)
			{
				for (yj = 0; yj < 3; yj++) {
					if (F[xj][yj] != null && minimum >= F[xj][yj]) {
						if(astar_closed_list[(x+xj-1)][(yj+y-1)] != null ){}				// Avoid to choose its parent node
						else {
							minimum = F[xj][yj];
							x_f_min = xj + x - 1;
							y_f_min = yj + y - 1;
						}
					}// end if
				}//end for yj
			}//end for xj
#ifdef DEBUG
printf("line 535 (after min search) : x_fmin = %d, y_fmin = %d\n, node = %d\n", x_f_min, y_f_min, x_f_min + y_f_min * 212);
#endif // DEBUG

			// 2.6 Change value of x and y and save the chosen node value in close list

			x = x_f_min;
			y = y_f_min;

			astar_closed_list[x][y] = x + (212 * y);

			// 2.7 Reset the F table to null value

			for (xj = 0; xj < 3; xj++)
			{
				for (yj = 0; yj < 3; yj++) {
					F[xj][yj] = null;
				}
			}

		} // end While loop



	/*
	* Step 3 : Path is found, now time to extract the path and to put it in a stack
	*
	*	3.1 Start from cvs->state->goal_position, save it in the stack and check mother node
	*	3.2 Save the mother nood in the stack and check for his mother nood until reaching starting node
	*
	*/

		int check_node = goal_node; // = top of the stack
		int parent_node;
		int x_check_node;
		int y_check_node;


		cvs->param->index_path = 0;

		while (check_node != starting_node)
		{

			cvs->param->path[cvs->param->index_path] = check_node;		// save the check_node in the Stack

			x_check_node = check_node % 212;
			y_check_node = (check_node - x_check_node) / 212;

#ifdef DEBUG
			printf("x_check_node, y_check_node, check_node =%d,%d,%d\n", x_check_node, y_check_node, check_node);
			printf("astar_parent_table[x_check_node][y_check_node] = %d\n", astar_parent_table[x_check_node][y_check_node]);
#endif // DEBUG


			check_node = astar_parent_table[x_check_node][y_check_node]; // new nood to check is the mother node
			cvs->param->index_path++;
		}
			cvs->param->path[cvs->param->index_path] = check_node;		// put the intial starting_node in the path array

} //end function


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

	// A. Prepare data
	int actual_step = cvs->param->path[cvs->param->index_path];
	int actual_step_x = actual_step % 212;
	int actual_step_y = (actual_step - actual_step_x) / 212;

#ifdef DEBUG
	printf("check_node (actual_step) = %d\n", actual_step);
	printf("actual_step_x = %d, actual_step_y = %d\n", actual_step_x, actual_step_y);
#endif // DEBUG


	int x = cvs->state->position[0] *100;
	x = (int) x + 106;				// to adapt the cvs->state->position with reference to the center of the table to the one with reference to the left lower corner (corresponding to the game_map)
	int y = cvs->state->position[1] *100 ;
	y = (int) y + 156;				// shift of reference to lower left corner
	double theta = cvs->state->position[2] ;

#ifdef DEBUG
	printf("x =%d, y= %d, theta = %lf\n", x, y, theta);
#endif // DEBUG


	double delta_x = actual_step_x - x;
	double delta_y = actual_step_y - y;

	// B. Check if on position and act accordingly

	// 1. ***** ON POSITION *****
	if (delta_x == 0 && delta_y == 0) {

		if (cvs->param->index_path == 0)				// at final destination
		{
		#ifdef DEBUG
			printf("finito");
		#endif // DEBUG

		}
		else {
			cvs->param->index_path = cvs->param->index_path - 1;
			#ifdef DEBUG
				printf("line 674 : check_node = %d\n", cvs->param->path[cvs->param->index_path]);
			#endif // DEBUG
	}

	} // enf if ************ ON POSITION ****************


	// 2. NOT YET ON POSITION
	else {

		// Calculate angle divergence

		double hypotenuse = sqrt((delta_x*delta_x) + (delta_y*delta_y));
		double arcos = acos(delta_x / hypotenuse);	// Calculate the angle between the 0rad reference angle and the angle to go to the next point

		double theta_required;

		if (delta_y >= 0) {					// positive angle between 0 and PI
			theta_required = arcos;
		}
		else {								// negative angle between -0 and -PI
			theta_required = -arcos;
		}

		double delta_theta = theta_required - theta;

//#ifdef DEBUG
		printf("delta_x= %lf, delta_y=%lf, theta_req =%lf, delta_theta = %lf\n", delta_x, delta_y, theta_required, delta_theta);
		printf("arcos = %lf\n", arcos);
//#endif // DEBUG

		// Different cases

		double fast_rotation = 30;//0.13*2;
		double slow_rotation = 10;// 0.13;
		double forward_speed = 40;

			// Orientation
		if (delta_theta > 0.25 && delta_theta <= M_PI_4)
		{
			cvs->state->omegaref[R_ID] = slow_rotation * delta_theta;
			cvs->state->omegaref[L_ID] = -slow_rotation * delta_theta;
		}

		else if (delta_theta > M_PI_4 && delta_theta <= M_PI)
		{
			cvs->state->omegaref[R_ID] = fast_rotation * delta_theta;
			cvs->state->omegaref[L_ID] = -fast_rotation * delta_theta;
		}

		else if (delta_theta < 0.25 && delta_theta >= -M_PI / 4)
		{
			cvs->state->omegaref[R_ID] = -slow_rotation * delta_theta;
			cvs->state->omegaref[L_ID] = slow_rotation * delta_theta;
		}

		else if (delta_theta < -M_PI_4 && delta_theta > -M_PI)
		{
			cvs->state->omegaref[R_ID] = -fast_rotation * delta_theta;
			cvs->state->omegaref[L_ID] = fast_rotation * delta_theta;
		}

		// GO forward

		else
		{
			cvs->state->omegaref[R_ID] = forward_speed * hypotenuse;
			cvs->state->omegaref[L_ID] = forward_speed * hypotenuse;

		} // end of GO forward

	} // end if (not on position)


}


NAMESPACE_CLOSE();

#endif
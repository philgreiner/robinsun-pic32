/*!
* \file maps_design.cc
* \brief drawing the map in a table representation with value 1 for clean space ; 0 for obstacles
*/

#ifndef _SIMU_GAME_GR1_H_
#define _SIMU_GAME_GR1_H_

#include "map_design_gr1.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

NAMESPACE_INIT(ctrlGr1);

 /*
 * Drawing the map, hardcoded from the picture received of the game table
 * value 1 for clean space ; 0 for obstacles 
 * (called at initial position before starting the game)
 * \param[in] cvs controller main structure
 * \param[out] game_map[][]
 */

// To do : Check if not necessary to add the robot radius to the edge of the table as an obstacle zone

void game_map_drawing(CtrlStruct *cvs)
{

#ifdef ASTAR
	// All values are in [cm]
	int x;
	int y;
	int radius_robot = 13;

	for (x = 0; x <= 212; x++)
	{
		for (y = 0; y <= 312; y++)
		{
		 // *** 1/ Block bar ***
			if (x <= (20+radius_robot) ) { // Close to violet cabin

				if (y >= (86 - radius_robot) && y <= (88 + radius_robot)) {	
						cvs->param->game_map[x][y] = 0;	
					}// end if 
				
				// Close to green cabin
				else if (y >= (224 - radius_robot) && y <= (226 + radius_robot)) {
						cvs->param->game_map[x][y] = 0;

					}// end else if 

				else cvs->param->game_map[x][y] = 1;

			} // end if  *** 1/ Block bar ***


	    // *** 2/ central bar vertical ***
			else if (x >= (81 - radius_robot) && x <= (83 + radius_robot) ) {


				if (y >= (96 - radius_robot) && y <= (216 + radius_robot)) {
						cvs->param->game_map[x][y] = 0;

					}// if y
				
				else cvs->param->game_map[x][y] = 1;

			} // end else if *** 2/ central bar vertical ***


	   // *** 3/ central bar horizontal ***
			else if (x >= (81 - radius_robot) && x <= (141 + radius_robot)) {

				if (y >= (154 - radius_robot) && y <= (159 + radius_robot)) {
					cvs->param->game_map[x][y] = 0;

				}// end if y

				else cvs->param->game_map[x][y] = 1;

			} // end else if *** 3/ central bar horizontal ***

	 

	   // *** 4/ Lower and Upper right corner steps ***
			else if (x >= (187 - radius_robot) && x <= 212) {
	
				if (y >= (287 - radius_robot) && y <= 312) {  // upper right corner
					cvs->param->game_map[x][y] = 0;
				}// end if y

				else if (y <= (25 + radius_robot)) {		 // lower right corner
					cvs->param->game_map[x][y] = 0;

				}// end else if y


				else cvs->param->game_map[x][y] = 1;

			} // end else if *** 4/ Lower and Upper right corner steps ***


	 // *** 5/ Other places
			else cvs->param->game_map[x][y] = 1;

		}// end for y

	}// end for x

#ifdef SIMU_PROJECT
	printf("The game_map is in the robot database\n");
#endif // SIMU_PROJECT


#endif
} //end function

						

NAMESPACE_CLOSE();

#endif
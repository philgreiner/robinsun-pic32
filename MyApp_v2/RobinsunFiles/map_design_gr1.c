/*!
* \file maps_design.cc
* \brief drawing the map in a table representation with value 1 for clean space ; 0 for obstacles
*/

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
	int radius_robot = 3;

	for (x = 0; x <= 42; x++)
	{
		for (y = 0; y <= 62; y++)
		{
		// *** 0/ borders
			if (x < 1 || x > 41 || y < 1 || y > 61) {
				cvs->param->game_map[x][y] = 0;
			}
            
		 // *** 1/ Block bar ***
			else if (x <= (4+radius_robot) ) { // Close to violet cabin
				if (y >= (17 - radius_robot) && y <= (18 + radius_robot)) {	
						cvs->param->game_map[x][y] = 0;	
					}// end if 
				// Close to green cabin
				else if (y >= (45 - radius_robot) && y <= (46 + radius_robot)) {
						cvs->param->game_map[x][y] = 0;

					}// end else if 
				else cvs->param->game_map[x][y] = 1;
			} // end if  *** 1/ Block bar ***

	    // *** 2/ central bar vertical ***
			else if (x >= (18 - radius_robot) && x <= (17 + radius_robot) ) {
				if (y >= (19 - radius_robot) && y <= (43 + radius_robot)) {
						cvs->param->game_map[x][y] = 0;
				}
				else cvs->param->game_map[x][y] = 1;
			} // end else if *** 2/ central bar vertical ***

	   // *** 3/ central bar horizontal ***
			else if (x >= (16 - radius_robot) && x <= (29 + radius_robot)) {
				if (y >= (31 - radius_robot) && y <= (31 + radius_robot)) {
					cvs->param->game_map[x][y] = 0;
				}
				else cvs->param->game_map[x][y] = 1;
			} // end else if *** 3/ central bar horizontal ***

	   // *** 4/ Lower and Upper right corner steps ***
			else if (x >= (37 - radius_robot) && x <= 42) {
				if (y >= (57 - radius_robot) && y <= 62) {  // upper right corner
					cvs->param->game_map[x][y] = 0;
				}
				else if (y <= (3 + radius_robot)) {		 // lower right corner
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
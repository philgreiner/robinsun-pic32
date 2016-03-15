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
 * \param[in] cvs controller main structure
 */

void game_map_drawing(CtrlStruct *cvs)
{
	// All values are in [mm]
	int x;
	int y;
	int radius_robot = 130;

	for (x = 0; x <= 2124; x++)
	{
		for (y = 0; x <= 3124; y++)
		{
		 // *** 1/ Block bar ***
			if (x <= (200+radius_robot) ) {
				
				// Close to violet cabin
				if (y >= (862 - radius_robot) && y <= (884 + radius_robot)) {	
						cvs->param->game_map[x][y] = 0;
					}// end if 
				
				// Close to green cabin
				else if (y >= (2240 - radius_robot) && y <= (2262 + radius_robot)) {
						cvs->param->game_map[x][y] = 0;
					}// end else if 

				else cvs->param->game_map[x][y] = 1;

			} // end if  *** 1/ Block bar ***


	    // *** 2/ central bar vertical ***
			else if (x >= (812 - radius_robot) && x <= (834 + radius_robot) ) {

				if (y >= (962 - radius_robot) && y <= (2162 + radius_robot)) {
						cvs->param->game_map[x][y] = 0;
					}// if y
				
				else cvs->param->game_map[x][y] = 1;

			} // end else if *** 2/ central bar vertical ***


	   // *** 3/ central bar horizontal ***
			else if (x >= (812 - radius_robot) && x <= (1412 + radius_robot)) {

				if (y >= (1538 - radius_robot) && y <= (1586 + radius_robot)) {
					cvs->param->game_map[x][y] = 0;
				}// end if y

				else cvs->param->game_map[x][y] = 1;

			} // end else if *** 3/ central bar horizontal ***

	 

	   // *** 4/ Upper right corner steps ***
			else if (x >= (1874 - radius_robot) && x <= 2124) {

				if (y >= (2874 - radius_robot) && y <= 3124) {
					cvs->param->game_map[x][y] = 0;
				}// end if y

				else cvs->param->game_map[x][y] = 1;

			} // end else if *** 4/ Upper right corner steps ***


	  // *** 5/ Lower right corner steps ***
			else if (x >= (1874 - radius_robot) && x <= 2124) {

				if (y <= (250 + radius_robot) ) {
					cvs->param->game_map[x][y] = 0;
				}// end if y

				else cvs->param->game_map[x][y] = 1;

			} // end else if *** 5/ Lower right corner steps  ***



		}// end for y

	}// end for x


} //end function

						

NAMESPACE_CLOSE();

#endif
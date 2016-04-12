/* 
 * File:   ctrlstruct.h
 * Author: pgreiner
 *
 * Created on 3 mars 2016, 16:47
 */

#ifndef CTRLSTRUCT_H
#define	CTRLSTRUCT_H

#include "MyCtrlIO.h"
#include <stdlib.h>

typedef struct CtrlState
{
	double errorIntR, errorIntL;
	double Ki, Kp;
	double lastT;
	double position[3];
	double initial_pos[3];
	double position_odo[3];
	
#ifdef ROBOTICS
    double position_triang[3];
	double beacons[6];
#endif
#ifdef MECHATRONICS
    
#endif

	double k_att;

	double obstacle_edges[2162];    // Coordinates of the table edges, under the format (x,y). Precision of 1 cm
	int nb_edges;                   // Number of points in edges
	double k_edge;
	double rho_edge;

	double obstacle_center[722];    // Coordinates of the central obstacle
	int nb_center;                  // Number of points in center
	double k_center;
	double rho_center;

	double K_SpeedX;                 // Coefficients of proportionnality between speed and force for the potential field approach
	double K_SpeedY;

	int game_map[2124][3124];		//Game map draws as a table. game_map[x=0][y=0] = bottom left corner near to violet cabin

} CtrlState;

/// Main controller structure
typedef struct CtrlStruct
{
	CtrlIn *inputs;   ///< controller inputs
	CtrlOut *outputs; ///< controller outputs
	CtrlState *state; ///< controller state
} CtrlStruct;

// function prototypes
CtrlStruct* init_CtrlStruct(CtrlIn *inputs, CtrlOut *outputs);
void free_CtrlStruct(CtrlStruct *cvs);


#endif	/* CTRLSTRUCT_H */


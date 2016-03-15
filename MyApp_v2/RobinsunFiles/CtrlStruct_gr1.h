/*!
 * \file CtrlStruct_gr1.h
 * \brief Controller main structure
 */

#ifndef _CTRL_STRUCT_GR1_H_
#define _CTRL_STRUCT_GR1_H_

#include "ctrl_io.h"
#include "namespace_ctrl.h"
#define POTENTIAL

NAMESPACE_INIT(ctrlGr1);

typedef enum {WAIT_FOR_START, GOTO_OBJ, WAIT_TO_TAKE, WAIT_FOR_DESTINATION} strategystate_t;
typedef enum {OBJ0, OBJ1, OBJ2, OBJ3, OBJ4, OBJ5, OBJ6, BASE} objectives;
typedef enum {NOTDONE, DONE} STATUS;

typedef struct CtrlState
{
    double omegaref[2];
	double errorIntR, errorIntL;
	double lastT;

	double position[3];
	double position_odo[3];
	double position_triang[3];
	double opponent_position[4];
	double goal_position[3];

	//Kalman filter parameters
	double covariance[3][3];
	double covariance_odo[3][3];
	double covariance_triang[6][6];
	double alpha[3];
    double r[3];

	// Strategy parameters
	strategystate_t   strategy_state;
	objectives		  next_objective;
	int				  done_objectives[7];
	int				  objectives_on_robot;
	double			  timer; 
} CtrlState;

typedef struct CtrlParam
{
	// Parameters of the playing area
	double R_beacon;				// Radius of the beacons
	double initial_pos[3];
	double beacons[6];

	// Non-deterministic parameters of the motor drive and the wheel-floor interaction
	double kr;
	double kl;

	// Wheel controller parameters
	double Ki, Kp;
#ifdef POTENTIAL
	double obstacle_center[722];    // Coordinates of the central obstacle
#endif
	int nb_center;                  // Number of points in center
	double k_center;
	double rho_center;
	double k_att;
#ifdef POTENTIAL
	double obstacle_edges[2210];    // Coordinates of the table edges, under the format (x,y). Precision of 1 cm
#endif
	int nb_edges;                   // Number of points in edges
	double k_edge;
	double rho_edge;

	double K_SpeedX;                 // Coefficients of proportionnality between speed and force for the potential field approach
	double K_SpeedRot;

	double k_att_conic;
	double k_att_quad;
	double rho_att;

	//int game_map[2124][3124];		 // Game map draws as a table. game_map[x=0][y=0] = bottom left corner near to violet cabin
} CtrlParam;

/// Main controller structure
typedef struct CtrlStruct
{
	CtrlIn *inputs;   ///< controller inputs
	CtrlOut *outputs; ///< controller outputs
	CtrlState *state; ///< controller state
	CtrlParam *param; ///< controller parameters
} CtrlStruct;

// function prototypes
CtrlStruct* init_CtrlStruct(CtrlIn *inputs, CtrlOut *outputs);
void free_CtrlStruct(CtrlStruct *cvs);

NAMESPACE_CLOSE();

#endif

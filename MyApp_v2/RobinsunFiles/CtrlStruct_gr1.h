/*!
 * \file CtrlStruct_gr1.h
 * \brief Controller main structure
 */

#ifndef _CTRL_STRUCT_GR1_H_
#define _CTRL_STRUCT_GR1_H_

#include "ctrl_io.h"
#include "namespace_ctrl.h"
#include <stddef.h>
#include <stdlib.h>

/* --- KALMAN or ODOMETRY or TRIANGULATION --- */
//#define KALMAN
#define ODO_ONLY
//#define TRIANG_ONLY

/* --- POTENTIAL or ASTAR --- */
//#define POTENTIAL
#define ASTAR

NAMESPACE_INIT(ctrlGr1);

typedef enum {CALIBRATEY, GOTOY, TURN, CALIBRATEX, GOHOMEX, GOHOMETURN, GOHOMEY} calibratestate;
typedef enum {WAIT_FOR_START, GOTO_OBJ, WAIT_TO_TAKE, WAIT_FOR_DESTINATION} strategystate_t;
typedef enum {CALIBRATE, WAIT_FOR_START1, BLOCKS_FRONT, CABINS, BLOCKS_DUNE_1, BLOCKS_DUNE_2, BLOCKS_CABINS, FISH, PARASOL, STOP} robinsun_competition;
typedef enum {NOTDONE1, DELAYED, DONE1} state;
typedef enum {UNCLAMPED, CLAMPED, OPEN, CLOSED} clampstate_t;

typedef enum {OBJ0, OBJ4, OBJ2, OBJ1, OBJ3, OBJ5, OBJ6, BASE} objectives;
typedef enum {NOTDONE, DONE} STATUS;

typedef struct CtrlState
{
    double omegaref[2];
	double errorIntR, errorIntL;
	double lastMesR[10], lastMesL[10];
    double avSpeedR, avSpeedL;
	double lastT;
    double avSonar[6];
    double prevAvSonar[6];
    double lastMesSonar[6][10];
    
	double position[3];
	double position_odo[3];
	double position_triang[3];
	double opponent_position[4];
    int    nb_opponents_detected;
	double goal_position[3];

	//Potential field & Astar orientation error
	double errorAngle;
	double lastT_pot;
	double lastT_astar;

	//Kalman filter parameters
	double covariance[3][3];
	double covariance_odo[3][3];
	double covariance_triang[6][6];
	double alpha[3];
    double r[3];

	// Strategy parameters
	strategystate_t   strategy_state;
	objectives		  next_objective;
    state             objectives[10];
    robinsun_competition current_objective;
    int               current_action_progress;
	int				  done_objectives[7];
	int				  objectives_on_robot;
	double			  timer;
    int               direction;

	// Calibrate initially
	calibratestate    calibration;
    
    // State of functional modules
    double            clamp_opening;
    clampstate_t      clamp;
} CtrlState;

typedef struct CtrlParam{
	// Parameters of the playing area
	double initial_pos[3];
	double beacons[6];

    double refspeed;
	// Non-deterministic parameters of the motor drive and the wheel-floor interaction
	double kr;
	double kl;

	// Wheel controller parameters
	double Ki[2], Kp[2];
	#ifdef POTENTIAL
	    double Ki_pot, Kp_pot;
		double obstacle_center[100];    // Coordinates of the central obstacle
		int nb_center;                  // Number of points in center
		double k_center;
		double rho_center;
		double k_att;

		double obstacle_edges[260];    // Coordinates of the table edges, under the format (x,y). Precision of 1 cm
		int nb_edges;                   // Number of points in edges
		double k_edge;
		double rho_edge;

		double obstacle_purpleZone[28];
		int nb_purple;                   // Number of points in edges
		double k_purple;
		double rho_purple;

		double obstacle_greenZone[28];
		int nb_green;                   // Number of points in edges
		double k_green;
		double rho_green;
        
        #ifdef SIMU_PROJECT
            double obstacle_robot1[42];
            int nb_robot1;
            double k_robot1;
            double rho_robot1;

            double obstacle_robot2[42];
            int nb_robot2;
            double k_robot2;
            double rho_robot2;
		#endif

		double k_att_conic;
		double k_att_quad;
		double rho_att;
	#endif

	#ifdef ASTAR
		int game_map[43][63];			//Game map draws as a table. game_map[x=0][y=0] = bottom left corner near to violet cabin

		int ready_start_astar;			// To be sure to have an objective define
		int path[100];					// Stack used to save all the path steps
		int index_path;					// Show in which path[] position we have to write/read the top stack item
		int Astar_path_active;			// =0 when no path was calculated, =1 when path is calculated

		double Ki_astar, Kp_astar;
	#endif

} CtrlParam;

/// Main controller structure
typedef struct CtrlStruct {
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

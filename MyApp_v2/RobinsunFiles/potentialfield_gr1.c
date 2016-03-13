/*!
* \file potentialfield_gr1.cc
* \brief File description
*/

#include "CtrlStruct_gr1.h"
#include <math.h>
#include "potentialfield_gr1.h"

/*! \brief Allocates the coordinates of an obstacle edge
*
* \param[in] qstart, qend, step, start_index, qconst : starting coordinate, ending coordinate, value of the step,
*                                                      index to start in array obstacle, value of the constant coordinate
* \param[out] outputs written in obstacle;
*/
int edge_allocate(double *obstacle, double qstart, double qend, double step, double qconst, int XY, int start_index)
{
	int n_iter = (int) ceil(fabs(qstart - qend) / step);     // Number of iterations to perform
	int index = start_index;

	if (qstart > qend) {
		step = -step;
	}

	int i;
	for (i = 0; i<n_iter + 1; i = i + 1)
	{
		if (XY == 0) { // The variable coordinate is along x
			obstacle[index] = qstart + (i*step);
			obstacle[index + 1] = qconst;
		}
		else {
			obstacle[index + 1] = qstart + (i*step);
			obstacle[index] = qconst;
		}
		index = index + 2;
	}
	return index - 2;
}

/*! \brief Allocates the coordinates of a diagonal (45 deg) obstacle edge
*
* \param[in] qstartx, qendx, qstarty, qendy, step, start_index: starting coordinate, ending coordinate, value of the step,
*                                                      index to start in array obstacle, value of the constant coordinate
* \param[out] outputs written in obstacle;
*/
int diag_allocate(double *obstacle, double qstartx, double qendx, double qstarty, double qendy, double step, int start_index)
{
    double stepx, stepy;
    int n_iter = (int) ceil(fabs(qstartx - qendx) / step);     // Number of iterations to perform
	int index = start_index;

	if (qstartx > qendx)    stepx = -step;
	else                    stepx = step;

	if (qstarty > qendy)    stepy = -step;
	else                    stepy = step;

    int i;

	for (i = 0; i<n_iter + 1; i = i + 1)
	{
        obstacle[index] = qstartx + (i*stepx);
        obstacle[index + 1] = qstarty + (i*stepy);
		index = index + 2;
	}
	return index - 2;
}

void potential_Field_Init(CtrlStruct *cvs)
{
    /* Allocation of the obstacles coordinates */
	cvs->state->nb_edges = 1105;
	double* edges = cvs->state->obstacle_edges;
	int start_ind = 0;
	int last_ind = 0;
	last_ind = edge_allocate(edges, -1.25, 1.25, 0.01, 1.0, 1, start_ind);
	start_ind = last_ind;
	last_ind = edge_allocate(edges, 1.0, 0.75, 0.01, 1.25, 0, start_ind);
	start_ind = last_ind;
	last_ind = edge_allocate(edges, 1.25, 1.5, 0.01, 0.75, 1, start_ind);
	start_ind = last_ind;
	last_ind = edge_allocate(edges, 0.75, -1.0, 0.01, 1.5, 0, start_ind);
	start_ind = last_ind;
	last_ind = edge_allocate(edges, 1.5, 0.7, 0.01, -1.0, 1, start_ind);
	start_ind = last_ind;
	last_ind = edge_allocate(edges, -1.0, -0.8, 0.01, 0.7, 0, start_ind);
	start_ind = last_ind;
	last_ind = edge_allocate(edges, 0.7, 0.68, 0.01, -0.8, 1, start_ind);
	start_ind = last_ind;
	last_ind = edge_allocate(edges, -0.8, -1.0, 0.01, 0.68, 0, start_ind);
	start_ind = last_ind;
	last_ind = edge_allocate(edges, 0.68, 0.27, 0.01, -1.0, 1, start_ind);
	start_ind = last_ind;
	last_ind = edge_allocate(edges, -1.0, -0.94, 0.01, 0.27, 0, start_ind);
	start_ind = last_ind;
	last_ind = edge_allocate(edges, 0.27, 0.09, 0.01, -0.94, 1, start_ind);
	start_ind = last_ind;
	last_ind = edge_allocate(edges, -0.94, -0.88, 0.01, 0.09, 0, start_ind);
	start_ind = last_ind;
	last_ind = edge_allocate(edges, 0.09, -0.09, 0.01, -0.88, 1, start_ind);
	start_ind = last_ind;
	last_ind = edge_allocate(edges, -0.88, -0.94, 0.01, -0.09, 0, start_ind);
	start_ind = last_ind;
	last_ind = edge_allocate(edges, -0.09, -0.27, 0.01, -0.94, 1, start_ind);
	start_ind = last_ind;
	last_ind = edge_allocate(edges, -0.94, -1.0, 0.01, -0.27, 0, start_ind);
	start_ind = last_ind;
	last_ind = edge_allocate(edges, -0.27, -0.68, 0.01, -1.0, 1, start_ind);
	start_ind = last_ind;
	last_ind = edge_allocate(edges, -1.0, -0.8, 0.01, -0.68, 0, start_ind);
	start_ind = last_ind;
	last_ind = edge_allocate(edges, -0.68, -0.7, 0.01, -0.8, 1, start_ind);
	start_ind = last_ind;
	last_ind = edge_allocate(edges, -0.8, -1.0, 0.01, -0.7, 0, start_ind);
	start_ind = last_ind;
	last_ind = edge_allocate(edges, -0.7, -1.5, 0.01, -1.0, 1, start_ind);
	start_ind = last_ind;
	last_ind = edge_allocate(edges, -1.0, 0.75, 0.01, -1.5, 0, start_ind);
	start_ind = last_ind;
	last_ind = edge_allocate(edges, -1.5, -1.25, 0.01, 0.75, 1, start_ind);
	start_ind = last_ind;
	last_ind = edge_allocate(edges, 0.75, 1.0, 0.01, -1.25, 0, start_ind);

    cvs->state->nb_center = 321;
    double* center = cvs->state->obstacle_center;
	start_ind = 0;
	last_ind = 0;
	last_ind = edge_allocate(center, 0.7, -0.7, 0.01, -0.35, 1, start_ind);
	start_ind = last_ind;
	last_ind = edge_allocate(center, -0.35, -0.15, 0.01, -0.7, 0, start_ind);
	start_ind = last_ind;
	last_ind = diag_allocate(center, -0.15, 0.45, -0.7, -0.1, 0.01, start_ind);
	start_ind = last_ind;
	last_ind = edge_allocate(center, -0.1, 0.1, 0.01, 0.45, 1, start_ind);
	start_ind = last_ind;
	last_ind = diag_allocate(center, 0.45, -0.15, 0.1, 0.7, 0.01, start_ind);
	start_ind = last_ind;
	last_ind = edge_allocate(center, -0.15, -0.35, 0.01, 0.7, 0, start_ind);

	/* Initialization of the parameters of the potential field path planning */
	cvs->state->k_edge = 150.0;
	cvs->state->rho_edge = 0.15;
	cvs->state->k_center = 150.0;
	cvs->state->rho_center = 0.1;
	cvs->state->rho_att = 0.3;
	cvs->state->k_att_conic = 150.0;
	cvs->state->k_att_quad = 150.0 / 0.3;
	cvs->state->K_SpeedX = 0.05;
	cvs->state->K_SpeedRot = 8.0;
}

/*! \brief Calculate the minimal distance between the robot and an obstacle
*
* \param[in] position, obstacle : position of the robot, obstacle coordinates
* \param[out] outputs written in coord, Euclidian distance;
*/
double min_Distance(double *position, double *obstacle, double *coord, int nbr_elements)
{
	double d = 0.0; //Euclidian distance
	double dist = 0.0;
	double x = position[0];
	double y = position[1];
	double x_obst, y_obst;

	int index = 0;
	int i;
	for (i = 0; i<nbr_elements; i = i + 1)
	{
		x_obst = obstacle[index];
		y_obst = obstacle[index + 1];

		if (i == 0) {
			dist = sqrt(((x - x_obst)*(x - x_obst)) + ((y - y_obst)*(y - y_obst)));
			d = dist;
			coord[0] = x_obst;
			coord[1] = y_obst;
		}
		else {
			dist = sqrt(((x - x_obst)*(x - x_obst)) + ((y - y_obst)*(y - y_obst)));
			if (dist<d) {
				d = dist;
				coord[0] = x_obst;
				coord[1] = y_obst;
			}
		}
		index = index + 2;
	}
	return d;
}

/*! \brief Calculate the angle between the robot orientation and the closest point of the obstacle
*
* \param[in] position, coord_obstacle : position of the robot, obstacle coordinates
* \param[out] angle in rads
*/
double angle_Obstacle(double *position, double *coord_obstacle)
{
    double angle = 0.0;         // Absolute orientation of the obstacle
    double rel_angle = 0.0;     // Orientation of the obstacle wrt the robot
    double delta_x = coord_obstacle[0] - position[0];
    double delta_y = coord_obstacle[1] - position[1];

    if((delta_x < 0)&&(delta_y > 0)) {              // 2nd quadrant
            angle = atan(delta_y/delta_x) + M_PI;
        }
        else if ((delta_x < 0)&&(delta_y < 0)) {    // 3rd quadrant
            angle = atan(delta_y/delta_x) - M_PI;
        }
        else {
            angle = atan(delta_y/delta_x);
        }


        rel_angle = angle - position[2];
        if (rel_angle > M_PI)          rel_angle = rel_angle - (2*M_PI);
        else if (rel_angle < -M_PI)    rel_angle = rel_angle + (2*M_PI);

        return rel_angle;
}

/*! \brief Calculate the reference speed using a potential field
*
* \param[in] position : position of the robot
* \param[out] omegaref : Reference speed of the wheels
*/
void potential_Field(CtrlStruct *cvs, double *position, double *goal_position, double *omegaref)
{
	double x = position[0];
	double y = position[1];
	double orientation = position[2];

	double x_goal = goal_position[0];
	double y_goal = goal_position[1];
	double d = sqrt((x - x_goal)*(x - x_goal) + (y - y_goal)*(y - y_goal));

	/* Compute the smallest distance and the coordinates of the closest point in the edge and the central obstacle */
	double d_edge;
	double coord_edge[2];
	double angle_edge;
	double d_center;
	double coord_center[2];
	double angle_center;

	d_edge = min_Distance(position, cvs->state->obstacle_edges, coord_edge, cvs->state->nb_edges);
	d_center = min_Distance(position, cvs->state->obstacle_center, coord_center, cvs->state->nb_center);
	angle_edge = angle_Obstacle(position, coord_edge);
	angle_center = angle_Obstacle(position, coord_center);

	/* Calculate the forces using the gradient of the potential field */
	double F_x_att, F_x_edge, F_x_center;
	double F_y_att, F_y_edge, F_y_center;
	double k_att_conic = cvs->state->k_att_conic;
	double k_att_quad = cvs->state->k_att_quad;
	double rho_att = cvs->state->rho_att;
	double k_edge = cvs->state->k_edge;
	double rho_edge = cvs->state->rho_edge;
	double k_center = cvs->state->k_center;
	double rho_center = cvs->state->rho_center;

	if (d < rho_att) {
		F_x_att = -k_att_quad * (x - x_goal);
		F_y_att = -k_att_quad * (y - y_goal);
	}
	else {
		F_x_att = -k_att_conic * (x - x_goal) / d;
		F_y_att = -k_att_conic * (y - y_goal) / d;
	}

	if (d_edge < rho_edge) {
		F_x_edge = k_edge*((1 / d_edge) - (1 / rho_edge))*((x - coord_edge[0]) / (d_edge*d_edge*d_edge));
		F_y_edge = k_edge*((1 / d_edge) - (1 / rho_edge))*((y - coord_edge[1]) / (d_edge*d_edge*d_edge));
	}
	else {
		F_x_edge = 0.0;
		F_y_edge = 0.0;
	}

	if (d_center < rho_center) {
		F_x_center = k_center*((1 / d_center) - (1 / rho_center))*((x - coord_center[0]) / (d_center*d_center*d_center));
		F_y_center = k_center*((1 / d_center) - (1 / rho_center))*((y - coord_center[1]) / (d_center*d_center*d_center));
	}
	else {
		F_x_center = 0.0;
		F_y_center = 0.0;
	}

	double Fx, Fy;      // Forces in the inertial frame
	double FxR, FyR;    // Forces in the robot frame
	double K_SpeedX = cvs->state->K_SpeedX;
	double K_SpeedRot = cvs->state->K_SpeedRot;
	double angle;
	double omega_R, omega_L;

	Fx = F_x_att + F_x_edge + F_x_center;
	Fy = F_y_att + F_y_edge + F_y_center;

	FxR = Fx*cos(position[2]) + Fy*sin(position[2]);
	FyR = -Fx*sin(position[2]) + Fy*cos(position[2]);

	angle = atan(FyR / fabs(FxR));              //Orientation in degrees

	if (FxR < 0)
	{
		if (FyR < 0) {
			omega_R = 0.02*K_SpeedX*FxR - 0.25*K_SpeedRot*(M_PI - fabs(angle));
            omega_L = 0.02*K_SpeedX*FxR + 0.25*K_SpeedRot*(M_PI - fabs(angle));
		}
		else {
			omega_R = 0.02*K_SpeedX*FxR + 0.25*K_SpeedRot*(M_PI - fabs(angle));
            omega_L = 0.02*K_SpeedX*FxR - 0.25*K_SpeedRot*(M_PI - fabs(angle));
		}
	}
	else
	{
		omega_R = K_SpeedX*FxR + K_SpeedRot*angle;
		omega_L = K_SpeedX*FxR - K_SpeedRot*angle;
    }

	if (isnan(omega_R) || (d < 0.1))
	{
		omegaref[R_ID] = 0.0;
	}
	else {
		omegaref[R_ID] = omega_R;
	}

	if (isnan(omega_L) || (d < 0.1))
	{
		omegaref[L_ID] = 0.0;
	}
	else {
		omegaref[L_ID] = omega_L;
	}
}
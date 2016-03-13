/*!
 * \file potentialfield_gr1.h
 * \brief Potential field path planning
 */
#ifndef _POTENTIALFIELD_GR1_H_
#define _POTENTIALFIELD_GR1_H_

#include "CtrlStruct_gr1.h"

int edge_allocate(double *obstacle, double qstart, double qend, double step, double qconst, int XY, int start_index);
int diag_allocate(double *obstacle, double qstartx, double qendx, double qstarty, double qendy, double step, int start_index);
double min_Distance(double *position, double *bstacle, double *coord, int nbr_elements);
double angle_Obstacle(double *position, double *coord_obstacle);
void potential_Field(CtrlStruct *cvs, double *position, double *goal_position, double *omegaref);
void potential_Field_Init(CtrlStruct *cvs);

#endif

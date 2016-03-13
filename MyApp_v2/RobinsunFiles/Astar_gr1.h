/*!
* \file Astar_gr1.h
* \brief Astar path planning
*/
#ifndef ASTAR_GR1_H_INCLUDED
#define ASTAR_GR1_H_INCLUDED

#include "CtrlStruct_gr1.h"

/*
* Drawing the map, hardcoded from the picture received of the game table
* \param[in] cvs controller main structure
*/
void Astar_get_path(CtrlStruct *cvs );

#endif // ASTAR_GR1_H_INCLUDED

/*!
* \file Astar_gr1.h
* \brief Astar path planning
*/
#ifndef ASTAR_GR1_H_INCLUDED
#define ASTAR_GR1_H_INCLUDED

#include "namespace_ctrl.h"
#include "CtrlStruct_gr1.h"

NAMESPACE_INIT(ctrlGr1);
/*
* Drawing the map, hardcoded from the picture received of the game table
* \param[in] cvs controller main structure
*/
void Astar_get_path(CtrlStruct *cvs );

NAMESPACE_CLOSE();

#endif // ASTAR_GR1_H_INCLUDED

/*!
* \file Astar_gr1.h
* \brief Astar path planning
*/
#ifndef ASTAR_GR1_H_INCLUDED
#define ASTAR_GR1_H_INCLUDED

#include "namespace_ctrl.h"
#include "CtrlStruct_gr1.h"
#include <math.h>

NAMESPACE_INIT(ctrlGr1);
/*
* Get the path
* \param[in] cvs controller main structure
*/
void Astar_get_path(CtrlStruct *cvs);


void Astar_read_path(CtrlStruct *cvs);


NAMESPACE_CLOSE();

#endif // ASTAR_GR1_H_INCLUDED

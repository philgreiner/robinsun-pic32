/*!
* \file Astar_gr1.h
* \brief Astar path planning
*/
#ifndef _ASTAR_GR1_H_
#define _ASTAR_GR1_H_

#include "namespace_ctrl.h"
#include "CtrlStruct_gr1.h"
#include <math.h>
#include <stdio.h>

NAMESPACE_INIT(ctrlGr1);
/*
* Get the path
* \param[in] cvs controller main structure
*/
void Astar_get_path(CtrlStruct *cvs);
void Astar_read_path(CtrlStruct *cvs);

NAMESPACE_CLOSE();

#endif // ASTAR_GR1_H_INCLUDED


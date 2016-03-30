/*!
* \file map_design.h
* \brief File description
*/
#ifndef _MAP_DESIGN_H_
#define _MAP_DESIGN_H_

#include "CtrlStruct_gr1.h"

NAMESPACE_INIT(ctrlGr1);
/*
* Drawing the map, hardcoded from the picture received of the game table
* \param[in] cvs controller main structure
*/
void game_map_drawing(CtrlStruct *cvs);


NAMESPACE_CLOSE();

#endif // MAP_DESIGN_H_INCLUDED

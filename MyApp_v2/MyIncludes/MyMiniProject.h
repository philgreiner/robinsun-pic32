/*******************************************************************************
* Header file for MyMiniProject                                                 *
*******************************************************************************/


#ifndef MyMINIPROJECT_H_
#define MyMINIPROJECT_H_
#include "CtrlStruct_gr1.h"
/*******************************************************************************
* Constants                                                                    *
*******************************************************************************/



/*******************************************************************************
* Functions Prototypes                                                         *
*******************************************************************************/

void    MyMiniProject_Init(void);
void    MyMiniProject_Task(void);
void    MyMiniProject_Update(CtrlStruct *cvs);
void    MyMiniProject_Send(CtrlStruct *cvs);

/*******************************************************************************
* Global Variables for MyMiniProject Functions                                 *
*******************************************************************************/

#ifdef   MyMiniProject


#endif


#endif /* MyMINIPROJET_H_ */
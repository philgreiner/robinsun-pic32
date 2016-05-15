#include "CtrlStruct_gr1.h"
#ifndef SIMU_PROJECT
    #include "MyApp.h"
#endif
#include <stdlib.h>

NAMESPACE_INIT(ctrlGr1);

/*! \brief initialize the controller structure
 *
 * \param[in] inputs of the controller
 * \param[in] outputs of the controller
 * \return controller main structure
 */
CtrlStruct* init_CtrlStruct(CtrlIn *inputs, CtrlOut *outputs)
{
	CtrlStruct *cvs;
    CtrlState *state;
    
	cvs = (CtrlStruct*) malloc(sizeof(CtrlStruct));
    state = (CtrlState*) malloc(sizeof(CtrlState));
    
    #ifdef SIMU_PROJECT
        CtrlParam *param;
        param = (CtrlParam*) malloc(sizeof(CtrlParam));
        cvs->param = param;
    #else
        cvs->param = &param; // to store in the heap memory instead of in the stack
    #endif

	cvs->inputs  = inputs;
	cvs->outputs = outputs;
	cvs->state = state;

	return cvs;
}

/*! \brief release controller main structure memory
 *
 * \param[in] cvs controller main structure
 */
void free_CtrlStruct(CtrlStruct *cvs)
{
	free(cvs);
}

NAMESPACE_CLOSE();
#include "CtrlStruct_gr1.h"
#include <stdlib.h>

NAMESPACE_INIT(ctrlGr1);

/*! \brief initialize the controller structure
 *
 * \param[in] inputs inputs of the controller
 * \param[in] outputs outputs of the controller
 * \return controller main structure
 */
CtrlStruct* init_CtrlStruct(CtrlIn *inputs, CtrlOut *outputs)
{
	CtrlStruct *cvs;
    CtrlState *state;
    CtrlParam *param;

#ifdef SIMU_PROJECT
	cvs = (CtrlStruct*) malloc(sizeof(CtrlStruct));
    state = (CtrlState*) malloc(sizeof(CtrlState));
    param = (CtrlParam*) malloc(sizeof(CtrlParam));
#endif

	cvs->inputs  = inputs;
	cvs->outputs = outputs;
	cvs->state = state;
	cvs->param = param;

	return cvs;
}

/*! \brief release controller main structure memory
 *
 * \param[in] cvs controller main structure
 */
void free_CtrlStruct(CtrlStruct *cvs)
{
#ifdef SIMU_PROJECT
	free(cvs);
#endif
}

NAMESPACE_CLOSE();
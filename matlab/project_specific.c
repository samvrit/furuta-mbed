#include <math.h>
#include <stdbool.h>
#include "project_specific.h"

#define PI (3.1415f)
#define DEG_TO_RAD(x)	((x) * (PI / 180.0f))

#define SAT(x, max, min)	( (x) > (max) ? (max) : ((x) < (min) ? (min) : (x)))


float control_output_process(const float computed_output, const float x_hat[N_STATES], const float timestep)
{
	const bool linearity = (fabsf(x_hat[1]) < DEG_TO_RAD(40.0f));
	
	return linearity ? SAT(computed_output, 4.0f, -4.0f) : 0.0f;
}
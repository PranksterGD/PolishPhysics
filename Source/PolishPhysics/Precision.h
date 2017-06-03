#pragma once
#include <math.h>
namespace PolishPhysics
{
	//Default precision used throughout the engine is float. Change this typedef to double to change the 
	//degree of precision.
	typedef float Precision;

#define precision_pow powf

#define precision_abs fabsf

#define precision_sin sinf

#define precision_cos cosf

#define precision_exp expf

#define precision_sqrt sqrtf

#define PRECISION_MAX FLT_MAX

#define UNREFERENCED_PARAMETER(P) (P)
}
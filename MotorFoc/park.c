#include "park.h"
#include <math.h>

void Park_Update(float ialpha, float ibeta, float theta, float *id, float *iq)
{
    *id =  ialpha * cosf(theta) + ibeta * sinf(theta);
    *iq = -ialpha * sinf(theta) + ibeta * cosf(theta);
}

void InvPark_Update(float vd, float vq, float theta, float *valpha, float *vbeta)
{
    *valpha = vd * cosf(theta) - vq * sinf(theta);
    *vbeta  = vd * sinf(theta) + vq * cosf(theta);
}

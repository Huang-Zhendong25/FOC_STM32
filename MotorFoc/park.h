#ifndef __PARK_H
#define __PARK_H

void Park_Update(float ialpha, float ibeta, float theta, float *id, float *iq);
void InvPark_Update(float vd, float vq, float theta, float *valpha, float *vbeta);

#endif

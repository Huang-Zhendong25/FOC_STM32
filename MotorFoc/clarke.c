#include "clarke.h"

/* Amplitude-invariant Clarke transform.
 * With Ia + Ib + Ic = 0:
 *   Ialpha = Ia
 *   Ibeta  = (Ia + 2 * Ib) / sqrt(3)
 */
void Clarke_Update(CurrentSense_t *cur)
{
    cur->Ialpha = cur->Iu;
    cur->Ibeta = (cur->Iu + 2.0f * cur->Iv) * 0.57735027f; /* 1 / sqrt(3) */
}

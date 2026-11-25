#include <math.h>
#include "ground_hold_gate.h"

void ghInit(groundHoldGate_t *g)
{
    g->inited = false;
    g->held = true;
    g->filt = 0.0f;
    g->timer = 0.0f;
}

bool ghUpdate(groundHoldGate_t *g, float height_m, float dt_s, bool armed)
{
    if (!armed || !isfinite(height_m)) {
        if (!armed) ghInit(g);
        return g->held;
    }
    if (!(dt_s > 0.0f)) dt_s = 0.0f;
    if (dt_s > 0.1f) dt_s = 0.1f;
    if (!g->inited) {
        g->inited = true;
        g->filt = height_m;
        g->timer = 0.0f;
    } else {
        g->filt += (1.0f - expf(-dt_s / GH_FILTER_TAU_S)) * (height_m - g->filt);
    }
    if (g->held) {
        if (g->filt > GH_RELEASE_M) {
            g->timer += dt_s;
            if (g->timer >= GH_RELEASE_DWELL_S) { g->held = false; g->timer = 0.0f; }
        } else {
            g->timer = 0.0f;
        }
    } else {
        if (g->filt < GH_REENGAGE_M) {
            g->timer += dt_s;
            if (g->timer >= GH_REENGAGE_DWELL_S) { g->held = true; g->timer = 0.0f; }
        } else {
            g->timer = 0.0f;
        }
    }
    return g->held;
}

float ghIntegratorCap(float i, float cap)
{
    if (!(cap > 0.0f) || !isfinite(i)) return 0.0f;
    return i > cap ? cap : (i < -cap ? -cap : i);
}

#ifndef CUSTOM_PHYSICS_ENGINE_UTILITIES_H
#define CUSTOM_PHYSICS_ENGINE_UTILITIES_H

#if defined(__APPLE__)
#define scs_force_inline inline
#else
#define scs_force_inline __forceinline // To account for the engine being ran on non-window software
#endif

namespace atg_scs {
    void freeArray(double *&data);
    void freeArray(int *&data);
} 

#endif 
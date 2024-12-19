#ifndef CUSTOM_PHYSICS_ENGINE_FORCE_GENERATOR_H
#define CUSTOM_PHYSICS_ENGINE_FORCE_GENERATOR_H

#include "system_state.h"

namespace vic_scs {
    class ForceGenerator {
        public:
        ForceGenerator();
        virtual ~ForceGenerator();
        
        virtual void apply(SystemState *system) = 0;

        int m_index;

    };
}

#endif
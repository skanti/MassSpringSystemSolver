#ifndef SPRINGS_MOVABLE_H
#define SPRINGS_MOVABLE_H

#include "VertexObject.h"
#include "GLSLProgram.h"
#include "Visualization.h"

namespace lb {

    class Movable {
    public:
        virtual void move() = 0;

    };
}

#endif //SPRINGS_MOVABLE_H

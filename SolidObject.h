//
// Created by amon on 20/02/16.
//

#ifndef SPRINGS_SOLIDOBJECT_H
#define SPRINGS_SOLIDOBJECT_H

#include "VertexObject.h"
#include "Shape.h"

namespace lb {
    class SolidObject {
        SolidObject(Shape shape, float density)
        : shape(shape), density(density) {

        }

    public:
        Shape shape;
        float density;
    };
}

#endif //SPRINGS_SOLIDOBJECT_H

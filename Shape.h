#ifndef LB_SHAPES_H
#define LB_SHAPES_H

#include <cmath>
#include "Types.h"

namespace lb {

    struct Shape {
        enum Type {
            circle = 0,
            rectangle = 1
        };

        Shape(float pos_x_, float pos_y_, sizet N_TRIANGLES_, Type SHAPE_TYPE_)
                : N_TRIANGLES(N_TRIANGLES_),
                  N_VERTICES(N_TRIANGLES_ * 3), // 3 vertices per triangle
                  SHAPE_TYPE(SHAPE_TYPE_),
                  pos_x(pos_x_),
                  pos_y(pos_y_),
                  vertices(N_VERTICES * 2) // 2 coordinates per vertex
        { }

        virtual void triangulate() = 0;

        virtual bool is_inside(float x, float y) = 0;


        const sizet N_TRIANGLES;
        const sizet N_VERTICES;
        const Type SHAPE_TYPE;
        float pos_x, pos_y;
        std::vector<float> vertices;
    };

    struct Circle : public Shape {
        Circle(float pos_x_, float pos_y_, float radius_, sizet n_triangles = 25)
                : Shape(pos_x_, pos_y_, n_triangles, Type::circle),
                  radius(radius_),
                  vertices_fan() {
            triangulate();
            triangulate_fan();
        }

        bool is_inside(float x, float y) {
            float dist2 = (x - pos_x) * (x - pos_x) + (y - pos_y) * (y - pos_y);
            return dist2 <= radius * radius;
        }

        void triangulate() {
            for (unsigned i = 0; i < N_TRIANGLES; i++) {
                float theta_0 = 2.0f * PI * (float) (i) / N_TRIANGLES;
                float theta_1 = 2.0f * PI * (float) ((i + 1) % N_TRIANGLES) / N_TRIANGLES;
                vertices[i * 6] = std::cos(theta_0);
                vertices[i * 6 + 1] = std::sin(theta_0);
                vertices[i * 6 + 2] = 0.0f;
                vertices[i * 6 + 3] = 0.0f;
                vertices[i * 6 + 4] = std::cos(theta_1);
                vertices[i * 6 + 5] = std::sin(theta_1);
            }
        }

        void triangulate_fan() {
            vertices_fan.push_back(0);
            vertices_fan.push_back(0);
            for (unsigned i = 0; i < N_TRIANGLES + 1; i++) {
                float theta = 2.0f * PI * (float) (i) / N_TRIANGLES;
                vertices_fan.push_back(std::cos(theta) * radius);
                vertices_fan.push_back(std::sin(theta) * radius);
            }
            N_VERTICES_FAN = vertices_fan.size()/2;
        }

        float radius;
        std::vector<float> vertices_fan;
        sizet N_VERTICES_FAN;

    };

    struct Rectangle : public Shape {
        Rectangle(float pos_x_, float pos_y_, float width_, float height_)
                : Shape(pos_x_, pos_y_, 2, Type::rectangle),
                  ASPECT_RATIO(width_ / height_),
                  width(width_),
                  height(height_) {
            triangulate();
        }

        bool is_inside(float x, float y) {
            return (y >= pos_y - height / 2.0f
                    && y <= pos_y + height / 2.0f
                    && x >= pos_x - width / 2.0f
                    && x <= pos_x + width / 2.0f);

        }

        void triangulate() {
            // first
            vertices[0] = -1.0f;
            vertices[1] = 1.0f;
            vertices[2] = 1.0f;
            vertices[3] = -1.0f;
            vertices[4] = 1.0f;
            vertices[5] = 1.0f;
            // second
            vertices[6] = -1.0f;
            vertices[7] = 1.0f;
            vertices[8] = -1.0f;
            vertices[9] = -1.0f;
            vertices[10] = 1.0f;
            vertices[11] = -1.0f;
        }

        const float ASPECT_RATIO;
        float width, height;
    };

}

#endif //LB_SPHERE_H

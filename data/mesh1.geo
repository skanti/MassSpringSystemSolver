// Gmsh project created on Thu Feb 25 01:48:23 2016
Point(1) = {-0.8, 0.6, 0, 1.0};
Point(2) = {0.7, 0.7, 0, 1.0};
Point(3) = {0.8, -0.6, 0, 1.0};
Point(4) = {-0.7, -0.5, 0, 1.0};
Line(1) = {1, 2};
Line(2) = {2, 3};
Line(3) = {3, 4};
Line(4) = {4, 1};
Line Loop(5) = {3, 4, 1, 2};
Plane Surface(6) = {5};

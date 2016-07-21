
#ifndef SPRINGS2D_SM_H
#define SPRINGS2D_SM_H

#include <map>
// symmetric ordered map

class SOM : public std::map<std::pair<int, int>, int> {
public:
    void stack(std::pair<int, int> i, int s) {
        std::pair<int, int> p;
        // -> making symmetric
        if (i.first <= i.second)
            p = {i.first, i.second};
        else
            p = {i.second, i.first};
        // <-

        // stacking
        this->operator[](p) = s;
    }

    int &get(std::pair<int, int> i) {
        std::pair<int, int> p;
        if (i.first <= i.second)
            p = {i.first, i.second};
        else
            p = {i.second, i.first};
        return this->operator[](p);
    }

};

#endif //SPRINGS2D_SM_H

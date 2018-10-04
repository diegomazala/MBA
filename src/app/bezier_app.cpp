#include <iostream>
#include "bezier.h"


int main(int arc, char* argv[])
{
    float coords[4] = { 1.0f, 2.0f, 3.0f, 4.0f};
    std::cout << "linear   : "<< curve::bezier::linear<float>(atof(argv[1]), coords) << std::endl;
    std::cout << "quadratic: "<< curve::bezier::quadratic<float>(atof(argv[1]), coords) << std::endl;
    std::cout << "cubic    : "<< curve::bezier::cubic<float>(atof(argv[1]), coords) << std::endl;

    float p[16] = 
    {
        0.0f, 0.33f, 0.66f, 1.0f,
        0.0f, 0.33f, 0.66f, 1.0f,
        0.0f, 0.33f, 0.66f, 1.0f,
        0.0f, 0.33f, 0.66f, 1.0f
    };

    float px[4] = {0.0f, 0.33f, 0.66f, 1.0f};
    float py[4] = {0.0f, 0.33f, 0.66f, 1.0f};

    std::cout << std::fixed << "\nBezier Surface Interpol: " << surface::bezier::cubic<float>(atof(argv[1]), atof(argv[2]), px, py) << std::endl << std::endl;

    return 0;
}
#include "accumulator.h"

#include <thrust/version.h>
#include <iostream>

int getThrustVersion()
{
    int major = THRUST_MAJOR_VERSION;
    int minor = THRUST_MINOR_VERSION;

    std::cout << "Thrust v" << major << "." << minor << std::endl;
    return 0;
}

#include "accumulator.h"

#include <thrust/copy.h>
#include <iostream>

namespace ibr
{
    /*********/
    Accumulator::Accumulator()
    {
        mhDatabase.resize(128);
    }

    /*********/
    Accumulator::~Accumulator()
    {
    }

    /*********/
    void Accumulator::accumulate(std::vector<float> factors)
    {
    }

    /*********/
    void Accumulator::setImage(std::vector<float> image, int index)
    {
        if (index >= mhDatabase.size())
            return;

        mhDatabase[index].resize(image.size());
        thrust::copy(image.begin(), image.end(), mhDatabase[index].begin());
    }

    /*********/
    void Accumulator::setSolidAngles(std::vector<float> values)
    {
        mhSolidAngles.resize(values.size());
        thrust::copy(values.begin(), values.end(), mhSolidAngles.begin());
    }
}

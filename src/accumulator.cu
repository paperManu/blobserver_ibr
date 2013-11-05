#include "accumulator.h"

#include <thrust/copy.h>
#include <iostream>

namespace ibr
{
    /*********/
    Accumulator::Accumulator()
    {
    }

    /*********/
    Accumulator::~Accumulator()
    {
    }

    /*********/
    void Accumulator::accumulate(std::list<float> factors)
    {
    }

    /*********/
    void Accumulator::setImage(std::list<float> image, int index)
    {
        if (mhDatabase.size() < index + 1)
            mhDatabase.resize(index);

        mhDatabase.resize(image.size());
        thrust::copy(image.begin(), image.end(), mhDatabase[index].begin());
    }

    /*********/
    void Accumulator::setSolidAngles(std::list<float> values)
    {
        mhSolidAngles.resize(values.size());
        thrust::copy(values.begin(), values.end(), mhSolidAngles.begin());
    }
}

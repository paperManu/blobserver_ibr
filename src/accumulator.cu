#include "accumulator.h"

#include <thrust/copy.h>
#include <iostream>

namespace ibr
{
    /*********/
    Accumulator::Accumulator()
    {
        mIsUploaded = false;
        mhDatabase.resize(128);
    }

    /*********/
    Accumulator::~Accumulator()
    {
    }

    /*********/
    void Accumulator::accumulate(std::vector<float> factors)
    {
        if (!mIsUploaded)
        {
            mdDatabase.clear();
            for (int i = 0; i < mhDatabase.size(); ++i)
            {
                thrust::device_vector<float> d = mhDatabase[i];
                mdDatabase.push_back(d);
            }
            mhDatabase.clear();
            mIsUploaded = true;
        }

        if (mdDatabase.size() == 0)
            return;

        thrust::device_vector<float> dOutput(mdDatabase[0].size());
        for (int i = 0; i < factors.size() && i < mdDatabase.size(); ++i)
            thrust::transform(mdDatabase[i].begin(), mdDatabase[i].end(), dOutput.begin(), dOutput.begin(), saxpy(factors[i]));

        mhResult.resize(dOutput.size());
        mhResult = dOutput;
    }

    /*********/
    std::vector<float> Accumulator::getResult()
    {
        std::vector<float> result(mhResult.size());
        thrust::copy(mhResult.begin(), mhResult.end(), result.begin());

        return result;
    }

    /*********/
    void Accumulator::setImage(std::vector<float> image, int index)
    {
        if (index >= mhDatabase.size())
            return;

        mhDatabase[index].resize(image.size());
        thrust::copy(image.begin(), image.end(), mhDatabase[index].begin());
        mIsUploaded = false;
    }

    /*********/
    void Accumulator::setSolidAngles(std::vector<float> values)
    {
        mhSolidAngles.resize(values.size());
        thrust::copy(values.begin(), values.end(), mhSolidAngles.begin());
        mdSolidAngles = mhSolidAngles;
    }
}

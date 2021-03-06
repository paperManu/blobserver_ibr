#include "accumulator.h"

#include <thrust/copy.h>
#include <iostream>

namespace ibr
{
    /*********/
    struct saxpy
    {
        const float a;
        saxpy(float _a) : a(_a) {}

        __host__ __device__
        float operator()(const float& x, const float& y) const
        {
            return a * x + y;
        }
    };

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
    void Accumulator::setDatabaseSize(int size)
    {
        if (size < 1)
            return;
        mhDatabase.resize(size);
    }

    /*********/
    void Accumulator::accumulate(std::vector<float> factors)
    {
        uploadImages(factors);

        if (mdDatabase.size() == 0)
            return;

        thrust::device_vector<float> dOutput(mdDatabase[0].size());
        for (int i = 0; i < factors.size() && i < mdDatabase.size(); ++i)
            thrust::transform(mdDatabase[i].begin(), mdDatabase[i].end(), dOutput.begin(), dOutput.begin(), saxpy(factors[i]));

        for (int i = mdDatabase.size(); i < factors.size(); ++i)
        {
            thrust::device_vector<float> d = mhDatabase[i];
            thrust::transform(d.begin(), d.end(), dOutput.begin(), dOutput.begin(), saxpy(factors[i]));
        }

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

    /*********/
    void Accumulator::uploadImages(std::vector<float> factors)
    {
        if (!mIsUploaded)
        {
            mdDatabase.clear();
            for (int i = 0; i < mhDatabase.size(); ++i)
            {
                size_t freeMem, totalMem;
                cudaMemGetInfo(&freeMem, &totalMem);
                if (freeMem < mhDatabase[i].capacity() * sizeof(float))
                    continue;

                thrust::device_vector<float> d = mhDatabase[i];
                //mdDatabase.push_back(d);
                mdDatabase[i] = d;
            }

            mIsUploaded = true;
            prepareOmniSource();
        }
    }

    /*********/
    void Accumulator::prepareOmniSource()
    {
        std::vector<float> factors;
        for (int i = 0; i < mhDatabase.size(); ++i)
            factors.push_back(1.f);

        thrust::device_vector<float> dOutput(mdDatabase[0].size());
        for (int i = 0; i < factors.size() && i < mdDatabase.size(); ++i)
            thrust::transform(mdDatabase[i].begin(), mdDatabase[i].end(), dOutput.begin(), dOutput.begin(), saxpy(factors[i]));

        for (int i = mdDatabase.size(); i < factors.size(); ++i)
        {
            thrust::device_vector<float> d = mhDatabase[i];
            thrust::transform(d.begin(), d.end(), dOutput.begin(), dOutput.begin(), saxpy(factors[i]));
        }

        mdOmniUnitary = dOutput;
    }
}

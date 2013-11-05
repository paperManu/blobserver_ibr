/*
 * Copyright (C) 2013 Emmanuel Durand
 *
 * This file is part of blobserver.
 *
 * This program is free software: you can redistribute it and/or modify
 *
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * blobserver is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with blobserver.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * @accumulator.h
 */

#ifndef ACCUMULATOR_H
#define ACCUMULATOR_H

#include <vector>
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>

namespace ibr
{
    class Accumulator
    {
    public:
        Accumulator();
        ~Accumulator();

        void setImage(std::vector<float> image, int index);
        void setSolidAngles(std::vector<float> values);

        void accumulate(std::vector<float> factors);

    private:
        std::vector< thrust::host_vector<float> > mhDatabase;
        thrust::host_vector<float> mhSolidAngles;
    };
}

#endif

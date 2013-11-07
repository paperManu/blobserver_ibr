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
 * @ibr.h
 * The Actuator_IBR class.
 */

#ifndef IBR_H
#define IBR_H

#include "config.h"
#include "actuator.h"
#include "accumulator.h"

 /*************/
// Class Actuator_IBR
class Actuator_IBR : public Actuator
{
    public:
        Actuator_IBR();
        Actuator_IBR(int pParam);

        static std::string getClassName() {return mClassName;}
        static std::string getDocumentation() {return mDocumentation;}

        atom::Message detect(std::vector< Capture_Ptr > pCaptures);
        void setParameter(atom::Message pMessage);

    private:
        static std::string mClassName;
        static std::string mDocumentation;

        static unsigned int mSourceNbr;
        unsigned int mFrameNumber;

        int mLatCells, mLongCells;
        std::vector<std::vector<float>> mCellsSolidAngle;

        std::string mDatabasePrefix;
        bool mDatabaseReload;
        std::vector<cv::Mat> mImageDatabase;

#if HAVE_CUDA
        std::vector<ibr::Accumulator> mAccumulators;
#endif

        void make();
        void computeSolidAngles();
        void loadDB();
        void loadFakeDB();
        void saveImage(cv::Mat img, std::string filename = "ibr.hdr");
};

REGISTER_ACTUATOR(Actuator_IBR)

#endif // IBR_H

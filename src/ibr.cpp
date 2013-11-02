#include "ibr.h"

using namespace std;

std::string Actuator_IBR::mClassName = "Actuator_Ibr";
std::string Actuator_IBR::mDocumentation = "N/A";
unsigned int Actuator_IBR::mSourceNbr = 1;

/*************/
Actuator_IBR::Actuator_IBR()
{
    make();
}

/*************/
Actuator_IBR::Actuator_IBR(int pParam)
{
    make();
}

/*************/
void Actuator_IBR::make()
{
    mOutputBuffer = cv::Mat::zeros(480, 640, CV_8UC3);

    mName = mClassName;
    // OSC path for this actuator
    mOscPath = "ibr";

    mFrameNumber = 0;
}

/*************/
atom::Message Actuator_IBR::detect(vector< Capture_Ptr > pCaptures)
{
    if (pCaptures.size() == 0)
        return mLastMessage;
    mCapture = pCaptures[0];

    mFrameNumber++;
    mLastMessage = atom::createMessage("iii", 1, 1, mFrameNumber);

    return mLastMessage;
}

/*************/
vector<Capture_Ptr> Actuator_IBR::getOutput() const
{
    vector<Capture_Ptr> outputVec;
    outputVec.push_back(mCapture);

    return outputVec;
}

/*************/
void Actuator_IBR::setParameter(atom::Message pMessage)
{
    setBaseParameter(pMessage);
}

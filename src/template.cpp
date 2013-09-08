#include "template.h"

using namespace std;

std::string Actuator_Template::mClassName = "Actuator_Template";
std::string Actuator_Template::mDocumentation = "N/A";
unsigned int Actuator_Template::mSourceNbr = 1;

/*************/
Actuator_Template::Actuator_Template()
{
    make();
}

/*************/
Actuator_Template::Actuator_Template(int pParam)
{
    make();
}

/*************/
void Actuator_Template::make()
{
    mOutputBuffer = cv::Mat::zeros(480, 640, CV_8UC3);

    mName = mClassName;
    // OSC path for this actuator
    mOscPath = "nop";

    mFrameNumber = 0;
}

/*************/
atom::Message Actuator_Template::detect(vector< Capture_Ptr > pCaptures)
{
    if (pCaptures.size() == 0)
        return mLastMessage;
    mCapture = pCaptures[0];

    mFrameNumber++;
    mLastMessage = atom::createMessage("iii", 1, 1, mFrameNumber);

    return mLastMessage;
}

/*************/
void Actuator_Template::setParameter(atom::Message pMessage)
{
    setBaseParameter(pMessage);
}

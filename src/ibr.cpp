#include "ibr.h"
#include <list>

#include <OpenImageIO/imageio.h>
OIIO_NAMESPACE_USING

using namespace std;

std::string Actuator_IBR::mClassName = "Actuator_IBR";
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

    mLatCells = 8;
    mLongCells = 16;
    computeSolidAngles();

    mDatabasePrefix = "";
    mDatabaseReload = false;
}

/*************/
atom::Message Actuator_IBR::detect(vector< Capture_Ptr > pCaptures)
{
    vector<cv::Mat> captures = captureToMat(pCaptures);
    if (captures.size() < mSourceNbr)
        return mLastMessage;

    if (mDatabaseReload)
    {
        g_log(NULL, G_LOG_LEVEL_DEBUG, "%s - Loading image database...", mClassName.c_str());
        loadDB();
    }

    if (mImageDatabase.size() < mLatCells * mLongCells)
    {
        g_log(NULL, G_LOG_LEVEL_WARNING, "%s - Not enough images in the database to accumulate.", mClassName.c_str());
        loadFakeDB();
    }

    cv::Mat input;
    captures[0].convertTo(input, CV_32F);
    float uStep = (float)input.rows / (float)mLatCells;
    float vStep = (float)input.cols / (float)mLongCells;

    // Computing the luminance for each cell
    cv::Mat accumulation = cv::Mat::zeros(mImageDatabase[0].size(), CV_MAKE_TYPE(CV_32F, mImageDatabase[0].channels()));
#if HAVE_CUDA
    vector<float> probe;
    for (int u = 0; u < mLatCells; ++u)
    {
        vector<cv::Scalar> light;
        for (int v = 0; v < mLongCells; ++v)
        {
            cv::Rect rect(v * vStep, u * uStep, vStep, uStep);
            cv::Mat roi = input(rect);
            cv::Scalar mean = cv::mean(roi);
            mean *= mCellsSolidAngle[u][v];

            //for (int c = 0; c < roi.channels(); ++c) 
            //    probe.push_back(mean[c]);
            probe.push_back(mean[0]); // For now, we only use the first channel
        }
    }

    mAccumulator.accumulate(probe);
    vector<float> result = mAccumulator.getResult();
#else
    vector<vector<cv::Scalar>> lightProbe;
    for (int u = 0; u < mLatCells; ++u)
    {
        vector<cv::Scalar> light;
        for (int v = 0; v < mLongCells; ++v)
        {
            cv::Rect rect(v * vStep, u * uStep, vStep, uStep);
            cv::Mat roi = input(rect);
            cv::Scalar mean = cv::mean(roi);
            mean *= mCellsSolidAngle[u][v];
            light.push_back(mean);
        }
        lightProbe.push_back(light);
    }

    // Accumulate the images
    for (int u = 0; u < mLatCells; ++u)
        for (int v = 0; v < mLongCells; ++v)
            cv::addWeighted(accumulation, 1.0, mImageDatabase[u * mLongCells + v], lightProbe[u][v][0], 0.0, accumulation);
#endif

    memcpy(accumulation.data, result.data(), result.size() * sizeof(float));

    mOutputBuffer = accumulation;

    mFrameNumber++;
    mLastMessage = atom::createMessage("iii", 1, 1, mFrameNumber);

    return mLastMessage;
}

/*************/
void Actuator_IBR::computeSolidAngles()
{
    mCellsSolidAngle.clear();

    float hStep = 2*M_PI / (float)mLongCells;
    float vStep = M_PI / (float)mLatCells;

    float latitude = 0.f; // This is not really latitude, as it starts at a pole
    for (int i = 0; i < mLatCells; ++i)
    {
        float longitude = 0.f;
        vector<float> solidAngles;
        for (int j = 0; j < mLongCells; ++j)
        {
            solidAngles.push_back(abs(hStep * (cos(latitude) - cos(latitude + vStep))));
            longitude += hStep;
        }

        mCellsSolidAngle.push_back(solidAngles);
        latitude += vStep;
    }
}

/*************/
void Actuator_IBR::loadDB()
{
    char extension[] = "hdr";

    vector<string> fileList;
    GError* error = NULL;
    GDir* directory;

    directory = g_dir_open((const gchar*)(string("./") + mDatabasePrefix).c_str(), 0, &error);
    if (directory == NULL)
    {
        g_log(NULL, G_LOG_LEVEL_WARNING, "%s - Unable to open directory %s", mClassName.c_str(), mDatabasePrefix.c_str());
        mDatabaseReload = false;
        return;
    }

    const gchar* filename;
    while ((filename = g_dir_read_name(directory)) != NULL)
    {
        char* substr = strstr((char*)filename, (char*)extension);
        if (substr == filename + strlen((const char*)filename) - strlen((const char*)extension))
        {
            string strName = string("./") + mDatabasePrefix + string("/") + string((const char*)filename);
            fileList.push_back(strName);
        }
    }

    std::sort(fileList.begin(), fileList.end());

    int index = 0;
    for_each (fileList.begin(), fileList.end(), [&] (string file)
    {
        ImageInput *in = ImageInput::create(file.c_str());
        if (!in)
        {
            g_log(NULL, G_LOG_LEVEL_WARNING, "%s - Unable to load image %s", mClassName.c_str(), file.c_str());
            return;
        }
        
        ImageSpec spec;
        in->open(file.c_str(), spec);
        cv::Mat image(spec.height, spec.width, CV_MAKE_TYPE(CV_32F, spec.nchannels));
        in->read_image(TypeDesc::FLOAT, image.data);
        in->close();
        delete in;

#if HAVE_CUDA
        std::vector<float> img;
        img.resize(image.total() * image.channels());
        memcpy(img.data(), image.data, image.total() * image.channels() * sizeof(float));
        mAccumulator.setImage(img, index);
        index++;
#endif
        mImageDatabase.push_back(image);

        g_log(NULL, G_LOG_LEVEL_DEBUG, "%s - Image %s loaded", mClassName.c_str(), file.c_str());

    });

    g_dir_close(directory);
    
    mDatabaseReload = false;
}

/*************/
void Actuator_IBR::loadFakeDB()
{
    mImageDatabase.clear();
    float uStep = 480.f / (float)mLatCells;
    float vStep = 640.f / (float)mLongCells;
    int index = 0;
    for (int u = 0; u < mLatCells; ++u)
    {
        for (int v = 0; v < mLongCells; ++v)
        {
            cv::Mat tmp = cv::Mat::zeros(480, 640, CV_32F);
            cv::Rect rect(v * vStep, u * uStep, vStep, uStep);
            cv::Mat roi = tmp(rect);
            roi.setTo(1.f);

#if HAVE_CUDA
            std::vector<float> img;
            img.resize(tmp.total() * tmp.channels());
            memcpy(img.data(), tmp.data, tmp.total() * tmp.channels() * sizeof(float));
            mAccumulator.setImage(img, index);
#endif
            mImageDatabase.push_back(tmp);
            index++;
        }
    }
}

/*************/
void Actuator_IBR::saveImage(cv::Mat img, string filename)
{
    if (img.total() == 0)
        return;

    ImageOutput* out = ImageOutput::create(filename.c_str());
    if (!out)
        return;
    ImageSpec spec(img.cols, img.rows, img.channels(), TypeDesc::FLOAT);
    out->open(filename.c_str(), spec);
    out->write_image(TypeDesc::FLOAT, img.data);
    out->close();
    delete out;
}

/*************/
void Actuator_IBR::setParameter(atom::Message pMessage)
{
    string cmd;
    try
    {
        cmd = toString(pMessage[0]);
    }
    catch (atom::BadTypeTagError error)
    {
        return;
    }

    if (cmd == "cellSize")
    {
        int w, h;
        if (!readParam(pMessage, w, 1))
            return;
        if (!readParam(pMessage, h, 2))
            return;
        mLatCells = w;
        mLongCells = h;
    }
    else if (cmd == "database")
    {
        string prefix;
        if (readParam(pMessage, prefix))
        {
            mDatabasePrefix = prefix;
            mDatabaseReload = true;
        }
    }
    else
        setBaseParameter(pMessage);
}

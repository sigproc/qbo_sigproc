/**********************COMMENTS***************************

READ AND WRITE CODE TAKEN FROM:
http://beansandbits.blogspot.co.uk/2012/07/readwrite-floating-point-images-with.html

************************************************************/


int writeMatToFile(const cv::Mat &I, std::string path) {
 
    //load the matrix size
    int matWidth = I.size().width, matHeight = I.size().height;
 
    //read type from Mat
    int type = I.type();
 
    //declare values to be written
    float fvalue;
    double dvalue;
    cv::Vec3f vfvalue;
    cv::Vec3d vdvalue;
 
    //create the file stream
    std::ofstream file(path.c_str(), std::ios::out | std::ios::binary );
    if (!file)
        return -1;
 
    //write type and size of the matrix first
    file.write((const char*) &type, sizeof(type));
    file.write((const char*) &matWidth, sizeof(matWidth));
    file.write((const char*) &matHeight, sizeof(matHeight));
 
    //write data depending on the image's type
    switch (type)
    {
    default:
        std::cout << "Error: wrong Mat type: must be CV_32F, CV_64F, CV_32FC3 or CV_64FC3" << std::endl;
        break;
    // FLOAT ONE CHANNEL
    case CV_32F:
        std::cout << "Writing CV_32F image" << std::endl;
        for (int i=0; i < matWidth*matHeight; ++i) {
            fvalue = I.at<float>(i);
            file.write((const char*) &fvalue, sizeof(fvalue));
        }
        break;
    // DOUBLE ONE CHANNEL
    case CV_64F:
        std::cout << "Writing CV_64F image" << std::endl;
        for (int i=0; i < matWidth*matHeight; ++i) {
            dvalue = I.at<double>(i);
            file.write((const char*) &dvalue, sizeof(dvalue));
        }
        break;
 
    // FLOAT THREE CHANNELS
    case CV_32FC3:
        std::cout << "Writing CV_32FC3 image" << std::endl;
        for (int i=0; i < matWidth*matHeight; ++i) {
            vfvalue = I.at<cv::Vec3f>(i);
            file.write((const char*) &vfvalue, sizeof(vfvalue));
        }
        break;
 
    // DOUBLE THREE CHANNELS
    case CV_64FC3:
        std::cout << "Writing CV_64FC3 image" << std::endl;
        for (int i=0; i < matWidth*matHeight; ++i) {
            vdvalue = I.at<cv::Vec3d>(i);
            file.write((const char*) &vdvalue, sizeof(vdvalue));
        }
        break;
 
    }
 
    //close file
    file.close();
 
    return 0;
}
 
int readFileToMat(cv::Mat &I, std::string path) {
 
    //declare image parameters
    int matWidth, matHeight, type;
 
    //declare values to be written
    float fvalue;
    double dvalue;
    cv::Vec3f vfvalue;
    cv::Vec3d vdvalue;
 
    //create the file stream
    std::ifstream file(path.c_str(), std::ios::in | std::ios::binary );
    if (!file)
        return -1;
 
    //read type and size of the matrix first
    file.read((char*) &type, sizeof(type));
    file.read((char*) &matWidth, sizeof(matWidth));
    file.read((char*) &matHeight, sizeof(matHeight));
 
    //change Mat type
    I = cv::Mat::zeros(matHeight, matWidth, type);
 
    //write data depending on the image's type
    switch (type)
    {
    default:
        std::cout << "Error: wrong Mat type: must be CV_32F, CV_64F, CV_32FC3 or CV_64FC3" << std::endl;
        break;
    // FLOAT ONE CHANNEL
    case CV_32F:
        std::cout << "Reading CV_32F image" << std::endl;
        for (int i=0; i < matWidth*matHeight; ++i) {
            file.read((char*) &fvalue, sizeof(fvalue));
            I.at<float>(i) = fvalue;
        }
        break;
    // DOUBLE ONE CHANNEL
    case CV_64F:
        std::cout << "Reading CV_64F image" << std::endl;
        for (int i=0; i < matWidth*matHeight; ++i) {
            file.read((char*) &dvalue, sizeof(dvalue));
            I.at<double>(i) = dvalue;
        }
        break;
 
    // FLOAT THREE CHANNELS
    case CV_32FC3:
        std::cout << "Reading CV_32FC3 image" << std::endl;
        for (int i=0; i < matWidth*matHeight; ++i) {
            file.read((char*) &vfvalue, sizeof(vfvalue));
            I.at<cv::Vec3f>(i) = vfvalue;
        }
        break;
 
    // DOUBLE THREE CHANNELS
    case CV_64FC3:
        std::cout << "Reading CV_64FC3 image" << std::endl;
        for (int i=0; i < matWidth*matHeight; ++i) {
            file.read((char*) &vdvalue, sizeof(vdvalue));
            I.at<cv::Vec3d>(i) = vdvalue;
        }
        break;
 
    }
 
    //close file
    file.close();
 
    return 0;
}

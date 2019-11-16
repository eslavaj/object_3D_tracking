
#include <numeric>
#include "matching2D.hpp"

using namespace std;

// Find best matches for keypoints in two camera images based on several matching methods
void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches, std::string descriptorType, std::string matcherType, std::string selectorType)
{
    // configure matcher
    bool crossCheck = false;
    cv::Ptr<cv::DescriptorMatcher> matcher;

    int normType;
    if(descriptorType.compare("DES_HOG")==0)
    {
    	normType = cv::NORM_L2;
    }
    else
    {
    	/*DES_BINARY*/
    	normType = cv::NORM_HAMMING;
    }


    if (matcherType.compare("MAT_BF") == 0)
    {
        matcher = cv::BFMatcher::create(normType, crossCheck);
    }
    else if (matcherType.compare("MAT_FLANN") == 0)
    {

/* Hack, replaced by flann matcher using LSH parameters
        if (descSource.type() != CV_32F)
        { // OpenCV bug workaround : convert binary descriptors to floating point due to a bug in current OpenCV implementation
        	descSource.convertTo(descSource, CV_32F);
        	descRef.convertTo(descRef, CV_32F);
        }
        matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
*/

        if (descriptorType.compare("DES_HOG") == 0)
        {
        	matcher = cv::FlannBasedMatcher::create();
        }
        else
        {
        	/*DES_BINARY*/
        	const cv::Ptr<cv::flann::IndexParams>& indexParams = cv::makePtr<cv::flann::LshIndexParams>(12, 20, 2);
        	matcher = cv::makePtr<cv::FlannBasedMatcher>(indexParams);
        }

    }

    // perform matching task
    if (selectorType.compare("SEL_NN") == 0)
    { // nearest neighbor (best match)

    	double t = (double)cv::getTickCount();
    	matcher->match(descSource, descRef, matches); // Finds the best match for each descriptor in desc1
    	t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    	cout << "(NN) with n=" << matches.size() << " matches in " << 1000 * t / 1.0 << " ms" << endl;
    }
    else if (selectorType.compare("SEL_KNN") == 0)
    { // k nearest neighbors (k=2)

    	vector<vector<cv::DMatch>> knnMatches;
    	double t = (double)cv::getTickCount();
    	matcher->knnMatch(descSource, descRef, knnMatches, 2);

    	double minDescDistRatio = 0.8;
    	for(auto it = knnMatches.begin(); it!=knnMatches.end(); it++)
    	{
    		if((*it)[0].distance < minDescDistRatio*( (*it)[1].distance ) )
    		{
    			matches.push_back((*it)[0]);
    		}
    	}
    	cout << "KNN - keypoints removed: " << knnMatches.size() - matches.size() << " points" << endl;
    	t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    	cout << "******* (KNN) with n=" << matches.size() << " matches in " << 1000 * t / 1.0 << " ms" << endl;

    }
}


int descriTypeHelper(std::string descriptorType)
{
	if(descriptorType.compare("BRISK")==0)
		return 1;
	if(descriptorType.compare("BRIEF")==0)
		return 2;
	if(descriptorType.compare("ORB")==0)
		return 3;
	if(descriptorType.compare("FREAK")==0)
		return 4;
	if(descriptorType.compare("AKAZE")==0)
		return 5;
	if(descriptorType.compare("SIFT")==0)
		return 6;
	/*Use BRISK by default*/
	return 1;

}


// Use one of several types of state-of-art descriptors to uniquely identify keypoints
void descKeypoints(vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, string descriptorType)
{
    // select appropriate descriptor
    cv::Ptr<cv::DescriptorExtractor> extractor;
    int descriTypeSelect = descriTypeHelper(descriptorType);
    int threshold = 30;        // FAST/AGAST detection threshold score.
    int octaves = 3;           // detection octaves (use 0 to do single scale)
    float patternScale = 1.0f; // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.

    switch (descriTypeSelect)
    {
    case 1:
    	/*BRISK*/
    	extractor = cv::BRISK::create(threshold, octaves, patternScale);
    	break;
    case 2:
    	/*BRIEF*/
    	extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
		break;
	case 3:
		/*ORB*/
		extractor = cv::ORB::create();
		break;
	case 4:
		/*FREAK*/
		extractor = cv::xfeatures2d::FREAK::create();
		break;
	case 5:
		/*AKAZE*/
		extractor = cv::AKAZE::create(cv::AKAZE::DESCRIPTOR_KAZE, 0, 1);
		break;
	case 6:
		/*SIFT*/
		extractor = cv::xfeatures2d::SIFT::create();
		break;
	default:
		break;

	}


    // perform feature description
    double t = (double)cv::getTickCount();
    extractor->compute(img, keypoints, descriptors);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << descriptorType << " descriptor extraction in " << 1000 * t / 1.0 << " ms" << endl;
}

// Detect keypoints in image using the traditional Shi-Thomasi detector
void detKeypointsShiTomasi(vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    // compute detector parameters based on image size
    int blockSize = 4;       //  size of an average block for computing a derivative covariation matrix over each pixel neighborhood
    double maxOverlap = 0.0; // max. permissible overlap between two features in %
    double minDistance = (1.0 - maxOverlap) * blockSize;
    int maxCorners = img.rows * img.cols / max(1.0, minDistance); // max. num. of keypoints

    double qualityLevel = 0.01; // minimal accepted quality of image corners
    double k = 0.04;

    // Apply corner detection
    double t = (double)cv::getTickCount();
    vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(img, corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, false, k);

    // add corners to result vector
    for (auto it = corners.begin(); it != corners.end(); ++it)
    {

        cv::KeyPoint newKeyPoint;
        newKeyPoint.pt = cv::Point2f((*it).x, (*it).y);
        newKeyPoint.size = blockSize;
        keypoints.push_back(newKeyPoint);
    }
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "Shi-Tomasi detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "Shi-Tomasi Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}



void detKeypointsHarris(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{

	// Detector parameters
	int blockSize = 2;     // for every pixel, a blockSize × blockSize neighborhood is considered
	int apertureSize = 3;  // aperture parameter for Sobel operator (must be odd)
	int minResponse = 100; // minimum value for a corner in the 8bit scaled response matrix
	double k = 0.04;       // Harris parameter (see equation for details)


	cv::Mat dst, dst_norm, dst_norm_scaled;
	dst = cv::Mat::zeros(img.size(), CV_32FC1);
	double t = (double)cv::getTickCount();
	cv::cornerHarris(img, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT);
	cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
	cv::convertScaleAbs(dst_norm, dst_norm_scaled);

	/*visualize results before non max suppression */
	/*
	string windowName = "Harris Corner Detector Response Matrix";
	cv::namedWindow(windowName, 4);
	cv::imshow(windowName, dst_norm_scaled);
	cv::waitKey(0); */


	/*Non maximum suppression implementation*/
	double overlapThreshold = 20.0;
	//vector<cv::KeyPoint> keyPoints;

	for(int j=0; j<dst_norm.rows; j++)
	{
		for(int i=0; i<dst_norm.cols; i++)
		{
			int response = (int)(dst_norm.at<float>(j, i));
			if( response > minResponse)
			{
				cv::KeyPoint newKeyPoint;
				newKeyPoint.pt = cv::Point2f(i, j);
				newKeyPoint.size = 2*apertureSize;
				newKeyPoint.response = response;

				/*Check overlap*/
				bool overlaping = false;
				for(auto it = keypoints.begin(); it!=keypoints.end(); it++)
				{
					double overlBetweenPts = cv::KeyPoint::overlap(newKeyPoint, *it);

					if(overlBetweenPts>=overlapThreshold)
					{
						overlaping = true;
						if(newKeyPoint.response > it->response)
						{
							*it = newKeyPoint;
							break;
						}
					}
				}
				if(!overlaping)
				{
					keypoints.push_back(newKeyPoint);
				}
			}
		}
	}

	t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
	cout << "Harris detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

	/*visualize results after NMS*/
	if (bVis)
	{
		string windowName1 = "Harris Corner Detector results";
		cv::namedWindow(windowName1, 5);
		cv::Mat visImage = dst_norm_scaled.clone();
		cv::drawKeypoints(dst_norm_scaled, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
		cv::imshow(windowName1, visImage);
		cv::waitKey(0);
	}

}

int modernDetectTypeHelper(std::string detectorType)
{
	if(detectorType.compare("FAST")==0)
		return 1;
	if(detectorType.compare("BRISK")==0)
		return 2;
	if(detectorType.compare("ORB")==0)
		return 3;
	if(detectorType.compare("AKAZE")==0)
		return 4;
	if(detectorType.compare("SIFT")==0)
		return 5;
	/*Use BRISK by default*/
	return 2;

}

void detKeypointsModern(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, std::string detectorType, bool bVis)
{

	int detTypeCase = modernDetectTypeHelper(detectorType);
	int threshold;
	cv::Ptr<cv::FeatureDetector> detector;

	double t = (double)cv::getTickCount();
	switch(detTypeCase)
	{
	case 1:
		/*FAST*/
		threshold = 30;
		cv::FAST(img, keypoints, threshold, true, cv::FastFeatureDetector::TYPE_9_16);
		break;
	case 2:
		/*BRISK*/
		detector = cv::BRISK::create();
		detector->detect(img, keypoints);
		break;
	case 3:
		/*ORB*/
		detector = cv::ORB::create(500, 1.2, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, 20);
		detector->detect(img, keypoints);
		break;
	case 4:
		/*AKAZE*/
		detector = cv::AKAZE::create();
		detector->detect(img, keypoints);
		break;
	case 5:
		/*SIFT*/
		detector = cv::xfeatures2d::SIFT::create();
		detector->detect(img, keypoints);
		break;
	default:
		break;

	}

	t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
	cout << "Keypoints detection with n= " << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

}



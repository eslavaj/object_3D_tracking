
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"
#include <boost/random.hpp>
#include <eigen3/Eigen/Dense>

using namespace std;


// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2); // pixel coordinates
        pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}


void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, currColor);  
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    //cv::namedWindow(windowName, 1);
    cv::namedWindow(windowName, cv::WINDOW_NORMAL);
    cv::imshow(windowName, topviewImg);

    if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}


// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{


	/*Calculate mean*/
	cv::Point2f mean_prev(0, 0);
	cv::Point2f mean_curr(0, 0);
	double mean_dist_curr_prev = 0;
	std::vector<cv::DMatch> kptMatchesCand;



	float cropFactor = 0;
	cv::Rect croppedROI;
	croppedROI.x = boundingBox.roi.x + cropFactor * boundingBox.roi.width / 2.0;
	croppedROI.y = boundingBox.roi.y + cropFactor * boundingBox.roi.height / 2.0;
	croppedROI.width = boundingBox.roi.width * (1 - cropFactor);
	croppedROI.height = boundingBox.roi.height * (1 - cropFactor);


	/*select keypoint matches inside ROI*/
	for(auto kptmatch : kptMatches)
	{
		if(kptsCurr[kptmatch.trainIdx].pt.inside(croppedROI))
		{
			kptMatchesCand.push_back(kptmatch);
		}
	}

	/*Calculate mean*/
	int kptMatchNbr = kptMatchesCand.size();
	for(auto kptmatch : kptMatchesCand)
	{
		/*
		mean_prev.x = mean_prev.x + kptsPrev[kptmatch.queryIdx].pt.x;
		mean_prev.y = mean_prev.y + kptsPrev[kptmatch.queryIdx].pt.y;
		mean_curr.x = mean_curr.x + kptsCurr[kptmatch.trainIdx].pt.x;
		mean_curr.y = mean_curr.y + kptsCurr[kptmatch.trainIdx].pt.y;
		*/
		mean_curr = mean_curr + kptsCurr[kptmatch.trainIdx].pt;
		mean_prev = mean_prev + kptsPrev[kptmatch.queryIdx].pt;
		mean_dist_curr_prev = mean_dist_curr_prev + cv::norm(kptsCurr[kptmatch.trainIdx].pt - kptsPrev[kptmatch.queryIdx].pt);
	}

	cout<<"_-_-_-_-_-_-_-_-_-_-_-_  "<< kptMatchesCand.size() << endl;
	/*
	mean_prev.x = mean_prev.x/kptMatchNbr;
	mean_prev.y = mean_prev.y/kptMatchNbr;
	mean_curr.x = mean_curr.x/kptMatchNbr;
	mean_curr.y = mean_curr.y/kptMatchNbr;
	*/
	mean_curr = mean_curr/kptMatchNbr;
	mean_prev = mean_prev/kptMatchNbr;
	mean_dist_curr_prev = mean_dist_curr_prev/kptMatchNbr;
	cout<<"######## "<< mean_curr.x <<" " << mean_curr.y << endl;
	cout<<"######## "<< mean_prev.x <<" " << mean_prev.y << endl;

	double dist_to_mean_th = 200;
	double dist_to_mean_th_2 = pow(dist_to_mean_th, 2);
	double std_dev_prev = 0;
	double std_dev_curr = 0;

	/*calculate standard deviation*/
	for(auto kptmatch : kptMatchesCand)
	{
		std_dev_curr = std_dev_curr + pow(cv::norm(mean_curr - kptsCurr[kptmatch.trainIdx].pt), 2);
		std_dev_prev = std_dev_prev + pow(cv::norm(mean_prev - kptsPrev[kptmatch.queryIdx].pt), 2);
	}
	std_dev_curr = sqrt(std_dev_curr/(kptMatchNbr-1));
	std_dev_prev = sqrt(std_dev_prev/(kptMatchNbr-1));
	cout<<"std_dev_curr  "<< std_dev_curr <<" " << endl;
	cout<<"std_dev_prev  "<< std_dev_prev <<" " << endl;

	/*Take only kptMatch near to the mean*/
	for(auto kptmatch : kptMatchesCand)
	{
		double dist_prev = cv::norm(kptsPrev[kptmatch.queryIdx].pt- mean_prev);
		double dist_curr = cv::norm(kptsCurr[kptmatch.trainIdx].pt- mean_curr);
		double dist_curr_prev = cv::norm(kptsCurr[kptmatch.trainIdx].pt- kptsPrev[kptmatch.queryIdx].pt);
		if( (dist_curr<=std_dev_curr) /*&& (dist_prev <=std_dev_prev) && (dist_curr_prev<=mean_dist_curr_prev*1.5) && (dist_curr_prev>=mean_dist_curr_prev*0.5)*/)
		{
			boundingBox.kptMatches.push_back(kptmatch);
			boundingBox.keypoints.push_back(kptsCurr[kptmatch.trainIdx]);
			//cout<<"*-*-*-*-*-*-*- "<< kptsCurr[kptmatch.trainIdx].pt.x <<" " << kptsCurr[kptmatch.trainIdx].pt.y<<endl;
		}
	}
	cout<<"------> boundingBox.keypoints size: "<<boundingBox.keypoints.size()<<endl;
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
	// compute distance ratios between all matched keypoints
	vector<double> distRatios;
	for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)
	{ // outer kpt. loop

		// get current keypoint and its matched partner in the prev. frame
		cv::KeyPoint kpOuterCurr = kptsCurr.at(it1->trainIdx);
		cv::KeyPoint kpOuterPrev = kptsPrev.at(it1->queryIdx);

		for (auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end(); ++it2)
		{ // inner kpt.-loop

			double minDist = 100; // min. required distance

			// get next keypoint and its matched partner in the prev. frame
			cv::KeyPoint kpInnerCurr = kptsCurr.at(it2->trainIdx);
			cv::KeyPoint kpInnerPrev = kptsPrev.at(it2->queryIdx);

			// compute distances and distance ratios
			double distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
			double distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);

			if (distPrev > std::numeric_limits<double>::epsilon() && (distCurr >= minDist) )
			{ // avoid division by zero

				double distRatio = distCurr / distPrev;
				distRatios.push_back(distRatio);

			}
		} // eof inner loop over all matched kpts
	}     // eof outer loop over all matched kpts

	// only continue if list of distance ratios is not empty
	if (distRatios.size() == 0)
	{
		TTC = NAN;
		return;
	}

	/*Taking the median of distances*/
	std::sort(distRatios.begin(), distRatios.end());

	for(auto distrat: distRatios)
	{
		cout<<"distrat "<<distrat<<endl;
	}

	long medIndex = floor(distRatios.size() / 2.0);
	double medDistRatio;
	/*
	do
	{
		medDistRatio = distRatios.size() % 2 == 0 ? (distRatios[medIndex - 1] + distRatios[medIndex]) / 2.0 : distRatios[medIndex]; // compute median dist. ratio to remove outlier influence
		medIndex++;
	}
	while(medDistRatio==1);
*/

	medDistRatio = distRatios.size() % 2 == 0 ? (distRatios[medIndex - 1] + distRatios[medIndex]) / 2.0 : distRatios[medIndex]; // compute median dist. ratio to remove outlier influence

	cout<<"======"<<medDistRatio<<endl;

	TTC = -(1 / frameRate) / (1 - medDistRatio);

}


bool compareLidarPointX (LidarPoint i,LidarPoint j) { return (i.x<j.x); }


void computeTTCLidar_median(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{

	sort(lidarPointsPrev.begin(), lidarPointsPrev.end(), compareLidarPointX);
	sort(lidarPointsCurr.begin(), lidarPointsCurr.end(), compareLidarPointX);

    // find
    double medianXPrev = 0, medianXCurr = 0;

	if(lidarPointsPrev.size()%2 == 0)
	{
		int idx = (int)(lidarPointsPrev.size()/2);
		medianXPrev = (lidarPointsPrev[idx].x + lidarPointsPrev[idx+1].x)/2;
	}
	else
	{
		int idx = (int)(lidarPointsPrev.size()/2) + 1;
		medianXPrev = lidarPointsPrev[idx].x;
	}

	if(lidarPointsCurr.size()%2 == 0)
	{
		int idx = (int)(lidarPointsCurr.size()/2);
		medianXCurr = (lidarPointsCurr[idx].x + lidarPointsCurr[idx+1].x)/2;
	}
	else
	{
		int idx = (int)(lidarPointsCurr.size()/2) + 1;
		medianXCurr = lidarPointsCurr[idx].x;
	}

    // compute TTC from both measurements
    TTC = medianXCurr * (1/frameRate) / fabs(medianXPrev - medianXCurr);


}



void calcDist_to_main_vert_plane(std::vector<LidarPoint> &lidarPoints, double &distance)
{
	boost::random::mt19937 rng;
	boost::random::uniform_int_distribution<> idx_gen(0,lidarPoints.size()-1);

	int pindex1, pindex2, pindex3;
	Eigen::Vector3f v1, v12, v13, normal_vect;

	double distanceThreshold_n;
	float d;
	int nbr_inliers_most = 0;
	Eigen::Vector3f most_normal_vect;
	Eigen::Vector3f most_v1;

	int cloud_size = lidarPoints.size();

	int maxIterations = 10;
	double distanceThreshold = 0.1;


	for(int iter=0; iter<maxIterations; iter++)
	{
		/* get 3 index randomly */
		pindex1 = idx_gen(rng);
		pindex2 = idx_gen(rng);
		pindex3 = idx_gen(rng);

		/*It is almost improbable but just in case let's check points are different*/
		if( (pindex1==pindex2) || (pindex2==pindex3) || (pindex1==pindex3))
		{
			continue;
		}

		/*Vector from origin to p1*/
		Eigen::Vector3f v1(lidarPoints[pindex1].x, lidarPoints[pindex1].y, lidarPoints[pindex1].z);
		Eigen::Vector3f v2(lidarPoints[pindex2].x, lidarPoints[pindex2].y, lidarPoints[pindex2].z);
		Eigen::Vector3f v3(lidarPoints[pindex3].x, lidarPoints[pindex3].y, lidarPoints[pindex3].z);
		/*Vector from p1 to p2*/
		v12 = v2 - v1;
		/*Vector from p1 to p3*/
		v13 = v3 - v1;
		/*Normal vector of plane*/
		/*
					normal_vect[0] = v12[1]*v13[2]- v12[2]*v13[1];
					normal_vect[1] = v12[2]*v13[0]- v12[0]*v13[2];
					normal_vect[2] = v12[0]*v13[1]- v12[1]*v13[0];
		 */
		normal_vect = v12.cross(v13);
		distanceThreshold_n = distanceThreshold*normal_vect.norm();
		int nbr_inliers=0;
		/*Measure distance between every point and fitted line*/
		for(int index=0; index< cloud_size; index++)
		{
			Eigen::Vector3f vp(lidarPoints[index].x, lidarPoints[index].y, lidarPoints[index].z);
			d = fabs(normal_vect.dot(vp-v1));

			/*If distance is smaller than threshold count it as inlier*/
			if (d<=distanceThreshold_n)
			{
				nbr_inliers++;
			}
		}

		if (nbr_inliers>nbr_inliers_most)
		{
			nbr_inliers_most = nbr_inliers;
			most_normal_vect = normal_vect;
			most_v1 = v1;
		}
	}

	/*Now search the minimal horizontal distance in the inliers using the best parameters*/
	distanceThreshold_n = distanceThreshold*most_normal_vect.norm();

	double min_horizont_dist = 10000;

	for(int index=0; index< cloud_size; index++)
	{
		/*p = cloud->points[index];
		  vp = p.getVector3fMap();*/
		Eigen::Vector3f vp(lidarPoints[index].x, lidarPoints[index].y, lidarPoints[index].z);
		d = fabs(most_normal_vect.dot(vp-most_v1));

		/*If distance is smaller than threshold count it as inlier*/
		if (d<=distanceThreshold_n)
		{
			/*Get the minimal horizontal distance of the inliers*/
			if(lidarPoints[index].x < min_horizont_dist)
			{
				min_horizont_dist = lidarPoints[index].x;
			}
		}
	}

	distance = min_horizont_dist;

}




void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{

	double minDistCurr, minDistPrev;

	calcDist_to_main_vert_plane(lidarPointsPrev, minDistPrev);
	calcDist_to_main_vert_plane(lidarPointsCurr, minDistCurr);


	// compute TTC from both measurements
	TTC = minDistCurr * (1/frameRate) / fabs(minDistPrev - minDistCurr);

}



void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    /*Get the candidate bounding boxes matches*/
	std::map<std::pair<int, int>, int> bbMatchesCand_withOccur;
	int nbr_of_pairs =1;

	for(auto it_kptm = matches.begin(); it_kptm!=matches.end(); it_kptm++)
	{
		cv::KeyPoint kpt_curr = currFrame.keypoints[it_kptm->trainIdx];
		for(auto it_bb_curr = currFrame.boundingBoxes.begin(); it_bb_curr!= currFrame.boundingBoxes.end(); it_bb_curr++)
		{
			/*See if this keypoint match is contained on ROI of this bounding box*/
			if(it_bb_curr->roi.contains(kpt_curr.pt) == true)
			{
				/*Add this keypoint Match to this bounding box*/
				//it_bb_curr->kptMatches.push_back(*it_kptm);

				/*Iterate through previous frame bounding boxes*/
				for(auto it_bb_prev = prevFrame.boundingBoxes.begin(); it_bb_prev!= prevFrame.boundingBoxes.end(); it_bb_prev++)
				{
					cv::KeyPoint kpt_prev = prevFrame.keypoints[it_kptm->queryIdx];
					if(it_bb_prev->roi.contains(kpt_prev.pt) == true)
					{
						if( bbMatchesCand_withOccur.find(std::pair<int,int>(it_bb_curr->boxID, it_bb_prev->boxID)) == bbMatchesCand_withOccur.end() )
						{
							bbMatchesCand_withOccur.insert( std::pair<std::pair<int, int>, int>(std::pair<int,int>(it_bb_curr->boxID, it_bb_prev->boxID), 1 ) );
						}
						else
						{
							bbMatchesCand_withOccur.at(std::pair<int,int>(it_bb_curr->boxID, it_bb_prev->boxID)) = \
									bbMatchesCand_withOccur.at(std::pair<int,int>(it_bb_curr->boxID, it_bb_prev->boxID))+ 1;
						}
					}
				}
			}
		}
	}


	/*Get the bounding box with the most number of keypoint matches*/
	std::map<int, int>::iterator it_tmp;
	int curr_pivot_bb = bbMatchesCand_withOccur.begin()->first.first;
	int kpt_match_bb_score = bbMatchesCand_withOccur.begin()->second;
	int kpt_match_in_bb_thold = 0;

	for(auto it_bbMwOcc = bbMatchesCand_withOccur.begin(); it_bbMwOcc!=bbMatchesCand_withOccur.end(); it_bbMwOcc++)
	{
		if(it_bbMwOcc->second >= kpt_match_in_bb_thold)
		{
			if(it_bbMwOcc->first.first == curr_pivot_bb)
			{
				if(it_bbMwOcc->second > kpt_match_bb_score)
				{
					bbBestMatches.at(it_bbMwOcc->first.first) = it_bbMwOcc->first.second;
					kpt_match_bb_score = it_bbMwOcc->second;
				}
			}
			else
			{
				kpt_match_bb_score = it_bbMwOcc->second;
				curr_pivot_bb = it_bbMwOcc->first.first;
				bbBestMatches.insert( it_bbMwOcc->first );
			}
		}
	}

/*
	for(auto i : bbBestMatches)
	{
		cout<<i.first<<" "<<i.second<<endl;
	}
*/

}




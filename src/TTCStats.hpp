/*
 * printReport.hpp
 *
 *  Created on: Nov 17, 2019
 *      Author: jeslava
 */

#ifndef SRC_TTCSTATS_HPP_
#define SRC_TTCSTATS_HPP_

#include <string>

class TTCStats {

private:
  std::string detectorType;
  std::string descriptorType;
  std::string matchingType;
  std::string selectorType;
  int framenbr;
  int   numKeyPointsPerframe;
  int   numKeyPointsPerROI;
  int   numMatchedKeyPoints;
  double detectorTime;
  double descriptorTime;
  double MatcherTime;
  double ttc_camera;
  double ttc_lidar;

public:
  TTCStats(std::string _detectorType, std::string _descriptorType, std::string _matchingType, std::string _selectorType, int _framenbr,\
		  int _numKeyPointsPerframe, int _numKeyPointsPerROI, int _numMatchedKeyPoints, double _detectorTime, double _descriptorTime, double _MatcherTime, \
		  double _ttc_camera, double _ttc_lidar): \
		  detectorType(_detectorType), descriptorType(_detectorType), matchingType(_matchingType), selectorType(_selectorType), framenbr(_framenbr),\
		  numKeyPointsPerframe(_numKeyPointsPerframe), numKeyPointsPerROI(_numKeyPointsPerROI), numMatchedKeyPoints(_numMatchedKeyPoints), \
		  detectorTime(_detectorTime), descriptorTime(_descriptorTime), MatcherTime(_MatcherTime), ttc_camera(_ttc_camera), ttc_lidar(_ttc_lidar) {}

  TTCStats(std::string _detectorType, std::string _descriptorType, std::string _matchingType, std::string _selectorType) : \
		  detectorType(_detectorType), descriptorType(_descriptorType), matchingType(_matchingType), selectorType(_selectorType){}

  std::string getStats()
  {
	  std::string res;

	  res.append(detectorType);
	  res.append(",");
	  res.append(descriptorType);
	  res.append(",");
	  /*res.append(matchingType);
	  res.append(",");
	  res.append(selectorType);
	  res.append(",");*/
	  res.append(std::to_string(framenbr));
	  res.append(",");

	  res.append(std::to_string(numKeyPointsPerframe));
	  res.append(",");
	  res.append(std::to_string(numKeyPointsPerROI));
	  res.append(",");
	  res.append(std::to_string(numMatchedKeyPoints));
	  res.append(",");

	  res.append(std::to_string(detectorTime));
	  res.append(",");
	  res.append(std::to_string(descriptorTime));
	  res.append(",");
	  res.append(std::to_string(MatcherTime));
	  res.append(",");
	  res.append(std::to_string(ttc_camera));
	  res.append(",");
	  res.append(std::to_string(ttc_lidar));


	  return res;

  }


  std::string getStatsHeader()
  {
	  std::string res(\
			  "Detector Type, Descriptor Type, Frame#, #KeyPointsPerFrame, #KeyPointsPerROI, #MatchedPoints, DetectorTime(ms), DescriptorTime(ms), MatchingTime(ms), " \
			  "ttc_camera, ttc_lidar");

	  return res;

  }


	double getDescriptorTime() const {
		return descriptorTime;
	}

	void setDescriptorTime(double descriptorTime) {
		this->descriptorTime = descriptorTime;
	}

	const std::string& getDescriptorType() const {
		return descriptorType;
	}

	void setDescriptorType(const std::string& descriptorType) {
		this->descriptorType = descriptorType;
	}

	double getDetectorTime() const {
		return detectorTime;
	}

	void setDetectorTime(double detectorTime) {
		this->detectorTime = detectorTime;
	}

	const std::string& getDetectorType() const {
		return detectorType;
	}

	void setDetectorType(const std::string& detectorType) {
		this->detectorType = detectorType;
	}

	double getMatcherTime() const {
		return MatcherTime;
	}

	void setMatcherTime(double matcherTime) {
		MatcherTime = matcherTime;
	}

	const std::string& getMatchingType() const {
		return matchingType;
	}

	void setMatchingType(const std::string& matchingType) {
		this->matchingType = matchingType;
	}

	int getNumKeyPointsPerframe() const {
		return numKeyPointsPerframe;
	}

	void setNumKeyPointsPerframe(int numKeyPointsPerframe) {
		this->numKeyPointsPerframe = numKeyPointsPerframe;
	}

	int getNumKeyPointsPerRoi() const {
		return numKeyPointsPerROI;
	}

	void setNumKeyPointsPerRoi(int numKeyPointsPerRoi) {
		numKeyPointsPerROI = numKeyPointsPerRoi;
	}

	int getNumMatchedKeyPoints() const {
		return numMatchedKeyPoints;
	}

	void setNumMatchedKeyPoints(int numMatchedKeyPoints) {
		this->numMatchedKeyPoints = numMatchedKeyPoints;
	}

	const std::string& getSelectorType() const {
		return selectorType;
	}

	void setSelectorType(const std::string& selectorType) {
		this->selectorType = selectorType;
	}

	int getFramenbr() const {
		return framenbr;
	}

	void setFramenbr(int framenbr) {
		this->framenbr = framenbr;
	}

	double getTtcCamera() const {
		return ttc_camera;
	}

	void setTtcCamera(double ttcCamera) {
		ttc_camera = ttcCamera;
	}

	double getTtcLidar() const {
		return ttc_lidar;
	}

	void setTtcLidar(double ttcLidar) {
		ttc_lidar = ttcLidar;
	}
};






#endif /* SRC_TTCSTATS_HPP_ */

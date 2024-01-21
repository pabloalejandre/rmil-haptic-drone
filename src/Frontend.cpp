/*
 * Frontend.cpp
 *
 *  Created on: 9 Dec 2020
 *      Author: sleutene
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <memory>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <ros/ros.h>

#include <brisk/brisk.h>

#include <arp/Frontend.hpp>

#ifndef CV_AA
#define CV_AA cv::LINE_AA // maintains backward compatibility with older OpenCV
#endif

#ifndef CV_BGR2GRAY
#define CV_BGR2GRAY cv::COLOR_BGR2GRAY // maintains backward compatibility with older OpenCV
#endif

// Colors in BGR order.
static const cv::Scalar unmatched_color(255, 0, 0);
static const cv::Scalar outlier_color(0, 0, 255);
static const cv::Scalar inlier_color(0, 255, 0);
// Marker width in pixels.
static constexpr int marker_size = 10;

static void draw_legend(cv::Mat& image) {
  cv::Mat legend(image.rows / 5, image.cols, image.type(), cv::Scalar(255, 255, 255));
  int x = marker_size;
  int y = marker_size;
  const int step = 2.5 * marker_size;
  cv::drawMarker(legend, cv::Point(x, y), unmatched_color, cv::MARKER_CROSS, marker_size, 1, cv::LINE_4);
  cv::drawMarker(legend, cv::Point(x, y + step), outlier_color, cv::MARKER_SQUARE, marker_size, 1, cv::LINE_4);
  cv::drawMarker(legend, cv::Point(x, y + 2 * step), inlier_color, cv::MARKER_SQUARE, marker_size, 1, cv::LINE_4);
  x *= 2;
  y += marker_size / 2;
  cv::putText(legend, "Unmatched keypoints", cv::Point(x, y), cv::FONT_HERSHEY_PLAIN, 1, 1, 1, CV_AA);
  cv::putText(legend, "RANSAC outliers", cv::Point(x, y + step), cv::FONT_HERSHEY_PLAIN, 1, 1, 1, CV_AA);
  cv::putText(legend, "RANSAC inliers", cv::Point(x, y + 2 * step), cv::FONT_HERSHEY_PLAIN, 1, 1, 1, CV_AA);
  cv::vconcat(image, legend, image);
}

namespace arp {

Frontend::Frontend(int imageWidth, int imageHeight, double focalLengthU, double focalLengthV,
                   double imageCenterU, double imageCenterV, double k1, double k2, double p1, double p2,
                   double focalLengthMap, uint32_t briskMatchingThreshold, double briskUniformityRadius,
                   size_t briskOctaves, double briskAbsoluteThreshold, size_t briskMaxNumKeypoints, float reprojectionDistThreshold) :
  camera_(imageWidth, imageHeight, focalLengthU, focalLengthV, imageCenterU,
          imageCenterV,
          arp::cameras::RadialTangentialDistortion(k1, k2, p1, p2)),
  trackedPoseId_(0), isTrackedPoseIdSet_(false), briskMatchingThreshold_(briskMatchingThreshold), reprojectionDistThreshold_(reprojectionDistThreshold)
{
  camera_.initialiseUndistortMaps();

  // also save for OpenCV RANSAC later
  cameraMatrix_ = cv::Mat::zeros(3, 3, CV_64FC1);
  cameraMatrix_.at<double>(0,0) = focalLengthU;
  cameraMatrix_.at<double>(1,1) = focalLengthV;
  cameraMatrix_.at<double>(0,2) = imageCenterU;
  cameraMatrix_.at<double>(1,2) = imageCenterV;
  cameraMatrix_.at<double>(2,2) = 1.0;
  distCoeffs_ = cv::Mat::zeros(1, 4, CV_64FC1);
  distCoeffs_.at<double>(0) = k1;
  distCoeffs_.at<double>(1) = k2;
  distCoeffs_.at<double>(2) = p1;
  distCoeffs_.at<double>(3) = p2;
  
  // BRISK detector and descriptor
  detector_.reset(new brisk::ScaleSpaceFeatureDetector<brisk::HarrisScoreCalculator>(
      briskUniformityRadius, briskOctaves, briskAbsoluteThreshold, briskMaxNumKeypoints));
  extractor_.reset(new brisk::BriskDescriptorExtractor(true, false));
  
  // leverage camera-aware BRISK (caution: needs the *_new* maps...)
  cv::Mat rays = cv::Mat(imageHeight, imageWidth, CV_32FC3);
  cv::Mat imageJacobians = cv::Mat(imageHeight, imageWidth, CV_32FC(6));
  for (int v=0; v<imageHeight; ++v) {
    for (int u=0; u<imageWidth; ++u) {
      Eigen::Vector3d ray;
      Eigen::Matrix<double, 2, 3> jacobian;
      if(camera_.backProject(Eigen::Vector2d(u,v), &ray)) {
        ray.normalize();
      } else {
        ray.setZero();
      }
      rays.at<cv::Vec3f>(v,u) = cv::Vec3f(ray[0],ray[1],ray[2]);
      Eigen::Vector2d pt;
      if(camera_.project(ray, &pt, &jacobian)
         ==cameras::ProjectionStatus::Successful) {
        cv::Vec6f j;
        j[0]=jacobian(0,0);
        j[1]=jacobian(0,1);
        j[2]=jacobian(0,2);
        j[3]=jacobian(1,0);
        j[4]=jacobian(1,1);
        j[5]=jacobian(1,2);
        imageJacobians.at<cv::Vec6f>(v,u) = j;
      }
    }
  }
  std::static_pointer_cast<cv::BriskDescriptorExtractor>(extractor_)->setCameraProperties(rays, imageJacobians, focalLengthMap);
}

bool  Frontend::loadMap(std::string path) {
  std::ifstream mapfile(path);
  if(!mapfile.good()) {
    return false;
  }
  
  // read each line
  std::string line;
  std::set<uint64_t> lmIds;
  uint64_t poseId = 0;
  LandmarkVec landmarks;
  while (std::getline(mapfile, line)) {

    // Convert to stringstream
    std::stringstream ss(line);
    
    if(0==line.compare(0, 7,"frame: ")) {
      // store previous set into map
      landmarks_[poseId] = landmarks;
      // get pose id:
      std::stringstream frameSs(line.substr(7,line.size()-1));
      frameSs >> poseId;
      if(!frameSs.eof()) {
        std::string covisStr;
        frameSs >> covisStr; // comma
        frameSs >> covisStr;
        int ctr=0;
        if(0==covisStr.compare("covisibilities:")) {
          while(!frameSs.eof()) {
            uint64_t covisId;
            frameSs >> covisId;
            if(ctr<10) { // only 10 best ones to keep things tractable.
              covisibilities_[poseId].insert(covisId);
            }
            ctr++;
          }
        }
      }
      // move to filling next set of landmarks
      landmarks.clear();
    } else {
      if(poseId>0) {
        Landmark landmark;

        // get keypoint idx
        size_t keypointIdx;
        std::string keypointIdxString;
        std::getline(ss, keypointIdxString, ',');
        std::stringstream(keypointIdxString) >> keypointIdx;
        
        // get landmark id
        uint64_t landmarkId;
        std::string landmarkIdString;
        std::getline(ss, landmarkIdString, ',');
        std::stringstream(landmarkIdString) >> landmarkId;
        landmark.landmarkId = landmarkId;

        // read 3d position
        for(int i=0; i<3; ++i) {
          std::string coordString;
          std::getline(ss, coordString, ',');
          double coord;
          std::stringstream(coordString) >> coord;
          landmark.point[i] = coord;
        }

        // Get descriptor
        std::string descriptorstring;
        std::getline(ss, descriptorstring);
        landmark.descriptor = cv::Mat(1,48,CV_8UC1);
        for(int col=0; col<48; ++col) {
          uint32_t byte;
          std::stringstream(descriptorstring.substr(2*col,2)) >> std::hex >> byte;
          landmark.descriptor.at<uchar>(0,col) = byte;
        }
        lmIds.insert(landmarkId);
        landmarks.push_back(landmark);
      }      
    } 
  }
  if(poseId>0) {
    // store into map
    landmarks_[poseId] = landmarks;
  }
  ROS_INFO_STREAM("loaded " << lmIds.size() << " landmarks from " << landmarks_.size() << " poses.");
  return lmIds.size() > 0;
}

bool Frontend::loadDBoW2Voc(std::string path) {
  ROS_INFO_STREAM("Loading DBoW2 vocabulary from " << path);
  dBowVocabulary_.load(path);
  // Hand over vocabulary to dataset. false = do not use direct index:
  dBowDatabase_.setVocabulary(dBowVocabulary_, false, 0);
  ROS_INFO_STREAM("loaded DBoW2 vocabulary with " << dBowVocabulary_.size() << " words.");
  return true;
}

bool Frontend::fillDBoW2Database() {
  if(dBowVocabulary_.empty()) {
    ROS_ERROR_STREAM("DBoW2 vocabulary not loaded.");
    return false;
  }
  if(landmarks_.empty()) {
    ROS_ERROR_STREAM("No landmarks loaded.");
    return false;
  }

  dBowIdToPoseId_.clear();
  uint64_t dbowId = 0;
  ROS_INFO_STREAM("filling DBoW2 database with " << landmarks_.size() << " poses.");
  for(const auto& lms : landmarks_) {
    std::vector<std::vector<unsigned char>> features;
    for(const auto& lm : lms.second) {
      for(int i = 0; i < lm.descriptor.rows; i++) {  // just one row
        std::vector<unsigned char> ft;
        for(int j = 0; j < lm.descriptor.cols; j++) {
          ft.push_back(lm.descriptor.at<uchar>(i, j));
        }
        features.push_back(ft);
      }
    }
    dBowDatabase_.add(features);
    dBowIdToPoseId_.insert(std::make_pair(dbowId, lms.first));
    dbowId++;
  }
  ROS_INFO_STREAM("added " << dBowDatabase_.size() << " keyframes to DBoW2 database.");
  return true;
}

int Frontend::detectAndDescribe(
    const cv::Mat& grayscaleImage, const Eigen::Vector3d& extractionDirection,
    std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors) const {

  // run BRISK detector
  detector_->detect(grayscaleImage, keypoints);

  // run BRISK descriptor extractor
  // orient the keypoints according to the extraction direction:
  Eigen::Vector3d ep;
  Eigen::Vector2d reprojection;
  Eigen::Matrix<double, 2, 3> Jacobian;
  Eigen::Vector2d eg_projected;
  for (size_t k = 0; k < keypoints.size(); ++k) {
    cv::KeyPoint& ckp = keypoints[k];
    const Eigen::Vector2d kp(ckp.pt.x, ckp.pt.y);
    // project ray
    camera_.backProject(kp, &ep);
    // obtain image Jacobian
    camera_.project(ep+extractionDirection.normalized()*0.001, &reprojection);
    // multiply with gravity direction
    eg_projected = reprojection-kp;
    double angle = atan2(eg_projected[1], eg_projected[0]);
    // set
    ckp.angle = angle / M_PI * 180.0;
  }
  extractor_->compute(grayscaleImage, keypoints, descriptors);

  return keypoints.size();
}

bool Frontend::ransac(const std::vector<cv::Point3d>& worldPoints, 
                      const std::vector<cv::Point2d>& imagePoints, 
                      kinematics::Transformation & T_CW,
                      std::vector<int>& inliers,
                      bool needsReInitialization) const {
  inliers.clear();
  if(worldPoints.size() != imagePoints.size()) {
    return false;
  }
  if(worldPoints.size() < 4) {
    return false;  // not enough points for PnP
  }
  if(worldPoints.size() < 5 && !needsReInitialization) {
    return false; // not realiabl enough
  }
  double confidence = needsReInitialization ? 0.8 : 0.99;

  cv::Mat rvec, tvec;
  bool success = cv::solvePnPRansac(
        worldPoints, imagePoints, cameraMatrix_, distCoeffs_,
        rvec, tvec, false, 100, 5.0, confidence, inliers, cv::SOLVEPNP_EPNP);
  if(!success) {
    inliers.clear(); // just in case OpenCV populates inliers even when ransac fails
  }

  // set pose
  cv::Mat R = cv::Mat::zeros(3, 3, CV_64FC1);
  cv::Rodrigues(rvec, R);
  Eigen::Matrix4d T_CW_mat = Eigen::Matrix4d::Identity();
  for(int i=0; i<3; i++) {
    T_CW_mat(i,3) = tvec.at<double>(i);
    for(int j=0; j<3; j++) {
      T_CW_mat(i,j) = R.at<double>(i,j);
    }
  }
  T_CW = kinematics::Transformation(T_CW_mat);

  const float inlierRatio = needsReInitialization ? 0.5 : 0.8;  
  return success && (double(inliers.size())/double(imagePoints.size()) > inlierRatio);
}

bool Frontend::detectAndMatch(const cv::Mat& image, const Eigen::Vector3d & extractionDirection, 
                              DetectionVec & detections, kinematics::Transformation & T_CW, 
                              cv::Mat & visualisationImage, bool needsReInitialisation)
{
  detections.clear(); // make sure empty

  // to gray:
  cv::Mat grayScale;
  cv::cvtColor(image, grayScale, CV_BGR2GRAY);

  // run BRISK detector and descriptor extractor:
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;
  detectAndDescribe(grayScale, extractionDirection, keypoints, descriptors);

  // match with DBoW2 for re-initialization
  std::set<uint64_t> keyFrameIds;
  if(needsReInitialisation || !isTrackedPoseIdSet_) {
    ROS_INFO_STREAM("matching with DBoW2 for re-initialization.");
    DBoW2::QueryResults results;
    std::vector<std::vector<unsigned char>> features(descriptors.rows);
    for (int i = 0; i < descriptors.rows; ++i) {
      std::vector<unsigned char> ft;
      for (int j = 0; j < descriptors.cols; ++j) {
        ft.push_back(descriptors.at<uchar>(i, j));
      }
      features[i] = ft;
    }

    dBowDatabase_.query(features, results, 20);

    // get the best match, aka the highest scored keyframe
    if(results.empty()) {
      ROS_WARN_STREAM("no DBow2 match found.");
      return false;
    }

    // Select the keyframes to take based on the DBow2 scoring. The results are sorted by score in descending order.
    for(size_t i = 0; i < results.size(); i++) {
      if(results[i].Score < 0.2) break;
      keyFrameIds.insert(dBowIdToPoseId_.at(results[i].Id));
    }
    if(keyFrameIds.empty()) {
      ROS_WARN_STREAM("not enough DBow2 matches found.");
      return false;
    }
    ROS_INFO_STREAM("... found " << keyFrameIds.size() << " keyframes with DBoW2.");

  } else {
    // Otherwise, just add the tracked keyframe and its co-visible keyframes for matching.
    keyFrameIds.insert(trackedPoseId_);
    const auto& covisibleIds = covisibilities_.at(trackedPoseId_);
    keyFrameIds.insert(covisibleIds.begin(), covisibleIds.end());
  }

  // match keypoints to landmarks in all keyframes
  std::vector<cv::Point3d> worldPoints;
  std::vector<cv::Point2d> imagePoints;
  std::vector<uint64_t> landmarkIds;
  std::vector<bool> keypointMatched(keypoints.size(), false);
  std::map<uint64_t, uint64_t> mostMatched;
  for (size_t k = 0; k < keypoints.size(); ++k) {
    const Eigen::Vector2d kp(keypoints[k].pt.x, keypoints[k].pt.y);
    uchar *keypointDescriptor = descriptors.data + k * 48; // descriptors are 48 bytes long
    const Landmark* closestLm = nullptr;
    uint32_t closestDist = UINT32_MAX;
    uint64_t closestLmKId = UINT64_MAX;

    for (const auto kId : keyFrameIds) {
      for (const auto& lm : landmarks_.at(kId)) {
        // the pose prior T_CW is valid if needsReInitialisation is false
        if (!needsReInitialisation) {
          Eigen::Vector2d imagePoint;
          const Eigen::Vector3d point_C = (T_CW * lm.point.homogeneous()).head<3>();
          const auto status = camera_.project(point_C, &imagePoint);
          // rule out potential matches if the landmarks are not projecting into the live frame or
          // are projecting too far from the keypoint
          if (status != arp::cameras::ProjectionStatus::Successful || (imagePoint - kp).norm() > reprojectionDistThreshold_) {
            continue;
          }
        }

        // compute desc. distance: 3 for 3x128bit (=48 bytes)
        const uint32_t dist = brisk::Hamming::PopcntofXORed(keypointDescriptor, lm.descriptor.data, 3);
        if (dist > briskMatchingThreshold_ || dist > closestDist) {
          continue;
        }

        closestDist = dist;
        closestLm = &lm;
        closestLmKId = kId;
      }
    }

    if (closestLm) {
      if (mostMatched.count(closestLmKId) == 0) {
        mostMatched[closestLmKId] = 0;
      }
      mostMatched[closestLmKId] += 1;
      imagePoints.emplace_back(keypoints[k].pt);
      worldPoints.emplace_back(closestLm->point.x(), closestLm->point.y(), closestLm->point.z());
      landmarkIds.emplace_back(closestLm->landmarkId);
      keypointMatched[k] = true;
    }
  }

  // track against the keyframe with the most matched landmarks
  size_t numMatchesBest = 0;
  uint64_t prevTrackedPoseId = trackedPoseId_;
  for (const auto& it: mostMatched) {
    if (it.second > numMatchesBest) {
      numMatchesBest = it.second;
      if (!isTrackedPoseIdSet_ || trackedPoseId_ != it.first) {
        trackedPoseId_ = it.first;
        isTrackedPoseIdSet_ = true;
      }
    }
  }

  if(numMatchesBest < 6) {
    ROS_INFO_STREAM("too few matches: " << numMatchesBest);
    isTrackedPoseIdSet_ = false; // trigger DBoW next time.
  }

  // run RANSAC (to remove outliers and get pose T_CW estimate)
  std::vector<int> inliers;
  bool ransacSucceeded = ransac(worldPoints, imagePoints, T_CW, inliers, needsReInitialisation);
  if(!ransacSucceeded) {
    if(!needsReInitialisation) {
      // consider all as inliers at this point so at least we can do something
      for (size_t i = 0; i < worldPoints.size(); i++) {
        inliers.push_back(i);
      }
      ROS_INFO_STREAM("ransac failed, accepting all as matches ");
    } else {
      ROS_WARN_STREAM("ransac failed, cannot reinitialise ");
    }
  }

  // set up detections from RANSAC inliers
  std::vector<bool> is_inlier(imagePoints.size(), false);
  for(int i : inliers) {
    is_inlier[i] = true;
    Detection detection;
    detection.keypoint = Eigen::Vector2d(imagePoints[i].x, imagePoints[i].y);
    detection.landmark = Eigen::Vector3d(worldPoints[i].x, worldPoints[i].y, worldPoints[i].z);
    detection.landmarkId = landmarkIds[i];
    detections.push_back(detection);
  }

  // visualize unmatched keypoints
  for (size_t k = 0; k < keypoints.size(); k++) {
    if (!keypointMatched[k]) {
      cv::drawMarker(visualisationImage, keypoints[k].pt, unmatched_color, cv::MARKER_CROSS, marker_size - 3, 1, cv::LINE_4);
    }
  }

  // visualize RANSAC inliers and outliers
  for(size_t i = 0; i < imagePoints.size(); i ++) {
    const cv::Scalar color = is_inlier[i] ? inlier_color : outlier_color;
    cv::drawMarker(visualisationImage, imagePoints[i], color, cv::MARKER_SQUARE, marker_size, 1, cv::LINE_4);
  }

  draw_legend(visualisationImage);
  return ransacSucceeded;
}

}  // namespace arp


/*
 * Frontend.hpp
 *
 *  Created on: 8 Dec 2020
 *      Author: sleutene
 */

#ifndef ARDRONE_PRACTICALS_INCLUDE_ARP_FRONTEND_HPP_
#define ARDRONE_PRACTICALS_INCLUDE_ARP_FRONTEND_HPP_

#include <memory>
#include <vector>
#include <map>
#include <Eigen/Core>
#include <opencv2/features2d/features2d.hpp>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#include <DBoW2/DBoW2.h>
#pragma GCC diagnostic pop
#include <DBoW2/FBrisk.hpp>

#include <arp/cameras/PinholeCamera.hpp>
#include <arp/cameras/RadialTangentialDistortion.hpp>
#include <arp/cameras/NoDistortion.hpp>
#include <arp/kinematics/Transformation.hpp>

namespace arp {

///\brief This class processes an image and returns the detected marker poses.
class Frontend
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// \brief Sets the underlying camera parameters (RadialTangentialDistortion)
  /// \parameter imageWidth The image width.
  /// \parameter imageHeight The image height.
  /// \parameter focalLengthU The focal length in u direction.
  /// \parameter focalLengthV The focal length in v direction.
  /// \parameter imageCenterU The image centre in u direction.
  /// \parameter imageCenterV The image centre in v direction.
  /// \parameter k1 1st radial distortion parameter.
  /// \parameter k2 2nd radial distortion parameter. 
  /// \parameter p1 1st tangential distortion parameter.
  /// \parameter p2 2nd tangential distortion parameter.
  /// \parameter focalLengthMap The focal length used to create the map.
  /// \parameter briskMatchingThreshold The Hamming distance threshold for BRISK feature descriptors to be matched.
  /// \parameter briskUniformityRadius The BRISK feature detector uniformity radius.
  /// \parameter briskOctaves The BRISK feature detector number of octaves.
  /// \parameter briskAbsoluteThreshold The BRISK feature detector absolute threshold.
  /// \parameter briskMaxNumKeypoints The BRISK feature detector maximum number of keypoints.
  Frontend(int imageWidth, int imageHeight, double focalLengthU, double focalLengthV,
           double imageCenterU, double imageCenterV, double k1, double k2, double p1, double p2,
           double focalLengthMap, uint32_t briskMatchingThreshold, double briskUniformityRadius,
           size_t briskOctaves, double briskAbsoluteThreshold, size_t briskMaxNumKeypoints, float reprojectionDistThreshold);

  /// \brief Load the map
  /// \parameter path The full path to the map file.
  /// \return True on success.
  bool loadMap(std::string path);
  
  /// \brief Load DBoW2 vocabulary
  /// \parameter path The full path to the vocabulary file.
  /// \return True on success.
  bool loadDBoW2Voc(std::string path);

  /// \brief Fill BoW database with map features.
  /// \return True on success.
  bool fillDBoW2Database();

  /// \brief Detect and match keypoints in image that can be fed to an estimator.
  /// \warning If not returning true, there may still be detections, but not verified
  ///          and T_CW will be invalid.
  /// \parameter image The input image.
  /// \parameter extractionDirection The extraction direction in camera coordinates.
  ///                                Use (0,1,0) if in vision-only case.
  /// \parameter detections The obtained detections.
  /// \parameter visualisationImage An image to visualise stuff into. Can be the input image.
  /// \parameter needsReInitialisation If true, T_CW *cannot* be used for efficient matching.
  /// \return True on success of RANSAC, i.e. T_CW will be valid.
  bool detectAndMatch(const cv::Mat& image, const Eigen::Vector3d & extractionDirection,
                      DetectionVec & detections, kinematics::Transformation & T_CW, 
                      cv::Mat & visualisationImage, bool needsReInitialisation);

  /// \brief Camera model accessor.
  /// \return Const reference to the underlying camera projection model.
  const arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion>& camera() const {
    return camera_;
  }

 protected:
  /// \brief Detects BRISK (HARRIS) keypoints and extracts descriptors along the extraction direction.
  int detectAndDescribe(
    const cv::Mat& grayscaleImage, const Eigen::Vector3d& extractionDirection,
    std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors) const;

  /// \brief Wraps the OpenCV 3D2D pose RANSAC.
  bool ransac(const std::vector<cv::Point3d>& worldPoints, 
              const std::vector<cv::Point2d>& imagePoints, 
              kinematics::Transformation & T_CW, std::vector<int>& inliers, bool needsReInitialisation) const;

  arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion> camera_; ///< Camera model
  cv::Mat cameraMatrix_; ///< Copy camera model for OpenCV RANSAC
  cv::Mat distCoeffs_; ///< Copy camera model for OpenCV RANSAC

  /// \brief A simple class to store a landmark with position and descriptors.
  struct Landmark {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    cv::Mat descriptor; ///< The descriptor.
    Eigen::Vector3d point; ///< The 3d point in World coordinates.
    uint64_t landmarkId;
  };
  typedef std::vector<Landmark, Eigen::aligned_allocator<Landmark>> LandmarkVec;
  std::map<uint64_t, LandmarkVec> landmarks_; ///< Landmarks grouped by pose ID.
  std::map<uint64_t, std::set<uint64_t>> covisibilities_; ///< Set of covisibilities by pose ID.

  std::shared_ptr<cv::FeatureDetector> detector_;  ///< the BRISK detector
  std::shared_ptr<cv::DescriptorExtractor> extractor_;  ///< the BRISK extractor

  /// \brief DBoW for loop closure: BRISK Vocabulary.
  typedef DBoW2::TemplatedVocabulary<DBoW2::FBrisk::TDescriptor, DBoW2::FBrisk> FBriskVocabulary;
  /// \brief DBoW for loop closure: BRISK Database.
  typedef DBoW2::TemplatedDatabase<DBoW2::FBrisk::TDescriptor, DBoW2::FBrisk> FBriskDatabase;

  FBriskVocabulary dBowVocabulary_; ///< The BRISK DBoW vocabulary -- load from disk.
  FBriskDatabase dBowDatabase_;   ///< The DBoW database to add frames to.
  std::map<uint64_t , uint64_t> dBowIdToPoseId_; ///< Map from DBoW2 ID to pose ID.
  uint64_t trackedPoseId_; ///< The pose ID of the last tracked frame.
  bool isTrackedPoseIdSet_; ///< True if the tracked pose ID is set.
  uint32_t briskMatchingThreshold_;
  float reprojectionDistThreshold_; ///< Threshold to consider correct match between keypoint and projected landmark

 private:
  Frontend() = delete;
};

}  // namespace arp

#endif /* ARDRONE_PRACTICALS_INCLUDE_ARP_FRONTEND_HPP_ */

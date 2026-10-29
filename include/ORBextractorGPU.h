/**
 * ORBextractorGPU -- orbslam3_ros2 patch (10), 2026-10-28.
 *
 * GPU implementation of the expensive part of ORBextractor: FAST corner
 * scoring + non-max suppression, the 7x7 Gaussian blur, intensity-centroid
 * orientation and the rotated rBRIEF descriptor. The octree keypoint
 * distribution (DistributeOctTree) stays on the CPU, so the SELECTION of
 * keypoints keeps ORB-SLAM3's uniform-coverage behaviour; only the per-pixel
 * and per-keypoint arithmetic moves.
 *
 * No CUDA toolkit is needed to BUILD this: the driver API (libcuda.so.1, part
 * of the NVIDIA driver) and NVRTC (libnvrtc.so.12, shipped e.g. with the torch
 * pip wheels) are loaded with dlopen at first use and the kernels are compiled
 * at runtime from the source string in ORBextractorGPU.cc. If any step fails
 * the extractor falls back to the CPU path and says so once.
 *
 * Thread model: ORB-SLAM3 runs the left and right extractors in two threads
 * (Frame.cc). One GpuOrb per ORBextractor instance, each with its own stream
 * and buffers; the compiled module is process-wide and created under a mutex.
 *
 * Library search order for libnvrtc.so.12 (libcuda.so.1 is found by the loader):
 *   $ORB_SLAM3_NVRTC_DIR, then the usual loader paths, then
 *   $HOME/.local/lib/python3.*\/site-packages/nvidia/cuda_nvrtc/lib (pip wheel),
 *   then /usr/local/cuda/lib64.
 */
#ifndef ORBEXTRACTORGPU_H
#define ORBEXTRACTORGPU_H

#include <opencv2/core/core.hpp>
#include <string>
#include <vector>

namespace ORB_SLAM3
{

class GpuOrb
{
public:
    GpuOrb();
    ~GpuOrb();

    /// True once the driver, NVRTC and the compiled module are available in
    /// this process. Cheap after the first call. Never throws.
    static bool Available();
    /// Why Available() is false (empty when it is true).
    static const std::string& Reason();

    /// Per-instance setup: level count, largest bordered level size, FAST
    /// thresholds, and the two tables the CPU extractor uses (umax, pattern).
    bool Init(int nlevels, int maxWholeW, int maxWholeH, int maxStep,
              int iniThFAST, int minThFAST, const std::vector<int>& umax,
              const std::vector<cv::Point>& pattern);

    /// Upload one pyramid level (the Mat is a ROI into its bordered parent,
    /// exactly as ORBextractor::ComputePyramid builds it). Must be called for
    /// every level before Detect/Describe of that level in the same frame.
    bool Upload(int level, const cv::Mat& levelImage);

    /// FAST + NMS + ORB-SLAM3's two-threshold cell rule on the uploaded level.
    /// Output keypoints are relative to (minBorderX, minBorderY) with the FAST
    /// score in `response`, i.e. what ComputeKeyPointsOctTree feeds into
    /// DistributeOctTree.
    bool Detect(int level, int minBorderX, int minBorderY, int maxBorderX, int maxBorderY,
                int wCell, int hCell, int nCols, int nRows, std::vector<cv::KeyPoint>& out);

    /// Intensity-centroid orientation (IC_Angle) for keypoints in level
    /// coordinates; fills kp.angle in degrees like computeOrientation().
    bool Orientation(int level, std::vector<cv::KeyPoint>& kps);

    /// 7x7 Gaussian blur (sigma 2, BORDER_REFLECT_101 within the level) and
    /// rotated rBRIEF for keypoints in level coordinates; `desc` must be
    /// kps.size() x 32 CV_8U.
    bool Describe(int level, const std::vector<cv::KeyPoint>& kps, cv::Mat& desc);

    struct Impl;          // public so file-local helpers in the .cc can take an Impl&
private:
    Impl* p_;
    GpuOrb(const GpuOrb&) = delete;
    GpuOrb& operator=(const GpuOrb&) = delete;
};

/**
 * GpuHamming -- orbslam3_ros2 patch (10b), 2026-10-29.
 *
 * Brute-force Hamming 2-nearest-neighbour matching on the GPU, the drop-in
 * equivalent of cv::BFMatcher(cv::NORM_HAMMING).knnMatch(query, train, m, 2)
 * that Frame::ComputeStereoFishEyeMatches runs on every stereo frame
 * (~2000 x 2000 256-bit descriptors with a full-width lapping area). Same
 * tie order as OpenCV (the lower train index wins), so the ratio test that
 * follows sees identical input. Shares the driver/NVRTC module of GpuOrb;
 * process-wide buffers behind a mutex (Frames are built on one thread).
 */
class GpuHamming
{
public:
    /// Fills `matches` (one entry per query row, two DMatch each: best then
    /// second best) exactly like the CPU knnMatch with k=2. Returns false and
    /// leaves `matches` untouched when the GPU path is unavailable, the
    /// descriptors are not CV_8U x 32, fewer than two train rows exist, or a
    /// side exceeds the 20000-row buffers -- the caller then uses the CPU matcher.
    static bool Knn2(const cv::Mat& query, const cv::Mat& train,
                     std::vector<std::vector<cv::DMatch> >& matches);
};

} // namespace ORB_SLAM3

#endif

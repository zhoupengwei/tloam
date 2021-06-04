/**
 * @Copyright 2020, Wuhan University of Technology
 * @Author: Pengwei Zhou
 * @Date: 2020/08/19 下午9:24
 * @FileName: PointCloud2.hpp
 * @Description: Increase the intensity information and other functions of the point cloud
 * @License: See LICENSE for the license information
 */

#include <Eigen/Dense>
#include <algorithm>
#include <numeric>
#include <random>

#include <open3d/geometry/BoundingVolume.h>
#include <open3d/geometry/KDTreeFlann.h>
#include <open3d/geometry/Qhull.h>
#include <open3d/geometry/TriangleMesh.h>
#include <open3d/utility/Console.h>
#include <open3d/utility/Eigen.h>

#include "tloam/open3d/PointCloud2.hpp"

namespace open3d{
namespace geometry{

PointCloud2 & PointCloud2::Clear() {
    points_.clear();
    normals_.clear();
    colors_.clear();
    intensity_.clear();

    return *this;
}

bool PointCloud2::IsEmpty() const {
    return !HasPoints();
}

Eigen::Vector3d PointCloud2::GetMinBound() const {
    return ComputeMinBound(points_);
}

Eigen::Vector3d PointCloud2::GetMaxBound() const {
    return ComputeMaxBound(points_);
}

Eigen::Vector3d PointCloud2::GetCenter() const {
    return ComputeCenter(points_);
}

AxisAlignedBoundingBox PointCloud2::GetAxisAlignedBoundingBox() const {
    return AxisAlignedBoundingBox::CreateFromPoints(points_);
}

OrientedBoundingBox PointCloud2::GetOrientedBoundingBox() const {
    return OrientedBoundingBox::CreateFromPoints(points_);
}

PointCloud2 & PointCloud2::Transform(const Eigen::Matrix4d &transformation) {
    TransformPoints(transformation, points_);
    TransformNormals(transformation, normals_);
    return *this;
}

PointCloud2 & PointCloud2::Translate(const Eigen::Vector3d &translation, bool relative) {
    TranslatePoints(translation, points_, relative);
    return *this;
}

PointCloud2 & PointCloud2::Scale(const double scale, const Eigen::Vector3d &center) {
    ScalePoints(scale, points_, center);
    return *this;
}

PointCloud2 & PointCloud2::Rotate(const Eigen::Matrix3d &R, const Eigen::Vector3d &center) {
    RotatePoints(R, points_, center);
    RotateNormals(R, normals_);
    return *this;
}

PointCloud2& PointCloud2::operator+=(const PointCloud2 &cloud) {
    // We do not use std::vector::insert to combine std::vector because it will
    // crash if the pointcloud is added to itself.
    if (cloud.IsEmpty()) return (*this);
    size_t old_vert_num = points_.size();
    size_t add_vert_num = cloud.points_.size();
    size_t new_vert_num = old_vert_num + add_vert_num;
    if ((!HasPoints() || HasNormals()) && cloud.HasNormals()) {
        normals_.resize(new_vert_num);
        for (size_t i = 0; i < add_vert_num; i++)
            normals_[old_vert_num + i] = cloud.normals_[i];
    } else {
        normals_.clear();
    }
    if ((!HasPoints() || HasColors()) && cloud.HasColors()) {
        colors_.resize(new_vert_num);
        for (size_t i = 0; i < add_vert_num; i++)
            colors_[old_vert_num + i] = cloud.colors_[i];
    } else {
        colors_.clear();
    }
    if ((!HasPoints() || HasIntensity()) && cloud.HasIntensity()) {
        intensity_.resize(new_vert_num);
        for (size_t i = 0; i < add_vert_num; i++)
            intensity_[old_vert_num + i] = cloud.intensity_[i];
    } else {
        intensity_.clear();
    }

    points_.resize(new_vert_num);
    for (size_t i = 0; i < add_vert_num; i++){
        points_[old_vert_num + i] = cloud.points_[i];
    }

    return (*this);
}

PointCloud2 & PointCloud2::operator+(const PointCloud2 &cloud) const {
    return (PointCloud2(*this) += cloud);
}

std::vector<double> PointCloud2::ComputePointCloudDistance(const PointCloud2 &target) {
    std::vector<double> distances(points_.size());
    KDTreeFlann kdtree;
    kdtree.SetGeometry(target);
#pragma omp parallel for schedule(static)
    for (int i = 0; i < (int)points_.size(); i++) {
        std::vector<int> indices(1);
        std::vector<double> dists(1);
        if (kdtree.SearchKNN(points_[i], 1, indices, dists) == 0) {
            utility::LogDebug(
                    "[ComputePointCloudToPointCloudDistance] Found a point "
                    "without neighbors.");
            distances[i] = 0.0;
        } else {
            distances[i] = std::sqrt(dists[0]);
        }
    }
    return distances;
}

PointCloud2 & PointCloud2::RemoveNonFinitePoints(bool remove_nan, bool remove_infinite) {
    bool has_normal = HasNormals();
    bool has_color = HasColors();
    bool has_intensity = HasIntensity();
    size_t old_point_num = points_.size();
    size_t k = 0;                                 // new index
//#pragma omp parallel for schedule(static)
    for (size_t i = 0; i < old_point_num; i++) {  // old index
        bool is_nan = remove_nan &&
                      (std::isnan(points_[i](0)) || std::isnan(points_[i](1)) ||
                       std::isnan(points_[i](2)));
        bool is_infinite = remove_infinite && (std::isinf(points_[i](0)) ||
                                               std::isinf(points_[i](1)) ||
                                               std::isinf(points_[i](2)));
        if (!is_nan && !is_infinite) {
            points_[k] = points_[i];
            if (has_normal) normals_[k] = normals_[i];
            if (has_color) colors_[k] = colors_[i];
            if (has_intensity) intensity_[k] = intensity_[i];
            k++;
        }
    }
    points_.resize(k);
    if (has_normal) normals_.resize(k);
    if (has_color) colors_.resize(k);
    if (has_intensity) intensity_.resize(k);
    utility::LogDebug(
            "[RemoveNonFinitePoints] {:d} nan points have been removed.",
            (int)(old_point_num - k));
    return *this;
}

std::shared_ptr<PointCloud2> PointCloud2::SelectByIndex(const std::vector<size_t> &indices, bool invert) const {
    auto output = std::make_shared<PointCloud2>();
    bool has_normals = HasNormals();
    bool has_colors = HasColors();
    bool has_intensity = HasIntensity();

    std::vector<bool> mask = std::vector<bool>(points_.size(), invert);
    for (size_t i : indices) {
        mask[i] = !invert;
    }
    size_t k = indices.size();
    size_t m = points_.size();
    if (!invert){
        output->points_.reserve(k);
        if (has_normals) output->normals_.reserve(k);
        if (has_colors) output->colors_.reserve(k);
        if (has_intensity) output->intensity_.reserve(k);
    }
    else {
        output->points_.reserve(m - k);
        if (has_normals) output->normals_.reserve(m - k);
        if (has_colors) output->colors_.reserve(m - k);
        if (has_intensity) output->intensity_.reserve(m - k);
    }

    for (size_t i = 0; i < m; i++) {
        if (mask[i]) {
            output->points_.push_back(points_[i]);
            if (has_normals) output->normals_.push_back(normals_[i]);
            if (has_colors) output->colors_.push_back(colors_[i]);
            if (has_intensity) output->intensity_.push_back(intensity_[i]);
        }
    }
    utility::LogDebug(
            "Pointcloud down sampled from {:d} points to {:d} points.",
            (int)points_.size(), (int)output->points_.size());
    return output;
}

namespace {
    class AccumulatedPoint {
    public:
        AccumulatedPoint()
                : num_of_points_(0),
                  point_(0.0, 0.0, 0.0),
                  normal_(0.0, 0.0, 0.0),
                  color_(0.0, 0.0, 0.0), intensity_(0.0) {}

    public:
        void AddPoint(const PointCloud2 &cloud, int index) {
            point_ += cloud.points_[index];
            if (cloud.HasNormals()) {
                if (!std::isnan(cloud.normals_[index](0)) &&
                    !std::isnan(cloud.normals_[index](1)) &&
                    !std::isnan(cloud.normals_[index](2))) {
                    normal_ += cloud.normals_[index];
                }
            }
            if (cloud.HasColors()) {
                color_ += cloud.colors_[index];
            }
            if (cloud.HasIntensity()) {
                intensity_ += cloud.intensity_[index];
            }
            num_of_points_++;
        }

        Eigen::Vector3d GetAveragePoint() const {
            return point_ / double(num_of_points_);
        }

        Eigen::Vector3d GetAverageNormal() const {
            // Call NormalizeNormals() afterwards if necessary
            return normal_ / double(num_of_points_);
        }

        Eigen::Vector3d GetAverageColor() const {
            return color_ / double(num_of_points_);
        }

        double GetAverageIntensity() const {
            return intensity_ / double(num_of_points_);
        }

    public:
        int num_of_points_;
        Eigen::Vector3d point_;
        Eigen::Vector3d normal_;
        Eigen::Vector3d color_;
        double intensity_;
    };

    class point_cubic_id {
    public:
        size_t point_id;
        int cubic_id;
    };

    class AccumulatedPointForTrace : public AccumulatedPoint {
    public:
        void AddPoint(const PointCloud2 &cloud,
                      size_t index,
                      int cubic_index,
                      bool approximate_class) {
            point_ += cloud.points_[index];
            if (cloud.HasNormals()) {
                if (!std::isnan(cloud.normals_[index](0)) &&
                    !std::isnan(cloud.normals_[index](1)) &&
                    !std::isnan(cloud.normals_[index](2))) {
                    normal_ += cloud.normals_[index];
                }
            }
            if (cloud.HasColors()) {
                if (approximate_class) {
                    auto got = classes.find(int(cloud.colors_[index][0]));
                    if (got == classes.end())
                        classes[int(cloud.colors_[index][0])] = 1;
                    else
                        classes[int(cloud.colors_[index][0])] += 1;
                } else {
                    color_ += cloud.colors_[index];
                }
            }
            if (cloud.HasIntensity()) {
                if (!std::isnan(cloud.intensity_[index])){
                    intensity_ += cloud.intensity_[index];
                }
            }
            point_cubic_id new_id{};
            new_id.point_id = index;
            new_id.cubic_id = cubic_index;
            original_id.push_back(new_id);
            num_of_points_++;
        }

        Eigen::Vector3d GetMaxClass() {
            int max_class = -1;
            int max_count = -1;
            for (auto & item : classes) {
                if (item.second > max_count) {
                    max_count = item.second;
                    max_class = item.first;
                }
            }
            return Eigen::Vector3d(max_class, max_class, max_class);
        }

        std::vector<point_cubic_id> GetOriginalID() { return original_id; }

    private:
        // original point cloud id in higher resolution + its cubic id
        std::vector<point_cubic_id> original_id;
        std::unordered_map<int, int> classes;
    };
} // namespace

std::shared_ptr<PointCloud2> PointCloud2::VoxelDownSample(double voxel_size) const {
    auto output = std::make_shared<PointCloud2>();
    if (voxel_size <= 0.0){
        utility::LogError("[VoxelDownSample] voxel_size <= 0.");
    }

    Eigen::Vector3d voxel_size3 =
            Eigen::Vector3d(voxel_size, voxel_size, voxel_size);
    Eigen::Vector3d voxel_min_bound = GetMinBound() - voxel_size3 * 0.5;
    Eigen::Vector3d voxel_max_bound = GetMaxBound() + voxel_size3 * 0.5;
    if (voxel_size * std::numeric_limits<int>::max() <
        (voxel_max_bound - voxel_min_bound).maxCoeff()) {
        utility::LogError("[VoxelDownSample] voxel_size is too small.");
    }
    std::unordered_map<Eigen::Vector3i, AccumulatedPoint,
            utility::hash_eigen<Eigen::Vector3i>>
            voxelindex_to_accpoint;

    Eigen::Vector3d ref_coord;
    Eigen::Vector3i voxel_index;
    for (int i = 0; i < (int)points_.size(); i++) {
        ref_coord = (points_[i] - voxel_min_bound) / voxel_size;
        voxel_index << int(floor(ref_coord(0))), int(floor(ref_coord(1))),
                int(floor(ref_coord(2)));
        voxelindex_to_accpoint[voxel_index].AddPoint(*this, i);
    }
    bool has_normals = HasNormals();
    bool has_colors = HasColors();
    bool has_intensity = HasIntensity();
    for (const auto& item : voxelindex_to_accpoint) {
        output->points_.push_back(item.second.GetAveragePoint());
        if (has_normals) {
            output->normals_.push_back(item.second.GetAverageNormal());
        }
        if (has_colors) {
            output->colors_.push_back(item.second.GetAverageColor());
        }
        if (has_intensity) {
            output->intensity_.push_back(item.second.GetAverageIntensity());
        }
    }
    utility::LogDebug(
            "Pointcloud down sampled from {:d} points to {:d} points.",
            (int)points_.size(), (int)output->points_.size());
    return output;
}

std::tuple<std::shared_ptr<PointCloud2>, Eigen::MatrixXi, std::vector<std::vector<int>>> PointCloud2::VoxelDownSampleAndTrace(
        double voxel_size, const Eigen::Vector3d &min_bound, const Eigen::Vector3d &max_bound,
        bool approximate_class) const {
    auto output = std::make_shared<PointCloud2>();
    Eigen::MatrixXi cubic_id;
    if (voxel_size <= 0.0) {
        utility::LogError("[VoxelDownSample] voxel_size <= 0.");
    }
    // Note: this is different from VoxelDownSample.
    // It is for fixing coordinate for multiscale voxel space
    const auto& voxel_min_bound = min_bound;
    const auto& voxel_max_bound = max_bound;
    if (voxel_size * std::numeric_limits<int>::max() <
        (voxel_max_bound - voxel_min_bound).maxCoeff()) {
        utility::LogError("[VoxelDownSample] voxel_size is too small.");
    }
    std::unordered_map<Eigen::Vector3i, AccumulatedPointForTrace,
            utility::hash_eigen<Eigen::Vector3i>>
            voxelindex_to_accpoint;
    int cid_temp[3] = {1, 2, 4};
    for (size_t i = 0; i < points_.size(); i++) {
        auto ref_coord = (points_[i] - voxel_min_bound) / voxel_size;
        auto voxel_index = Eigen::Vector3i(int(floor(ref_coord(0))),
                                           int(floor(ref_coord(1))),
                                           int(floor(ref_coord(2))));
        int cid = 0;
        for (int c = 0; c < 3; c++) {
            if ((ref_coord(c) - voxel_index(c)) >= 0.5) {
                cid += cid_temp[c];
            }
        }
        voxelindex_to_accpoint[voxel_index].AddPoint(*this, i, cid,
                                                     approximate_class);
    }
    bool has_normals = HasNormals();
    bool has_colors = HasColors();
    bool has_intensity = HasIntensity();
    int cnt = 0;
    cubic_id.resize(voxelindex_to_accpoint.size(), 8);
    cubic_id.setConstant(-1);
    std::vector<std::vector<int>> original_indices(
            voxelindex_to_accpoint.size());
    for (auto item : voxelindex_to_accpoint) {
        output->points_.push_back(item.second.GetAveragePoint());
        if (has_normals) {
            output->normals_.push_back(item.second.GetAverageNormal());
        }
        if (has_colors) {
            if (approximate_class) {
                output->colors_.push_back(item.second.GetMaxClass());
            } else {
                output->colors_.push_back(item.second.GetAverageColor());
            }
        }
        if (has_intensity){
            output->intensity_.push_back(item.second.GetAverageIntensity());
        }
        auto original_id = item.second.GetOriginalID();
        for (auto & originalId : original_id) {
            size_t pid = originalId.point_id;
            int cid = originalId.cubic_id;
            cubic_id(cnt, cid) = int(pid);
            original_indices[cnt].push_back(int(pid));
        }
        cnt++;
    }
    utility::LogDebug(
            "Pointcloud down sampled from {:d} points to {:d} points.",
            (int)points_.size(), (int)output->points_.size());
    return std::make_tuple(output, cubic_id, original_indices);
}

std::shared_ptr<PointCloud2> PointCloud2::UniformDownSample(size_t every_k_points) const {
    if (every_k_points == 0) {
        utility::LogError("[UniformDownSample] Illegal sample rate.");
    }
    std::vector<size_t> indices;
    for (size_t i = 0; i < points_.size(); i += every_k_points) {
        indices.push_back(i);
    }
    return SelectByIndex(indices);
}

std::shared_ptr<PointCloud2> PointCloud2::RandomDownSample(double sampling_ratio) const {
    if (sampling_ratio < 0 || sampling_ratio > 1) {
        utility::LogError(
                "[RandomDownSample] Illegal sampling_ratio {}, sampling_ratio "
                "must be between 0 and 1.");
    }
    std::vector<size_t> indices(points_.size());
    std::iota(std::begin(indices), std::end(indices), (size_t)0);
    std::random_device rd;
    std::mt19937 prng(rd());
    std::shuffle(indices.begin(), indices.end(), prng);
    indices.resize((int)(sampling_ratio * double(points_.size())));
    return SelectByIndex(indices);
}

std::shared_ptr<PointCloud2> PointCloud2::RandomDownSample(size_t numbers, bool dummy) const {
    if (numbers > this->points_.size()){
        utility::LogError("the sample number of points less than the original cloud.");
        return nullptr;
    }
    auto output_ptr = std::make_shared<PointCloud2>();
    output_ptr->points_.resize(numbers);

    if (HasNormals()){
        output_ptr->normals_.resize(numbers);
    }
    if (HasColors()){
        output_ptr->colors_.resize(numbers);
    }
    if (HasIntensity()){
        output_ptr->intensity_.resize(numbers);
    }

    int size = static_cast<int>(points_.size());
    for (size_t i = 0; i < numbers; i++){
        int sample_id = utility::UniformRandInt(0, size);
        output_ptr->points_[i] = points_[sample_id];
        if (HasNormals()){
            output_ptr->normals_[i] = normals_[sample_id];
        }
        if (HasIntensity()){
            output_ptr->intensity_[i] = intensity_[sample_id];
        }
        if (HasColors()){
            output_ptr->colors_[i] = colors_[sample_id];
        }
    }

    utility::LogDebug("Pointcloud random down sampled from {:d} points to {:d} points.",
                      static_cast<int>(points_.size()), static_cast<int>(output_ptr->points_.size()));
    return output_ptr;
}


std::shared_ptr<PointCloud2> PointCloud2::Crop(
        const AxisAlignedBoundingBox &bbox) const {
    if (bbox.IsEmpty()) {
        utility::LogError(
                "[CropPointCloud] AxisAlignedBoundingBox either has zeros "
                "size, or has wrong bounds.");
    }
    return SelectByIndex(bbox.GetPointIndicesWithinBoundingBox(points_));
}

std::shared_ptr<PointCloud2> PointCloud2::Crop(
        const OrientedBoundingBox &bbox) const {
    if (bbox.IsEmpty()) {
        utility::LogError(
                "[CropPointCloud] AxisAlignedBoundingBox either has zeros "
                "size, or has wrong bounds.");
    }
    return SelectByIndex(bbox.GetPointIndicesWithinBoundingBox(points_));
}

std::tuple<std::shared_ptr<PointCloud2>, std::vector<size_t>> PointCloud2::RemoveRadiusOutliers(size_t nb_points,
                                                                                                double search_radius) const {
    if (nb_points < 1 || search_radius <= 0) {
        utility::LogError(
                "[RemoveRadiusOutliers] Illegal input parameters,"
                "number of points and radius must be positive");
    }
    KDTreeFlann kdtree;
    kdtree.SetGeometry(*this);
    std::vector<bool> mask = std::vector<bool>(points_.size());
#pragma omp parallel for schedule(static)
    for (int i = 0; i < int(points_.size()); i++) {
        std::vector<int> tmp_indices;
        std::vector<double> dist;
        size_t nb_neighbors = kdtree.SearchRadius(points_[i], search_radius,
                                                  tmp_indices, dist);
        mask[i] = (nb_neighbors > nb_points);
    }
    std::vector<size_t> indices;
    for (size_t i = 0; i < mask.size(); i++) {
        if (mask[i]) {
            indices.push_back(i);
        }
    }
    return std::make_tuple(SelectByIndex(indices), indices);
}

std::tuple<std::shared_ptr<PointCloud2>, std::vector<size_t>> PointCloud2::RemoveStatisticalOutliers(
        size_t nb_neighbors, double std_ratio) const {
    if (nb_neighbors < 1 || std_ratio <= 0) {
        utility::LogError(
                "[RemoveStatisticalOutliers] Illegal input parameters, number "
                "of neighbors and standard deviation ratio must be positive");
    }
    if (points_.size() == 0) {
        return std::make_tuple(std::make_shared<PointCloud2>(),
                               std::vector<size_t>());
    }
    KDTreeFlann kdtree;
    kdtree.SetGeometry(*this);
    std::vector<double> avg_distances = std::vector<double>(points_.size());
    std::vector<size_t> indices;
    size_t valid_distances = 0;

#pragma omp parallel for schedule(static)
    for (int i = 0; i < int(points_.size()); i++) {
        std::vector<int> tmp_indices;
        std::vector<double> dist;
        kdtree.SearchKNN(points_[i], int(nb_neighbors), tmp_indices, dist);
        double mean = -1.0;
        if (!dist.empty()) {
            valid_distances++;
            std::for_each(dist.begin(), dist.end(),
                          [](double &d) { d = std::sqrt(d); });
            mean = std::accumulate(dist.begin(), dist.end(), 0.0) / dist.size();
        }
        avg_distances[i] = mean;
    }
    if (valid_distances == 0) {
        return std::make_tuple(std::make_shared<PointCloud2>(),
                               std::vector<size_t>());
    }
    double cloud_mean = std::accumulate(
            avg_distances.begin(), avg_distances.end(), 0.0,
            [](double const &x, double const &y) { return y > 0 ? x + y : x; });
    cloud_mean /= double(valid_distances);
    double sq_sum = std::inner_product(
            avg_distances.begin(), avg_distances.end(), avg_distances.begin(),
            0.0, [](double const &x, double const &y) { return x + y; },
            [cloud_mean](double const &x, double const &y) {
                return x > 0 ? (x - cloud_mean) * (y - cloud_mean) : 0;
            });
    // Bessel's correction
    double std_dev = std::sqrt(sq_sum / double(valid_distances - 1));
    double distance_threshold = cloud_mean + std_ratio * std_dev;
    for (size_t i = 0; i < avg_distances.size(); i++) {
        if (avg_distances[i] > 0 && avg_distances[i] < distance_threshold) {
            indices.push_back(i);
        }
    }
    return std::make_tuple(SelectByIndex(indices), indices);
}

std::tuple<Eigen::Vector3d, Eigen::Matrix3d> PointCloud2::ComputeMeanAndCovariance() const {
    if (IsEmpty()) {
        return std::make_tuple(Eigen::Vector3d::Zero(),
                               Eigen::Matrix3d::Identity());
    }
    std::vector<size_t> all_idx(points_.size());
    std::iota(all_idx.begin(), all_idx.end(), 0);
    return utility::ComputeMeanAndCovariance(points_, all_idx);
}

std::vector<double> PointCloud2::ComputeMahalanobisDistance() const {
    std::vector<double> mahalanobis(points_.size());
    Eigen::Vector3d mean;
    Eigen::Matrix3d covariance;
    std::tie(mean, covariance) = ComputeMeanAndCovariance();
    Eigen::Matrix3d cov_inv = covariance.inverse();
#pragma omp parallel for schedule(static)
    for (int i = 0; i < (int)points_.size(); i++) {
        Eigen::Vector3d p = points_[i] - mean;
        mahalanobis[i] = std::sqrt(p.transpose() * cov_inv * p);
    }
    return mahalanobis;
}

std::vector<double> PointCloud2::ComputeNearestNeighborDistance() const {
    if (points_.size() < 2) {
        return std::vector<double>(points_.size(), 0);
    }

    std::vector<double> nn_dis(points_.size());
    KDTreeFlann kdtree(*this);
#pragma omp parallel for schedule(static)
    for (int i = 0; i < (int)points_.size(); i++) {
        std::vector<int> indices(2);
        std::vector<double> dists(2);
        if (kdtree.SearchKNN(points_[i], 2, indices, dists) <= 1) {
            utility::LogDebug(
                    "[ComputePointCloudNearestNeighborDistance] Found a point "
                    "without neighbors.");
            nn_dis[i] = 0.0;
        } else {
            nn_dis[i] = std::sqrt(dists[1]);
        }
    }
    return nn_dis;
}

std::tuple<std::shared_ptr<TriangleMesh>, std::vector<size_t>> PointCloud2::ComputeConvexHull() const {
    return Qhull::ComputeConvexHull(points_);
}

std::tuple<std::shared_ptr<TriangleMesh>, std::vector<size_t>> PointCloud2::HiddenPointRemoval(
        const Eigen::Vector3d &camera_location, const double radius) const {
    if (radius <= 0) {
        utility::LogError(
                "[HiddenPointRemoval] radius must be larger than zero.");
    }

    // perform spherical projection
    std::vector<Eigen::Vector3d> spherical_projection;
    for (size_t pidx = 0; pidx < points_.size(); ++pidx) {
        Eigen::Vector3d projected_point = points_[pidx] - camera_location;
        double norm = projected_point.norm();
        spherical_projection.push_back(
                projected_point + 2 * (radius - norm) * projected_point / norm);
    }

    // add origin
    size_t origin_pidx = spherical_projection.size();
    spherical_projection.push_back(Eigen::Vector3d(0, 0, 0));

    // calculate convex hull of spherical projection
    std::shared_ptr<TriangleMesh> visible_mesh;
    std::vector<size_t> pt_map;
    std::tie(visible_mesh, pt_map) =
            Qhull::ComputeConvexHull(spherical_projection);

    // reassign original points to mesh
    size_t origin_vidx = pt_map.size();
    for (size_t vidx = 0; vidx < pt_map.size(); vidx++) {
        size_t pidx = pt_map[vidx];
        if (pidx != origin_pidx) {
            visible_mesh->vertices_[vidx] = points_[pidx];
        } else {
            origin_vidx = vidx;
            visible_mesh->vertices_[vidx] = camera_location;
        }
    }

    // erase origin if part of mesh
    if (origin_vidx < visible_mesh->vertices_.size()) {
        visible_mesh->vertices_.erase(visible_mesh->vertices_.begin() +
                                      origin_vidx);
        pt_map.erase(pt_map.begin() + origin_vidx);
        for (size_t tidx = visible_mesh->triangles_.size(); tidx-- > 0;) {
            if (visible_mesh->triangles_[tidx](0) == (int)origin_vidx ||
                visible_mesh->triangles_[tidx](1) == (int)origin_vidx ||
                visible_mesh->triangles_[tidx](2) == (int)origin_vidx) {
                visible_mesh->triangles_.erase(
                        visible_mesh->triangles_.begin() + tidx);
            } else {
                if (visible_mesh->triangles_[tidx](0) > (int)origin_vidx)
                    visible_mesh->triangles_[tidx](0) -= 1;
                if (visible_mesh->triangles_[tidx](1) > (int)origin_vidx)
                    visible_mesh->triangles_[tidx](1) -= 1;
                if (visible_mesh->triangles_[tidx](2) > (int)origin_vidx)
                    visible_mesh->triangles_[tidx](2) -= 1;
            }
        }
    }
    return std::make_tuple(visible_mesh, pt_map);
}

namespace {
    using namespace geometry;

// Disjoint set data structure to find cycles in graphs
    class DisjointSet {
    public:
        DisjointSet(size_t size) : parent_(size), size_(size) {
            for (size_t idx = 0; idx < size; idx++) {
                parent_[idx] = idx;
                size_[idx] = 0;
            }
        }

        // find representative element for given x
        // using path compression
        size_t Find(size_t x) {
            if (x != parent_[x]) {
                parent_[x] = Find(parent_[x]);
            }
            return parent_[x];
        }

        // combine two sets using size of sets
        void Union(size_t x, size_t y) {
            x = Find(x);
            y = Find(y);
            if (x != y) {
                if (size_[x] < size_[y]) {
                    size_[y] += size_[x];
                    parent_[x] = y;
                } else {
                    size_[x] += size_[y];
                    parent_[y] = x;
                }
            }
        }

    private:
        std::vector<size_t> parent_;
        std::vector<size_t> size_;
    };

    struct WeightedEdge {
        WeightedEdge(size_t v0, size_t v1, double weight)
                : v0_(v0), v1_(v1), weight_(weight) {}
        size_t v0_;
        size_t v1_;
        double weight_;
    };

    Eigen::Vector3d ComputeEigenvector0(const Eigen::Matrix3d &A, double eval0) {
        Eigen::Vector3d row0(A(0, 0) - eval0, A(0, 1), A(0, 2));
        Eigen::Vector3d row1(A(0, 1), A(1, 1) - eval0, A(1, 2));
        Eigen::Vector3d row2(A(0, 2), A(1, 2), A(2, 2) - eval0);
        Eigen::Vector3d r0xr1 = row0.cross(row1);
        Eigen::Vector3d r0xr2 = row0.cross(row2);
        Eigen::Vector3d r1xr2 = row1.cross(row2);
        double d0 = r0xr1.dot(r0xr1);
        double d1 = r0xr2.dot(r0xr2);
        double d2 = r1xr2.dot(r1xr2);

        double dmax = d0;
        int imax = 0;
        if (d1 > dmax) {
            dmax = d1;
            imax = 1;
        }
        if (d2 > dmax) {
            imax = 2;
        }

        if (imax == 0) {
            return r0xr1 / std::sqrt(d0);
        } else if (imax == 1) {
            return r0xr2 / std::sqrt(d1);
        } else {
            return r1xr2 / std::sqrt(d2);
        }
    }

    Eigen::Vector3d ComputeEigenvector1(const Eigen::Matrix3d &A,
                                        const Eigen::Vector3d &evec0,
                                        double eval1) {
        Eigen::Vector3d U, V;
        if (std::abs(evec0(0)) > std::abs(evec0(1))) {
            double inv_length =
                    1 / std::sqrt(evec0(0) * evec0(0) + evec0(2) * evec0(2));
            U << -evec0(2) * inv_length, 0, evec0(0) * inv_length;
        } else {
            double inv_length =
                    1 / std::sqrt(evec0(1) * evec0(1) + evec0(2) * evec0(2));
            U << 0, evec0(2) * inv_length, -evec0(1) * inv_length;
        }
        V = evec0.cross(U);

        Eigen::Vector3d AU(A(0, 0) * U(0) + A(0, 1) * U(1) + A(0, 2) * U(2),
                           A(0, 1) * U(0) + A(1, 1) * U(1) + A(1, 2) * U(2),
                           A(0, 2) * U(0) + A(1, 2) * U(1) + A(2, 2) * U(2));

        Eigen::Vector3d AV = {A(0, 0) * V(0) + A(0, 1) * V(1) + A(0, 2) * V(2),
                              A(0, 1) * V(0) + A(1, 1) * V(1) + A(1, 2) * V(2),
                              A(0, 2) * V(0) + A(1, 2) * V(1) + A(2, 2) * V(2)};

        double m00 = U(0) * AU(0) + U(1) * AU(1) + U(2) * AU(2) - eval1;
        double m01 = U(0) * AV(0) + U(1) * AV(1) + U(2) * AV(2);
        double m11 = V(0) * AV(0) + V(1) * AV(1) + V(2) * AV(2) - eval1;

        double absM00 = std::abs(m00);
        double absM01 = std::abs(m01);
        double absM11 = std::abs(m11);
        double max_abs_comp;
        if (absM00 >= absM11) {
            max_abs_comp = std::max(absM00, absM01);
            if (max_abs_comp > 0) {
                if (absM00 >= absM01) {
                    m01 /= m00;
                    m00 = 1 / std::sqrt(1 + m01 * m01);
                    m01 *= m00;
                } else {
                    m00 /= m01;
                    m01 = 1 / std::sqrt(1 + m00 * m00);
                    m00 *= m01;
                }
                return m01 * U - m00 * V;
            } else {
                return U;
            }
        } else {
            max_abs_comp = std::max(absM11, absM01);
            if (max_abs_comp > 0) {
                if (absM11 >= absM01) {
                    m01 /= m11;
                    m11 = 1 / std::sqrt(1 + m01 * m01);
                    m01 *= m11;
                } else {
                    m11 /= m01;
                    m01 = 1 / std::sqrt(1 + m11 * m11);
                    m11 *= m01;
                }
                return m11 * U - m01 * V;
            } else {
                return U;
            }
        }
    }

    Eigen::Vector3d FastEigen3x3(Eigen::Matrix3d &A) {
        // Previous version based on:
        // https://en.wikipedia.org/wiki/Eigenvalue_algorithm#3.C3.973_matrices
        // Current version based on
        // https://www.geometrictools.com/Documentation/RobustEigenSymmetric3x3.pdf
        // which handles edge cases like points on a plane

        double max_coeff = A.maxCoeff();
        if (max_coeff == 0) {
            return Eigen::Vector3d::Zero();
        }
        A /= max_coeff;

        double norm = A(0, 1) * A(0, 1) + A(0, 2) * A(0, 2) + A(1, 2) * A(1, 2);
        if (norm > 0) {
            Eigen::Vector3d eval;
            Eigen::Vector3d evec0;
            Eigen::Vector3d evec1;
            Eigen::Vector3d evec2;

            double q = (A(0, 0) + A(1, 1) + A(2, 2)) / 3;

            double b00 = A(0, 0) - q;
            double b11 = A(1, 1) - q;
            double b22 = A(2, 2) - q;

            double p =
                    std::sqrt((b00 * b00 + b11 * b11 + b22 * b22 + norm * 2) / 6);

            double c00 = b11 * b22 - A(1, 2) * A(1, 2);
            double c01 = A(0, 1) * b22 - A(1, 2) * A(0, 2);
            double c02 = A(0, 1) * A(1, 2) - b11 * A(0, 2);
            double det = (b00 * c00 - A(0, 1) * c01 + A(0, 2) * c02) / (p * p * p);

            double half_det = det * 0.5;
            half_det = std::min(std::max(half_det, -1.0), 1.0);

            double angle = std::acos(half_det) / (double)3;
            double const two_thirds_pi = 2.09439510239319549;
            double beta2 = std::cos(angle) * 2;
            double beta0 = std::cos(angle + two_thirds_pi) * 2;
            double beta1 = -(beta0 + beta2);

            eval(0) = q + p * beta0;
            eval(1) = q + p * beta1;
            eval(2) = q + p * beta2;

            if (half_det >= 0) {
                evec2 = ComputeEigenvector0(A, eval(2));
                if (eval(2) < eval(0) && eval(2) < eval(1)) {
                    A *= max_coeff;
                    return evec2;
                }
                evec1 = ComputeEigenvector1(A, evec2, eval(1));
                A *= max_coeff;
                if (eval(1) < eval(0) && eval(1) < eval(2)) {
                    return evec1;
                }
                evec0 = evec1.cross(evec2);
                return evec0;
            } else {
                evec0 = ComputeEigenvector0(A, eval(0));
                if (eval(0) < eval(1) && eval(0) < eval(2)) {
                    A *= max_coeff;
                    return evec0;
                }
                evec1 = ComputeEigenvector1(A, evec0, eval(1));
                A *= max_coeff;
                if (eval(1) < eval(0) && eval(1) < eval(2)) {
                    return evec1;
                }
                evec2 = evec0.cross(evec1);
                return evec2;
            }
        } else {
            A *= max_coeff;
            if (A(0, 0) < A(1, 1) && A(0, 0) < A(2, 2)) {
                return Eigen::Vector3d(1, 0, 0);
            } else if (A(1, 1) < A(0, 0) && A(1, 1) < A(2, 2)) {
                return Eigen::Vector3d(0, 1, 0);
            } else {
                return Eigen::Vector3d(0, 0, 1);
            }
        }
    }

    Eigen::Vector3d ComputeNormal(const PointCloud2 &cloud,
                                  const std::vector<int> &indices,
                                  bool fast_normal_computation) {
        if (indices.size() == 0) {
            return Eigen::Vector3d::Zero();
        }
        Eigen::Matrix3d covariance =
                utility::ComputeCovariance(cloud.points_, indices);

        if (fast_normal_computation) {
            return FastEigen3x3(covariance);
        } else {
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver;
            solver.compute(covariance, Eigen::ComputeEigenvectors);
            return solver.eigenvectors().col(0);
        }
    }

// Minimum Spanning Tree algorithm (Kruskal's algorithm)
    std::vector<WeightedEdge> Kruskal(std::vector<WeightedEdge> &edges,
                                      size_t n_vertices) {
        std::sort(edges.begin(), edges.end(),
                  [](WeightedEdge &e0, WeightedEdge &e1) {
                      return e0.weight_ < e1.weight_;
                  });
        DisjointSet disjoint_set(n_vertices);
        std::vector<WeightedEdge> mst;
        for (size_t eidx = 0; eidx < edges.size(); ++eidx) {
            size_t set0 = disjoint_set.Find(edges[eidx].v0_);
            size_t set1 = disjoint_set.Find(edges[eidx].v1_);
            if (set0 != set1) {
                mst.push_back(edges[eidx]);
                disjoint_set.Union(set0, set1);
            }
        }
        return mst;
    }

// Find the plane such that the summed squared distance from the
// plane to all points is minimized.
//
// Reference:
// https://www.ilikebigbits.com/2015_03_04_plane_from_points.html
    Eigen::Vector4d GetPlaneFromPoints(const std::vector<Eigen::Vector3d> &points,
                                       const std::vector<size_t> &inliers) {
        Eigen::Vector3d centroid(0, 0, 0);
        for (size_t idx : inliers) {
            centroid += points[idx];
        }
        centroid /= double(inliers.size());

        double xx = 0, xy = 0, xz = 0, yy = 0, yz = 0, zz = 0;

        for (size_t idx : inliers) {
            Eigen::Vector3d r = points[idx] - centroid;
            xx += r(0) * r(0);
            xy += r(0) * r(1);
            xz += r(0) * r(2);
            yy += r(1) * r(1);
            yz += r(1) * r(2);
            zz += r(2) * r(2);
        }

        double det_x = yy * zz - yz * yz;
        double det_y = xx * zz - xz * xz;
        double det_z = xx * yy - xy * xy;

        Eigen::Vector3d abc;
        if (det_x > det_y && det_x > det_z) {
            abc = Eigen::Vector3d(det_x, xz * yz - xy * zz, xy * yz - xz * yy);
        } else if (det_y > det_z) {
            abc = Eigen::Vector3d(xz * yz - xy * zz, det_y, xy * xz - yz * xx);
        } else {
            abc = Eigen::Vector3d(xy * yz - xz * yy, xy * xz - yz * xx, det_z);
        }

        double norm = abc.norm();
        // Return invalid plane if the points don't span a plane.
        if (norm == 0) {
            return Eigen::Vector4d(0, 0, 0, 0);
        }
        abc /= abc.norm();
        double d = -abc.dot(centroid);
        return Eigen::Vector4d(abc(0), abc(1), abc(2), d);
    }
} // unnamed namespace


void PointCloud2::EstimateNormals(const KDTreeSearchParam &search_param, bool fast_normal_computation) {
    bool has_normal = HasNormals();
    if (!has_normal) {
        normals_.resize(points_.size());
    }
    KDTreeFlann kdtree;
    kdtree.SetGeometry(*this);
#pragma omp parallel for schedule(static)
    for (int i = 0; i < (int)points_.size(); i++) {
        std::vector<int> indices;
        std::vector<double> distance2;
        Eigen::Vector3d normal;
        if (kdtree.Search(points_[i], search_param, indices, distance2) >= 3) {
            normal = ComputeNormal(*this, indices, fast_normal_computation);
            if (normal.norm() == 0.0) {
                if (has_normal) {
                    normal = normals_[i];
                } else {
                    normal = Eigen::Vector3d(0.0, 0.0, 1.0);
                }
            }
            if (has_normal && normal.dot(normals_[i]) < 0.0) {
                normal *= -1.0;
            }
            normals_[i] = normal;
        } else {
            normals_[i] = Eigen::Vector3d(0.0, 0.0, 1.0);
        }
    }
}

void PointCloud2::OrientNormalsToAlignWithDirection(const Eigen::Vector3d &orientation_reference) {
    if (!HasNormals()) {
        utility::LogError(
                "[OrientNormalsToAlignWithDirection] No normals in the "
                "PointCloud. Call EstimateNormals() first.");
    }
#pragma omp parallel for schedule(static)
    for (int i = 0; i < (int)points_.size(); i++) {
        auto &normal = normals_[i];
        if (normal.norm() == 0.0) {
            normal = orientation_reference;
        } else if (normal.dot(orientation_reference) < 0.0) {
            normal *= -1.0;
        }
    }
}

void PointCloud2::OrientNormalsTowardsCameraLocation(const Eigen::Vector3d &camera_location) {
    if (!HasNormals()) {
        utility::LogError(
                "[OrientNormalsTowardsCameraLocation] No normals in the "
                "PointCloud. Call EstimateNormals() first.");
    }
#pragma omp parallel for schedule(static)
    for (int i = 0; i < (int)points_.size(); i++) {
        Eigen::Vector3d orientation_reference = camera_location - points_[i];
        auto &normal = normals_[i];
        if (normal.norm() == 0.0) {
            normal = orientation_reference;
            if (normal.norm() == 0.0) {
                normal = Eigen::Vector3d(0.0, 0.0, 1.0);
            } else {
                normal.normalize();
            }
        } else if (normal.dot(orientation_reference) < 0.0) {
            normal *= -1.0;
        }
    }
}

void PointCloud2::OrientNormalsConsistentTangentPlane(size_t k) {
    if (!HasNormals()) {
        utility::LogError(
                "[OrientNormalsConsistentTangentPlane] No normals in the "
                "PointCloud. Call EstimateNormals() first.");
    }

    // Create Riemannian graph (Euclidian MST + kNN)
    // Euclidian MST is subgraph of Delaunay triangulation
    std::shared_ptr<TetraMesh> delaunay_mesh;
    std::vector<size_t> pt_map;
    std::tie(delaunay_mesh, pt_map) = TetraMesh::CreateFromPointCloud(PointCloud(this->points_));
    std::vector<WeightedEdge> delaunay_graph;
    std::unordered_set<size_t> graph_edges;
    auto EdgeIndex = [&](size_t v0, size_t v1) -> size_t {
        return std::min(v0, v1) * points_.size() + std::max(v0, v1);
    };
    auto AddEdgeToDelaunayGraph = [&](size_t v0, size_t v1) {
        v0 = pt_map[v0];
        v1 = pt_map[v1];
        size_t edge = EdgeIndex(v0, v1);
        if (graph_edges.count(edge) == 0) {
            double dist = (points_[v0] - points_[v1]).squaredNorm();
            delaunay_graph.emplace_back(v0, v1, dist);
            graph_edges.insert(edge);
        }
    };
    for (const Eigen::Vector4i &tetra : delaunay_mesh->tetras_) {
        AddEdgeToDelaunayGraph(tetra[0], tetra[1]);
        AddEdgeToDelaunayGraph(tetra[0], tetra[2]);
        AddEdgeToDelaunayGraph(tetra[0], tetra[3]);
        AddEdgeToDelaunayGraph(tetra[1], tetra[2]);
        AddEdgeToDelaunayGraph(tetra[1], tetra[3]);
        AddEdgeToDelaunayGraph(tetra[2], tetra[3]);
    }

    std::vector<WeightedEdge> mst = Kruskal(delaunay_graph, points_.size());

    auto NormalWeight = [&](size_t v0, size_t v1) -> double {
        return 1.0 - std::abs(normals_[v0].dot(normals_[v1]));
    };
    for (auto &edge : mst) {
        edge.weight_ = NormalWeight(edge.v0_, edge.v1_);
    }

    // Add k nearest neighbors to Riemannian graph
    KDTreeFlann kdtree(*this);
    for (size_t v0 = 0; v0 < points_.size(); ++v0) {
        std::vector<int> neighbors;
        std::vector<double> dists2;
        kdtree.SearchKNN(points_[v0], int(k), neighbors, dists2);
        for (size_t vidx1 = 0; vidx1 < neighbors.size(); ++vidx1) {
            size_t v1 = size_t(neighbors[vidx1]);
            if (v0 == v1) {
                continue;
            }
            size_t edge = EdgeIndex(v0, v1);
            if (graph_edges.count(edge) == 0) {
                double weight = NormalWeight(v0, v1);
                mst.push_back(WeightedEdge(v0, v1, weight));
                graph_edges.insert(edge);
            }
        }
    }

    // extract MST from Riemannian graph
    mst = Kruskal(mst, points_.size());

    // convert list of edges to graph
    std::vector<std::unordered_set<size_t>> mst_graph(points_.size());
    for (const auto &edge : mst) {
        size_t v0 = edge.v0_;
        size_t v1 = edge.v1_;
        mst_graph[v0].insert(v1);
        mst_graph[v1].insert(v0);
    }

    // find start node for tree traversal
    // init with node that maximizes z
    double max_z = std::numeric_limits<double>::lowest();
    size_t v0;
    for (size_t vidx = 0; vidx < points_.size(); ++vidx) {
        const Eigen::Vector3d &v = points_[vidx];
        if (v(2) > max_z) {
            max_z = v(2);
            v0 = vidx;
        }
    }

    // traverse MST and orient normals consistently
    std::queue<size_t> traversal_queue;
    std::vector<bool> visited(points_.size(), false);
    traversal_queue.push(v0);
    auto TestAndOrientNormal = [&](const Eigen::Vector3d &n0,
                                   Eigen::Vector3d &n1) {
        if (n0.dot(n1) < 0) {
            n1 *= -1;
        }
    };
    TestAndOrientNormal(Eigen::Vector3d(0, 0, 1), normals_[v0]);
    while (!traversal_queue.empty()) {
        v0 = traversal_queue.front();
        traversal_queue.pop();
        visited[v0] = true;
        for (size_t v1 : mst_graph[v0]) {
            if (!visited[v1]) {
                traversal_queue.push(v1);
                TestAndOrientNormal(normals_[v0], normals_[v1]);
            }
        }
    }
}

std::vector<int> PointCloud2::ClusterDBSCAN(double eps, size_t min_points, bool print_progress) const {
    KDTreeFlann kdtree(*this);

    // Precompute all neighbors.
    utility::LogDebug("Precompute neighbors.");
    utility::ConsoleProgressBar progress_bar(
            points_.size(), "Precompute neighbors.", print_progress);
    std::vector<std::vector<int>> nbs(points_.size());
#pragma omp parallel for schedule(static)
    for (int idx = 0; idx < int(points_.size()); ++idx) {
        std::vector<double> dists2;
        kdtree.SearchRadius(points_[idx], eps, nbs[idx], dists2);

#pragma omp critical
        { ++progress_bar; }
    }
    utility::LogDebug("Done Precompute neighbors.");

    // Set all labels to undefined (-2).
    utility::LogDebug("Compute Clusters");
    progress_bar.Reset(points_.size(), "Clustering", print_progress);
    std::vector<int> labels(points_.size(), -2);
    int cluster_label = 0;
    for (size_t idx = 0; idx < points_.size(); ++idx) {
        // Label is not undefined.
        if (labels[idx] != -2) {
            continue;
        }

        // Check density.
        if (nbs[idx].size() < min_points) {
            labels[idx] = -1;
            continue;
        }

        std::unordered_set<int> nbs_next(nbs[idx].begin(), nbs[idx].end());
        std::unordered_set<int> nbs_visited;
        nbs_visited.insert(int(idx));

        labels[idx] = cluster_label;
        ++progress_bar;
        while (!nbs_next.empty()) {
            int nb = *nbs_next.begin();
            nbs_next.erase(nbs_next.begin());
            nbs_visited.insert(nb);

            // Noise label.
            if (labels[nb] == -1) {
                labels[nb] = cluster_label;
                ++progress_bar;
            }
            // Not undefined label.
            if (labels[nb] != -2) {
                continue;
            }
            labels[nb] = cluster_label;
            ++progress_bar;

            if (nbs[nb].size() >= min_points) {
                for (int qnb : nbs[nb]) {
                    if (nbs_visited.count(qnb) == 0) {
                        nbs_next.insert(qnb);
                    }
                }
            }
        }

        cluster_label++;
    }

    utility::LogDebug("Done Compute Clusters: {:d}", cluster_label);
    return labels;
}

namespace {
    using namespace geometry;
/// \class RANSACResult
///
/// \brief Stores the current best result in the RANSAC algorithm.
    class RANSACResult {
    public:
        RANSACResult() : fitness_(0), inlier_rmse_(0) {}
        ~RANSACResult() = default;

    public:
        double fitness_;
        double inlier_rmse_;
    };


// Calculates the number of inliers given a list of points and a plane model,
// and the total distance between the inliers and the plane. These numbers are
// then used to evaluate how well the plane model fits the given points.
    RANSACResult EvaluateRANSACBasedOnDistance(
            const std::vector<Eigen::Vector3d> &points,
            const Eigen::Vector4d plane_model,
            std::vector<size_t> &inliers,
            double distance_threshold,
            double error) {
        RANSACResult result;

        for (size_t idx = 0; idx < points.size(); ++idx) {
            Eigen::Vector4d point(points[idx](0), points[idx](1), points[idx](2),
                                  1);
            double distance = std::abs(plane_model.dot(point));

            if (distance < distance_threshold) {
                error += distance;
                inliers.emplace_back(idx);
            }
        }

        size_t inlier_num = inliers.size();
        if (inlier_num == 0) {
            result.fitness_ = 0;
            result.inlier_rmse_ = 0;
        } else {
            result.fitness_ = (double)inlier_num / (double)points.size();
            result.inlier_rmse_ = error / std::sqrt((double)inlier_num);
        }
        return result;
    }
} // unnamed namespace

std::tuple<Eigen::Vector4d, std::vector<size_t>> PointCloud2::SegmentPlane(
        const double distance_threshold /* = 0.01 */,
        const int ransac_n /* = 3 */,
        const int num_iterations /* = 100 */) const {
    RANSACResult result;
    double error = 0;

    // Initialize the plane model ax + by + cz + d = 0.
    Eigen::Vector4d plane_model = Eigen::Vector4d(0, 0, 0, 0);
    // Initialize the best plane model.
    Eigen::Vector4d best_plane_model = Eigen::Vector4d(0, 0, 0, 0);

    // Initialize consensus set.
    std::vector<size_t> inliers;

    size_t num_points = points_.size();
    std::vector<size_t> indices(num_points);
    std::iota(std::begin(indices), std::end(indices), 0);

    std::random_device rd;
    std::mt19937 rng(rd());

    // Return if ransac_n is less than the required plane model parameters.
    if (ransac_n < 3) {
        utility::LogError(
                "ransac_n should be set to higher than or equal to 3.");
        return std::make_tuple(best_plane_model, inliers);
    }
    if (num_points < size_t(ransac_n)) {
        utility::LogError("There must be at least 'ransac_n' points.");
        return std::make_tuple(best_plane_model, inliers);
    }

    for (int itr = 0; itr < num_iterations; itr++) {
        for (int i = 0; i < ransac_n; ++i) {
            std::swap(indices[i], indices[rng() % num_points]);
        }
        inliers.clear();
        for (int idx = 0; idx < ransac_n; ++idx) {
            inliers.emplace_back(indices[idx]);
        }

        // Fit model to num_model_parameters randomly selected points among the
        // inliers.
        plane_model = TriangleMesh::ComputeTrianglePlane(
                points_[inliers[0]], points_[inliers[1]], points_[inliers[2]]);
        if (plane_model.isZero(0)) {
            continue;
        }

        error = 0;
        inliers.clear();
        auto this_result = EvaluateRANSACBasedOnDistance(
                points_, plane_model, inliers, distance_threshold, error);
        if (this_result.fitness_ > result.fitness_ ||
            (this_result.fitness_ == result.fitness_ &&
             this_result.inlier_rmse_ < result.inlier_rmse_)) {
            result = this_result;
            best_plane_model = plane_model;
        }
    }

    // Find the final inliers using best_plane_model.
    inliers.clear();
    for (size_t idx = 0; idx < points_.size(); ++idx) {
        Eigen::Vector4d point(points_[idx](0), points_[idx](1), points_[idx](2),
                              1);
        double distance = std::abs(best_plane_model.dot(point));

        if (distance < distance_threshold) {
            inliers.emplace_back(idx);
        }
    }

    // Improve best_plane_model using the final inliers.
    best_plane_model = GetPlaneFromPoints(points_, inliers);

    utility::LogDebug("RANSAC | Inliers: {:d}, Fitness: {:e}, RMSE: {:e}",
                      inliers.size(), result.fitness_, result.inlier_rmse_);
    return std::make_tuple(best_plane_model, inliers);
}

namespace {
    using namespace geometry;

    int CountValidDepthPixels(const Image &depth, int stride) {
        int num_valid_pixels = 0;
        for (int i = 0; i < depth.height_; i += stride) {
            for (int j = 0; j < depth.width_; j += stride) {
                const float *p = depth.PointerAt<float>(j, i);
                if (*p > 0) num_valid_pixels += 1;
            }
        }
        return num_valid_pixels;
    }

    std::shared_ptr<PointCloud2> CreatePointCloudFromFloatDepthImage(
            const Image &depth,
            const camera::PinholeCameraIntrinsic &intrinsic,
            const Eigen::Matrix4d &extrinsic,
            int stride,
            bool project_valid_depth_only) {
        auto pointcloud = std::make_shared<PointCloud2>();
        Eigen::Matrix4d camera_pose = extrinsic.inverse();
        auto focal_length = intrinsic.GetFocalLength();
        auto principal_point = intrinsic.GetPrincipalPoint();
        int num_valid_pixels;
        if (!project_valid_depth_only) {
            num_valid_pixels =
                    int(depth.height_ / stride) * int(depth.width_ / stride);
        } else {
            num_valid_pixels = CountValidDepthPixels(depth, stride);
        }
        pointcloud->points_.resize(num_valid_pixels);
        int cnt = 0;
        for (int i = 0; i < depth.height_; i += stride) {
            for (int j = 0; j < depth.width_; j += stride) {
                const float *p = depth.PointerAt<float>(j, i);
                if (*p > 0) {
                    auto z = (double)(*p);
                    double x = (j - principal_point.first) * z / focal_length.first;
                    double y =
                            (i - principal_point.second) * z / focal_length.second;
                    Eigen::Vector4d point =
                            camera_pose * Eigen::Vector4d(x, y, z, 1.0);
                    pointcloud->points_[cnt++] = point.block<3, 1>(0, 0);
                } else if (!project_valid_depth_only) {
                    double z = std::numeric_limits<float>::quiet_NaN();
                    double x = std::numeric_limits<float>::quiet_NaN();
                    double y = std::numeric_limits<float>::quiet_NaN();
                    pointcloud->points_[cnt++] = Eigen::Vector3d(x, y, z);
                }
            }
        }
        return pointcloud;
    }

    template <typename TC, int NC>
    std::shared_ptr<PointCloud2> CreatePointCloudFromRGBDImageT(
            const RGBDImage &image,
            const camera::PinholeCameraIntrinsic &intrinsic,
            const Eigen::Matrix4d &extrinsic,
            bool project_valid_depth_only) {
        auto pointcloud = std::make_shared<PointCloud2>();
        Eigen::Matrix4d camera_pose = extrinsic.inverse();
        auto focal_length = intrinsic.GetFocalLength();
        auto principal_point = intrinsic.GetPrincipalPoint();
        double scale = (sizeof(TC) == 1) ? 255.0 : 1.0;
        int num_valid_pixels;
        if (!project_valid_depth_only) {
            num_valid_pixels = image.depth_.height_ * image.depth_.width_;
        } else {
            num_valid_pixels = CountValidDepthPixels(image.depth_, 1);
        }
        pointcloud->points_.resize(num_valid_pixels);
        pointcloud->colors_.resize(num_valid_pixels);
        int cnt = 0;
        for (int i = 0; i < image.depth_.height_; i++) {
            auto *p = (float *)(image.depth_.data_.data() +
                                i * image.depth_.BytesPerLine());
            TC *pc = (TC *)(image.color_.data_.data() +
                            i * image.color_.BytesPerLine());
            for (int j = 0; j < image.depth_.width_; j++, p++, pc += NC) {
                if (*p > 0) {
                    auto z = (double)(*p);
                    double x = (j - principal_point.first) * z / focal_length.first;
                    double y =
                            (i - principal_point.second) * z / focal_length.second;
                    Eigen::Vector4d point =
                            camera_pose * Eigen::Vector4d(x, y, z, 1.0);
                    pointcloud->points_[cnt] = point.block<3, 1>(0, 0);
                    pointcloud->colors_[cnt++] =
                            Eigen::Vector3d(pc[0], pc[(NC - 1) / 2], pc[NC - 1]) /
                            scale;
                } else if (!project_valid_depth_only) {
                    double z = std::numeric_limits<float>::quiet_NaN();
                    double x = std::numeric_limits<float>::quiet_NaN();
                    double y = std::numeric_limits<float>::quiet_NaN();
                    pointcloud->points_[cnt] = Eigen::Vector3d(x, y, z);
                    pointcloud->colors_[cnt++] =
                            Eigen::Vector3d(std::numeric_limits<TC>::quiet_NaN(),
                                            std::numeric_limits<TC>::quiet_NaN(),
                                            std::numeric_limits<TC>::quiet_NaN());
                }
            }
        }
        return pointcloud;
    }
} // unnamed namespace

std::shared_ptr<PointCloud2> PointCloud2::CreateFromDepthImage(
        const Image &depth,
        const camera::PinholeCameraIntrinsic &intrinsic,
        const Eigen::Matrix4d &extrinsic, double depth_scale,
        double depth_trunc, int stride,
        bool project_valid_depth_only) {

    if (depth.num_of_channels_ == 1) {
        if (depth.bytes_per_channel_ == 2) {
            auto float_depth =
                    depth.ConvertDepthToFloatImage(depth_scale, depth_trunc);
            return CreatePointCloudFromFloatDepthImage(
                    *float_depth, intrinsic, extrinsic, stride,
                    project_valid_depth_only);
        } else if (depth.bytes_per_channel_ == 4) {
            return CreatePointCloudFromFloatDepthImage(
                    depth, intrinsic, extrinsic, stride,
                    project_valid_depth_only);
        }
    }
    utility::LogError(
            "[CreatePointCloudFromDepthImage] Unsupported image format.");
    return std::make_shared<PointCloud2>();
}

std::shared_ptr<PointCloud2> PointCloud2::CreateFromRGBDImage(
        const RGBDImage &image,
        const camera::PinholeCameraIntrinsic &intrinsic,
        const Eigen::Matrix4d &extrinsic,
        bool project_valid_depth_only) {
    if (image.depth_.num_of_channels_ == 1 &&
        image.depth_.bytes_per_channel_ == 4) {
        if (image.color_.bytes_per_channel_ == 1 &&
            image.color_.num_of_channels_ == 3) {
            return CreatePointCloudFromRGBDImageT<uint8_t, 3>(
                    image, intrinsic, extrinsic, project_valid_depth_only);
        } else if (image.color_.bytes_per_channel_ == 1 &&
                   image.color_.num_of_channels_ == 4) {
            return CreatePointCloudFromRGBDImageT<uint8_t, 4>(
                    image, intrinsic, extrinsic, project_valid_depth_only);
        } else if (image.color_.bytes_per_channel_ == 4 &&
                   image.color_.num_of_channels_ == 1) {
            return CreatePointCloudFromRGBDImageT<float, 1>(
                    image, intrinsic, extrinsic, project_valid_depth_only);
        }
    }
    utility::LogError(
            "[CreatePointCloudFromRGBDImage] Unsupported image format.");
    return std::make_shared<PointCloud2>();
}

std::shared_ptr<PointCloud2> PointCloud2::CreateFromVoxelGrid(const VoxelGrid &voxel_grid) {
    auto output = std::make_shared<PointCloud2>();
    output->points_.resize(voxel_grid.voxels_.size());
    bool has_colors = voxel_grid.HasColors();
    if (has_colors) {
        output->colors_.resize(voxel_grid.voxels_.size());
    }
    size_t vidx = 0;
    for (auto &it : voxel_grid.voxels_) {
        const geometry::Voxel voxel = it.second;
        output->points_[vidx] =
                voxel_grid.GetVoxelCenterCoordinate(voxel.grid_index_);
        if (has_colors) {
            output->colors_[vidx] = voxel.color_;
        }
        vidx++;
    }
    return output;
}

} // namespace geometry
} // namespace open3d


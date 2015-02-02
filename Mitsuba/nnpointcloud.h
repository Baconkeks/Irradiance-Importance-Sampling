#pragma once

#include <stdlib.h>
#include <mitsuba/nanoflann.hpp>
MTS_NAMESPACE_BEGIN


template <class T, uint32_t N>
struct PointCloud
{
    std::vector<Vector3> points;

    inline std::size_t kdtree_get_point_count() const { return points.size(); }

    inline float kdtree_distance(const float *points_, const std::size_t idx_point2, size_t /*size*/) const
    {
        T d = T(0);
        for (uint32_t i = 0; i < N; ++i) {
            const T temp = points_[i] - points[idx_point2][i];
            d += temp * temp;
        }
        return d;
    }

    inline float kdtree_get_pt(const std::size_t idx, int dim) const { return points[idx][uint32_t(dim)]; }

    template <class BBOX>
	bool kdtree_get_bbox(BBOX const& /*bb*/) const { return false; }
};


template <class T, uint32_t N>
class PointCloudMap
{
    typedef nanoflann::L2_Simple_Adaptor<T, PointCloud<T, N>> Adaptor;
    typedef nanoflann::KDTreeSingleIndexAdaptor<Adaptor, PointCloud<T, N>, N> Kdtree;
public:
    PointCloudMap(PointCloud<T, N> const& pointCloud) :
        m_kdtree(3, pointCloud, nanoflann::KDTreeSingleIndexAdaptorParams(10))
    {
        m_kdtree.buildIndex();
    }

    /*
     * Returns the indices and squared distances of
     * the neighbours within the given radius
     */
    void query_radius(float radius,
                      Vector3f const& query_point,
                      std::vector<std::pair<std::size_t, float>>& indices_distances,
                      bool sorted = true)
    const {
        indices_distances.reserve(10);

        nanoflann::SearchParams params;
        params.sorted = sorted;

        Vector3f query = query_point;
        m_kdtree.radiusSearch(&query[0], radius * radius + 1e-6f, indices_distances, params);

        for (uint32_t i = 0; i < indices_distances.size(); ++i)
        {
            indices_distances[i].second = sqrtf(indices_distances[i].second);
        }
    }

    /*
     * Returns the indices and squared distances of
     * the K nearest neighbours
     */
    void query_knn(uint32_t K,
                   Vector3 const& query_point,
                   std::vector<std::pair<std::size_t, T>>& indices_distances)
    const {
        indices_distances.reserve(K);
        std::vector<std::size_t> indices(K);
        std::vector<T>  distances(K);

        Vector3f query = query_point;
        m_kdtree.knnSearch(&query[0], K, indices.data(), distances.data());

        for (uint32_t i = 0; i < K; ++i)
        {
            indices_distances.push_back(std::make_pair(indices[i], sqrtf(distances[i])));
        }
    }

private:
    Kdtree m_kdtree;
};

MTS_NAMESPACE_END
#ifndef _SmartStream_MUTABLE_POINTCLOUD_HPP_
#define _SmartStream_MUTABLE_POINTCLOUD_HPP_

#include <mutex>

#include <FW/FWVisualizer.h>
#include <FW/FWFactory.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace FW {

class SmartStream;

class mutable_pointcloud {
    public:
        typedef std::shared_ptr<mutable_pointcloud>       ptr_t;
        typedef std::weak_ptr<mutable_pointcloud>         wptr_t;
        typedef std::shared_ptr<const mutable_pointcloud> const_ptr_t;
        typedef std::weak_ptr<const mutable_pointcloud>   const_wptr_t;

        typedef Eigen::Vector2i                 vec2i_t;
        typedef Eigen::Vector2f                 vec2f_t;
        typedef Eigen::Vector3f                 vec3f_t;
        typedef Eigen::Matrix3f                 base_t;
        typedef Eigen::AlignedBox<float, 2>     bbox2f_t;
        typedef Eigen::AlignedBox<float, 3>     bbox3f_t;
        typedef pcl::PointNormal                point_normal_t;
        typedef pcl::PointCloud<point_normal_t> cloud_normal_t;
        typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> vertex_data_t;

        typedef harmont::pointcloud_object<cloud_normal_t, boost::shared_ptr> renderable_t;

    public:
        mutable_pointcloud();

        virtual ~mutable_pointcloud();

        mutable_pointcloud::renderable_t::ptr_t init(uint32_t num_points);

        void finish();

        void add_data(cloud_normal_t::ConstPtr data);

        void damage();

        void render_poll();

        template <typename F>
        void with_vertex_data(F&& func) const;

    protected:
        void process_data();

    protected:
        // cloud management
        std::mutex cloud_mutex_;
        renderable_t::ptr_t cloud_;
        // data management
        vertex_data_t vertex_data_;
        std::mutex data_mutex_;
        std::deque<cloud_normal_t::ConstPtr> data_queue_;
        std::shared_ptr<std::thread> data_thread_;

        bbox3f_t bbox_;
        uint32_t last_point_idx_;
        uint32_t range_start_;
        bool damaged_;
        bool finished_;
};


} // FW

#include "mutable_pointcloud.ipp"

#endif /* _SmartStream_MUTABLE_POINTCLOUD_HPP_ */

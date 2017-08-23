#ifndef _SmartClient_VIS_CLIENT_HPP_
#define _SmartClient_VIS_CLIENT_HPP_

#include <mutex>

#define USE_PCL_COMPRESS
#include <pcl_compress_stream/smart_client.hpp>

#include "mutable_pointcloud.hpp"

#include <roomarr/RoomArrangement.hpp>
typedef RoomArr::RoomArrangement::Arr Arr;

#include "types.hpp"

class vis_client : public pcl_compress::smart_client {
public:
    typedef std::shared_ptr<vis_client> ptr_t;
    typedef std::weak_ptr<vis_client> wptr_t;
    typedef std::shared_ptr<const vis_client> const_ptr_t;
    typedef std::weak_ptr<const vis_client> const_wptr_t;

    typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> vertex_data_t;

public:
    vis_client(std::string host, short port, uint32_t min_patch_count = 500);

    virtual ~vis_client();

    RoomArr::RoomArrangement& arrangement();

    const RoomArr::RoomArrangement& arrangement() const;

    pcl_compress::merged_global_data_t& global_data();

    FW::mutable_pointcloud::renderable_t::ptr_t renderable(uint32_t patch_index) const;

    harmont::renderable_group::ptr_t renderable_group() const;

    const pcl_compress::merged_global_data_t& global_data() const;

    void request(const FW::request_t& req, FW::SmartStream* vis);

    const std::vector<Eigen::Vector3f>& mesh() const;

    //void init_cloud(harmont::renderable::ptr_t& cloud, FW::SmartStream* vis) {
        //if (cloud || !patch_count_) return;
        //cloud = std::make_shared<harmont::pointcloud_object<cloud_normal_t, boost::shared_ptr>>(global_data_.num_points);
        //cloud->init();
        //vis->addObject("cloud", cloud);
    //}
    //
    void render_poll();

    bool idle() const;

    //void update_cloud(harmont::renderable::ptr_t& cloud, FW::SmartStream* vis) {
        //if (data_mutex_.try_lock()) {
            //if (update_cloud_) {
                //auto display_map = cloud->eigen_map_display_buffer();
                //auto shadow_map = cloud->eigen_map_shadow_buffer();
                //uint32_t count = last_point_idx_ - range_start_;
                //display_map.block(range_start_, 0, count, 9) = vertex_data_.block(range_start_, 0, count, 9);
                //shadow_map.block(range_start_, 0, count, 3) = vertex_data_.block(range_start_, 0, count, 3);
                //range_start_ = last_point_idx_;
                //cloud->unmap_display_buffer();
                //cloud->unmap_shadow_buffer();
                ////cloud->update_geometry(vertex_data_);
                //cloud->set_bounding_box(bbox_);
                //update_cloud_ = false;
            //}
            //data_mutex_.unlock();
        //}
    //}

protected:
    // asio thread
    void on_global_data_ready(const pcl_compress::merged_global_data_t& g_data, std::istream& arr_data, std::vector<Eigen::Vector3f>& mesh);

    // asio thread
    void on_patch_ready(uint32_t global_index, uint32_t request_index, pcl_compress::cloud_normal_t::Ptr patch);

    void on_cloud_finished();

protected:
    uint32_t min_patch_count_;
    uint32_t last_patch_count_;
    uint32_t patch_count_;
    pcl_compress::merged_global_data_t global_data_;
    RoomArr::RoomArrangement::UPtr arrangement_;
    FW::mutable_pointcloud::ptr_t current_room_;
    bool finished_room_;
    std::future<void> current_thread_;
    uint32_t requested_patch_count_;
    std::set<uint32_t> requested_sets_;
    std::set<uint32_t> requested_sets_hash_;
    //std::mutex data_mutex_;
    //std::mutex cloud_mutex_;
    //vertex_data_t vertex_data_;
    //bbox3f_t bbox_;
    bool idle_;
    uint32_t room_idx_;
    std::map<uint32_t, FW::mutable_pointcloud::renderable_t::ptr_t> renderables_;
    harmont::renderable_group::ptr_t renderable_group_;
    std::vector<Eigen::Vector3f> mesh_;
};

#endif /* _SmartClient_VIS_CLIENT_HPP_ */

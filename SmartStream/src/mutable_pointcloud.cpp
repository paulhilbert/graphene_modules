#include <mutable_pointcloud.hpp>

namespace FW {

mutable_pointcloud::mutable_pointcloud() : last_point_idx_(0), range_start_(0), damaged_(true), finished_(false) {
}

mutable_pointcloud::~mutable_pointcloud() {
}

mutable_pointcloud::renderable_t::ptr_t mutable_pointcloud::init(uint32_t num_points) {
    std::lock_guard<std::mutex> g(cloud_mutex_);
    vertex_data_ = harmont::renderable::vertex_data_t::Zero(num_points, 9);
    float rgba = harmont::renderable::color_to_rgba(Eigen::Vector4f::Ones());
    vertex_data_.col(3) = Eigen::VectorXf::Constant(num_points, rgba);
    cloud_ = std::make_shared<harmont::pointcloud_object<cloud_normal_t, boost::shared_ptr>>(num_points);
    cloud_->init();

    // start data copy thread
    data_thread_ = std::make_shared<std::thread>(std::bind(&mutable_pointcloud::process_data, this));

    return cloud_;
}

void mutable_pointcloud::finish() {
    data_mutex_.lock();
    finished_ = true;
    data_mutex_.unlock();
    data_thread_->join();
}

void mutable_pointcloud::add_data(cloud_normal_t::ConstPtr data) {
    std::lock_guard<std::mutex> g(data_mutex_);
    data_queue_.push_back(data);
    //for (const auto& point : *data) {
        //vec3f_t pos = point.getVector3fMap();// - global_data_.scan_origin;
        //vec3f_t nrm = point.getNormalVector3fMap();
        //bbox_.extend(pos);
        //vertex_data_.block(last_point_idx_, 0, 1, 3) = pos.transpose();
        //vertex_data_.block(last_point_idx_, 4, 1, 3) = nrm.transpose();
        //vertex_data_.block(last_point_idx_, 7, 1, 2) = Eigen::RowVector2f::Zero();
        //vertex_data_(last_point_idx_, 3) = harmont::renderable::color_to_rgba(Eigen::Vector4f::Ones());
        //++last_point_idx_;
    //}
}

void mutable_pointcloud::damage() {
    std::lock_guard<std::mutex> g(cloud_mutex_);
    damaged_ = true;
}

void mutable_pointcloud::render_poll() {
    if (cloud_mutex_.try_lock()) {
        if (damaged_) {
            auto display_map = cloud_->eigen_map_display_buffer();
            auto shadow_map = cloud_->eigen_map_shadow_buffer();
            uint32_t count = last_point_idx_ - range_start_;
            data_mutex_.lock();
            display_map.block(range_start_, 0, count, 9) = vertex_data_.block(range_start_, 0, count, 9);
            shadow_map.block(range_start_, 0, count, 3) = vertex_data_.block(range_start_, 0, count, 3);
            data_mutex_.unlock();
            range_start_ = last_point_idx_;
            cloud_->unmap_display_buffer();
            cloud_->unmap_shadow_buffer();
            //cloud->update_geometry(vertex_data_);
            cloud_->set_bounding_box(bbox_);
        }
        cloud_mutex_.unlock();
    }
}

void mutable_pointcloud::process_data() {
    while (true) {
        if (!data_mutex_.try_lock()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        if (finished_) return;

        if (data_queue_.size() > 100) {
            while (!data_queue_.empty()) {
                cloud_normal_t::ConstPtr cloud = data_queue_.front();
                data_queue_.pop_front();
                for (const auto& point : *cloud) {
                    vec3f_t pos = point.getVector3fMap();// - global_data_.scan_origin;
                    vec3f_t nrm = point.getNormalVector3fMap();
                    bbox_.extend(pos);
                    vertex_data_.block(last_point_idx_, 0, 1, 3) = pos.transpose();
                    vertex_data_.block(last_point_idx_, 4, 1, 3) = nrm.transpose();
                    vertex_data_.block(last_point_idx_, 7, 1, 2) = Eigen::RowVector2f::Zero();
                    vertex_data_(last_point_idx_, 3) = harmont::renderable::color_to_rgba(Eigen::Vector4f::Ones());
                    ++last_point_idx_;
                }
            }
        } else {
            data_mutex_.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }
        data_mutex_.unlock();
    }
}

} // FW

#include <vis_client.hpp>

#include "SmartStream.h"

using namespace pcl_compress;

vis_client::vis_client(std::string host, short port, uint32_t min_patch_count)
    : smart_client(host, port),
      min_patch_count_(min_patch_count),
      last_patch_count_(0),
      patch_count_(0),
      idle_(true),
      room_idx_(0),
      renderable_group_(new harmont::renderable_group(std::vector<harmont::renderable::ptr_t>())) {}

vis_client::~vis_client() {
}

RoomArr::RoomArrangement& vis_client::arrangement() {
    return *arrangement_;
}

const RoomArr::RoomArrangement& vis_client::arrangement() const {
    return *arrangement_;
}

merged_global_data_t& vis_client::global_data() {
    return global_data_;
}

const merged_global_data_t& vis_client::global_data() const {
    return global_data_;
}

FW::mutable_pointcloud::renderable_t::ptr_t vis_client::renderable(uint32_t patch_index) const {
    auto find_it = renderables_.find(patch_index);
    if (find_it == renderables_.end()) return nullptr;
    return find_it->second;
}

harmont::renderable_group::ptr_t vis_client::renderable_group() const {
    return renderable_group_;
}

void vis_client::request(const FW::request_t& req, FW::SmartStream* vis) {
    if (requested_sets_hash_.find(std::get<1>(req)) != requested_sets_hash_.end()) {
        return;
    }
    std::vector<uint32_t> patch_indices = std::get<3>(req);
    if (patch_indices.empty()) return;

    requested_patch_count_ = patch_indices.size();

    // determine point count
    uint32_t point_count = 0; //patch_indices.size() * 32 *32;
    for (const auto& patch_idx : patch_indices) {
        point_count += global_data_.point_counts[patch_idx];
    }
    if (!point_count) return;
    std::cout << "requesting " << point_count << " points in " << patch_indices.size() << " patches" << "\n";

    // init mutable pointcloud
    finished_room_ = false;
    auto current_room = std::make_shared<FW::mutable_pointcloud>();
    auto cloud = current_room->init(point_count, std::get<0>(req));
    for (const auto& idx : patch_indices) {
        renderables_[idx] = cloud;
    }
    vis->addObject("room_" + std::to_string(room_idx_++), cloud);
    if (!renderable_group_) renderable_group_ = std::make_shared<harmont::renderable_group>(std::vector<harmont::renderable::ptr_t>());
    renderable_group_->add_object(cloud);
    current_room_ = current_room;

    //receive_patch_scan_data(patch_indices, global_data_);
    idle_ = false;
    requested_sets_hash_.insert(std::get<1>(req));
    current_thread_ = std::async(std::launch::async, [this, patch_indices] () {
        receive_patch_scan_data(patch_indices, global_data_);
    });
    //current_thread_.get();

    // request patches
}

const std::vector<Eigen::Vector3f>& vis_client::mesh() const {
    return mesh_;
}

void vis_client::render_poll() {
    if (current_room_) {
        current_room_->render_poll();
        if (finished_room_) {
            current_room_->finish();
            current_room_.reset();
            idle_ = true;
        }
    }
}

bool vis_client::idle() const {
    return idle_;
}

// asio thread
void vis_client::on_global_data_ready(const merged_global_data_t& g_data, std::istream& arr_data, std::vector<Eigen::Vector3f>& mesh) {
    global_data_ = g_data;

    // load arrangement data
    arrangement_ = std::make_unique<RoomArr::RoomArrangement>();
    arrangement_->importArrangement(arr_data);

    mesh_ = mesh;
}

// asio thread
void vis_client::on_patch_ready(uint32_t global_index, uint32_t request_index, cloud_normal_t::Ptr patch) {
    current_room_->add_data(patch);
    ++patch_count_;
    if (patch_count_ - last_patch_count_ >= min_patch_count_) {
        current_room_->damage();
        last_patch_count_ = patch_count_;
        current_room_->damage();
    }

    if (request_index == requested_patch_count_ - 1) {
        on_cloud_finished();
    }
}

void vis_client::on_cloud_finished() {
    current_room_->damage();
    finished_room_ = true;
}

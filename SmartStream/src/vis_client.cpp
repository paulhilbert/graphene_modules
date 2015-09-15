#include <vis_client.hpp>

#include "SmartStream.h"

using namespace pcl_compress;

vis_client::vis_client(std::string host, short port, uint32_t min_patch_count)
    : smart_client(host, port),
      min_patch_count_(min_patch_count),
      last_patch_count_(0),
      patch_count_(0),
      idle_(true) /*,
      update_cloud_(false)*/ {}

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

void vis_client::request_room(int room_idx, FW::SmartStream* vis) {
    idle_ = false;
    // determine patches
    Arr::Face_handle f = face_map_[room_idx];

    //std::set<uint32_t> wall_patches;
    //auto curCirc = f->outer_ccb();
    //auto endCirc = curCirc;
    //do {
        //wall_patches.insert(curCirc->data()->patchIndices.begin(), curCirc->data()->patchIndices.end());

        //++curCirc;
    //} while (curCirc != endCirc);
    //std::vector<uint32_t> patch_indices(wall_patches.begin(), wall_patches.end());

    std::vector<uint32_t> patch_indices(f->data()->patchIndices.begin(), f->data()->patchIndices.end());

    requested_patch_count_ = patch_indices.size();

    // determine point count
    uint32_t point_count = 0; //patch_indices.size() * 32 *32;
    for (const auto& patch_idx : patch_indices) {
        point_count += global_data_.point_counts[patch_idx];
    }
    std::cout << "requesting " << point_count << " points in " << patch_indices.size() << " patches" << "\n";

    // init mutable pointcloud
    finished_room_ = false;
    auto current_room = std::make_shared<FW::mutable_pointcloud>();
    auto cloud = current_room->init(point_count);
    vis->addObject("room_" + std::to_string(room_idx), cloud);
    current_room_ = current_room;

    //receive_patch_scan_data(patch_indices, global_data_);
    current_thread_ = std::async(std::launch::async, [this, patch_indices] () {
        receive_patch_scan_data(patch_indices, global_data_);
    });
    //current_thread_.get();

    // request patches
}

void vis_client::render_poll() {
    if (current_room_) {
        current_room_->render_poll();
        if (finished_room_) {
            current_room_.reset();
            std::cout << "finished room" << "\n";
            idle_ = true;
        }
    }
}

bool vis_client::idle() const {
    return idle_;
}

// asio thread
void vis_client::on_global_data_ready(const merged_global_data_t& g_data, std::istream& arr_data) {
    global_data_ = g_data;

    // load arrangement data
    arrangement_ = std::make_unique<RoomArr::RoomArrangement>();
    arrangement_->importArrangement(arr_data);
    uint32_t face_count = 0;
    for (auto fIt = arrangement_->getArrangement().faces_begin();
         fIt != arrangement_->getArrangement().faces_end(); ++fIt) {
        if (fIt->data()->index >= 0) ++face_count;
    }
    face_map_.resize(face_count);
    for (auto fIt = arrangement_->getArrangement().faces_begin();
         fIt != arrangement_->getArrangement().faces_end(); ++fIt) {
        int index = fIt->data()->index;
        if (index >= 0) face_map_[index] = fIt;
    }
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

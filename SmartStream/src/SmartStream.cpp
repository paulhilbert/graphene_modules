#include "SmartStream.h"

#include <mutex>

#include <Library/Colors/Generation.h>
using namespace GUI::Property;

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


//#define USE_CEREAL
//#include <pcl_compress_client/client.hpp>
//using namespace pcl_compress;

//#include <boost/bind.hpp>

typedef pcl::PointNormal point_t;
typedef pcl::PointCloud<point_t> cloud_t;

using namespace pcl_compress;


namespace FW {


SmartStream::SmartStream(std::string id, const std::string& host, const std::string& port) : Visualizer(id), host_(host), port_(port), crt_face_(-1), tracking_(false) {
}

SmartStream::~SmartStream() {
    //if (current_thread_.valid()) current_thread_.wait();
    //client_->close();
}

void SmartStream::init() {
    short port = std::atoi(port_.c_str());
    client_ = std::make_shared<vis_client>(host_, port);
    client_->receive_global_scan_data();

	addProperties();
	registerEvents();

    renderArrangement();

    // render crosshair
    auto crosshair = std::make_shared<harmont::crosshair_object>(Eigen::Vector3f::Zero(), Eigen::Vector4f(1.f, 0.f, 0.f, 1.f), 0.5f, 0.5f);
    crosshair->init();
    addObject("crosshair", crosshair);
}

void SmartStream::addProperties() {
    // file select
    //auto file_tree = gui()->properties()->add<Tree>("Available Files", "scan_files");
    //for (const auto& entry : file_list_) {
        //file_tree->add(std::to_string(entry.index), {entry.name}, false);
        //current_file_selection_[entry.index] = false;
    //}
    //current_file_selection_changed_ = false;
    //file_tree->setCallback([&] (std::string id_str, bool selected) {
        //uint32_t idx;
        //std::stringstream sstr; sstr << id_str; sstr >> idx;
        //bool old_value = current_file_selection_[idx];
        //current_file_selection_[idx] = selected;
        //if (selected != old_value) {
            //gui()->properties()->get<Button>({"get_files"})->enable();
        //}
        //std::cout << "switched " << idx << " to " << std::boolalpha << selected << "\n";
    //});

    //auto get_files = gui()->properties()->add<Button>("Get Files", "get_files");
    //get_files->disable();
    //get_files->setCallback([&] () {
        //auto self = gui()->properties()->get<Button>({"get_files"});
        //auto list = gui()->properties()->get<Tree>({"scan_files"});
        //std::cout << "getting files" << "\n";
        //stream_thread_ = std::make_shared<std::thread>([this] () {
            //for (const auto& v : current_file_selection_) {
                //if (!v.second) continue;
                //global_data_t gdata;
                //client_->receive_global_scan_data(v.first, gdata);
                //std::vector<uint32_t> all_patches(gdata.num_patches);
                //std::iota(all_patches.begin(), all_patches.end(), 0);
                //client_->receive_patch_scan_data(v.first, all_patches, gdata);
            //}

            //auto button = gui()->properties()->get<Button>({"get_files"});
            //auto files = gui()->properties()->get<Tree>({"scan_files"});
            //files->enable();
            //button->enable();
        //});
        //self->disable();
        //list->disable();
        ////stream_thread_->join();
    //});

    auto track_btn = gui()->properties()->add<ToggleButton>("Track Camera", "track");
    track_btn->setValue(false);
    track_btn->setCallback([&] (bool state) { tracking_ = state; });


    auto  showGroup = gui()->modes()->addGroup("showGroup");
    showGroup->addOption("showClip", "Enable Clipping", std::string(ICON_PREFIX) + "clipping.png");

    showGroup->setCallback([&] (std::string option, bool state) {
        if (option == "showClip") {
            auto objs = objects();
            for (auto& obj : objs) {
                const char test[] = "room";
                if (obj.first.length() >= 4 && strncmp(obj.first.c_str(), test, 4) == 0) {
                    obj.second->set_clipping(state);
                }
            }
        }
    });

    auto  transformGroup = gui()->modes()->addGroup("TransformGroup");
    transformGroup->addOption("Clip", "Modify Clipping Plane", std::string(ICON_PREFIX) + "clipplane.png");
}

void SmartStream::registerEvents() {
    fw()->events()->connect<void (int, int, int, int)>("LEFT_DRAG", [&] (int dx, int dy, int, int) {
        bool clipping = gui()->modes()->group("TransformGroup")->option("Clip")->active();

        if (clipping) {
            auto objs = objects();
            for (auto& obj : objs) {
                const char test[] = "room";
                if (obj.first.length() >= 4 && strncmp(obj.first.c_str(), test, 4) == 0) {
                    obj.second->delta_clipping_height(-dy * 0.01f);
                }
            }
        }
    });

    fw()->events()->connect<void(int, int)>("LEFT_CLICK", [&](int x, int y) {
//      if (!client_->idle()) return;
        Eigen::Vector3f pointerPos = fw()->camera()->position();//Eigen::Vector3f::Zero();
        //Eigen::Vector3f pickRayOrigin = Eigen::Vector3f::Zero();
        //Eigen::Vector3f pickRayDirection = Eigen::Vector3f::Zero();

        //std::tie(pickRayOrigin, pickRayDirection) =
            //fw()->camera()->pick_ray(x, y);
        //float pickRayLambda = -pickRayOrigin[2] / pickRayDirection[2];
        //pointerPos = pickRayOrigin + pickRayLambda * pickRayDirection;

        //for (auto faceLines : lines_) {
            //faceLines->set_active(false);
        //}

        auto pickedFace = client_->arrangement().getFaceAtPoint(pointerPos.head(2));
        if (!pickedFace || pickedFace.get()->data()->index < 0) {
            std::cout << "No inside face picked." << std::endl;
            return;
        }

        if (pickedFace.get()->data()->index == crt_face_) return;


        //std::set<int> set_new({pickedFace.get()->data()->index}), set_old;
        //update(set_new, set_old);

        std::vector<int> new_scans, old_scans;
        computeNewSets(new_scans, old_scans, *pickedFace);
        update(new_scans, old_scans);
        crt_face_ = pickedFace.get()->data()->index;
        //for (const auto& scan_idx : old_scans) {
            //tryRemoveObject("room_" + std::to_string(scan_idx));
        //}
        //for (const auto& scan_idx : new_scans) {
            //cloud_t::Ptr crt_cloud(new cloud_t());

            //Arr::Face_handle f = face_map_[scan_idx];
            //for (const auto& patch_idx : f->data()->patchIndices) {
                //point_t point;
                //// compute center
                //point.getVector3fMap() = gdata_.origins[patch_idx];

                //// compute normal
                //point.getNormalVector3fMap() =
                    //gdata_.bases[patch_idx].col(2).normalized();
                //crt_cloud->push_back(point);
            //}

            //auto scan = std::make_shared<
                //harmont::pointcloud_object<cloud_t, boost::shared_ptr>>(crt_cloud);
            //scan->init();
            //scan->set_point_colors(Colors::Generation::randomHueRGBA());
            //addObject("room_" + std::to_string(scan_idx), scan);
        //}
        //for (const auto& f : new_scans) {
            //lines_[f]->set_active(true);
        //}

        // std::cout << "Face picked" << std::endl;
        // int faceIndex = pickedFace.get()->data()->index;
        // std::cout << faceIndex << std::endl;

        // if (faceIndex >= 0) {
        // lines_[faceIndex]->set_active(true);

        // std::cout << "Num patch indices: " <<
        // pickedFace.get()->data()->patchIndices.size() << std::endl;

        // auto oneRingFaces = arrangement_->getOneRingFaces(pickedFace.get());
        // std::cout << "Num one ring faces: " << oneRingFaces.size() <<
        // std::endl;
        // for (auto f : oneRingFaces) {
        // lines_[f->data()->index]->set_active(true);
        //}
        //}
    });
}

void SmartStream::renderArrangement() {
    std::vector<Eigen::Vector3f> allLineVertices;

    RoomArr::RoomArrangement& arrangement = client_->arrangement();

    for (auto fIt = arrangement.getArrangement().faces_begin();
         fIt != arrangement.getArrangement().faces_end(); ++fIt) {
        if (fIt->is_unbounded()) continue;

        if (!fIt->data()->patchIndices.size()) {
            continue;
        }

        std::vector<Eigen::Vector3f> lineVertices;

        auto curCirc = fIt->outer_ccb();
        auto endCirc = curCirc;
        do {
            float z = fIt->data()->floorHeight;
            Eigen::Vector3f s = arrangement.getNodePos3(curCirc->source(), z);
            Eigen::Vector3f t = arrangement.getNodePos3(curCirc->target(), z);

            lineVertices.push_back(s);
            lineVertices.push_back(t);

            allLineVertices.push_back(s - Eigen::Vector3f(0, 0, 0.01));
            allLineVertices.push_back(t - Eigen::Vector3f(0, 0, 0.01));

            ++curCirc;
        } while (curCirc != endCirc);

        //lines_.push_back(std::make_shared<harmont::lines_object>(
            //lineVertices, Eigen::Vector4f(0, 0, 0, 1)));
        //lines_.back()->init();
        //lines_.back()->set_active(false);
    }

    auto allLines = std::make_shared<harmont::lines_object>(
        allLineVertices, Eigen::Vector4f(0.8, 0.8, 0.8, 1));
    allLines->init();
    addObject("AllArrangementLines", allLines);

    //int i = 0;
    //for (auto faceLines : lines_) {
        //addObject("ArrangementLines_" + std::to_string(i++), faceLines);
    //}
}

void
SmartStream::computeNewSets(std::vector<int>& new_scans,
                               std::vector<int>& old_scans,
                               //std::set<int>& query_patches,
                               const Arr::Face_handle& current_face) {
    std::set<int> new_scans_, old_scans_;
    int current_index = current_face->data()->index;
    if (current_index >= 0) {
        std::set<int> current_scans;
        current_scans.insert(current_index);
        auto ring = client_->arrangement().getOneRingFaces(current_face);
        std::cout << "Num one ring faces: " << ring.size() << std::endl;
        for (auto f : ring) {
            current_scans.insert(f->data()->index);
        }

        std::set_difference(current_scans.begin(), current_scans.end(),
                            current_scans_.begin(), current_scans_.end(),
                            std::inserter(new_scans_, new_scans_.end()));
        std::set_difference(current_scans_.begin(), current_scans_.end(),
                            current_scans.begin(), current_scans.end(),
                            std::inserter(old_scans_, old_scans_.end()));

        new_scans.clear();
        old_scans.clear();
        if (new_scans_.find(current_index) != new_scans_.end()) {
            new_scans.push_back(current_index);
        }
        for (int idx : new_scans_) {
            if (idx != current_index) new_scans.push_back(idx);
        }
        for (int idx : old_scans_) {
            old_scans.push_back(idx);
        }

        //query_patches.clear();
        //for (const auto& idx : new_scans) {
            //Arr::Face_handle f = face_map_[idx];
            //query_patches.insert(f->data()->patchIndices.begin(),
                                 //f->data()->patchIndices.end());
        //}

        current_scans_ = current_scans;
    }
}

void SmartStream::update(const std::vector<int>& new_rooms, const std::vector<int>& old_rooms) {
    //for (const auto& idx : old_rooms) {
        //tryRemoveObject("room_" + std::to_string(idx));
    //}
    std::lock_guard<std::mutex> lg(request_queue_mutex_);
    for (const auto& idx : new_rooms) {
        bool has_object = false;
        try {
            object("room_"+std::to_string(idx));
            has_object = true;
        } catch (...) {
        }
        if (!has_object) {
            request_queue_.push_back(idx);
        }
        //client_->request_room(idx, this);
    }
}

void SmartStream::preRender() {


    //if (current_thread_.valid()) {
        client_->render_poll();
        if (!client_->idle()) gui()->status()->set("Streaming...");
        else gui()->status()->set("Idle.");
        {
            std::lock_guard<std::mutex> lg(request_queue_mutex_);
            if (client_->idle() && !request_queue_.empty()) {
                int next_idx = request_queue_.front();
                request_queue_.pop_front();
                client_->request_room(next_idx, this);
            }
        }
        //std::future_status status = current_thread_.wait_for(std::chrono::milliseconds(1));
        //if (status == std::future_status::ready) {
            //if (current_thread_.valid()) current_thread_.get();
        //}
    //}
    //if (!renderable_) client_->init_cloud(renderable_, this);
    //if (renderable_) {
        //if (!display_map_) {
            //typedef harmont::renderable::map_t map_t;
            //display_map_ = std::make_shared<map_t>(renderable_->eigen_map_display_buffer());
            //shadow_map_ = std::make_shared<map_t>(renderable_->eigen_map_shadow_buffer());
            //*display_map_ = renderable_->eigen_map_display_buffer();
            //*shadow_map_ = renderable_->eigen_map_shadow_buffer();
        //}
        //client_->update_cloud(renderable_, this);
    //}

    // crosshair positioning
    vec3f_t pos = fw()->camera()->position();
    auto pickedFace = client_->arrangement().getFaceAtPoint(pos.head(2));
    if (!pickedFace || pickedFace.get()->data()->index < 0) {
        pos[2] = 0.f;
    } else {
        pos[2] = pickedFace.get()->data()->floorHeight;
    }
    Eigen::Matrix4f tr = Eigen::Matrix4f::Identity();
    tr.block<3,1>(0, 3) = pos;
    auto crosshair = object("crosshair");
    crosshair->set_transformation(tr);

    if (tracking_) {
        if (pickedFace.get()->data()->index == crt_face_) return;

        std::vector<int> new_scans, old_scans;
        computeNewSets(new_scans, old_scans, *pickedFace);
        update(new_scans, old_scans);

        crt_face_ = pickedFace.get()->data()->index;
    }

}

SmartStream::Factory::Factory() : FW::Factory() {
}

SmartStream::Factory::~Factory() {
}

void SmartStream::Factory::init() {
    gui()->properties()->add<String>("Host", "host")->setValue("localhost");
    gui()->properties()->add<String>("Port", "port")->setValue("9003");
}

Visualizer::Ptr SmartStream::Factory::addVisualizer() {
	std::string name = gui()->properties()->get<String>({"__name__"})->value();
    std::string host = gui()->properties()->get<String>({"host"})->value();
    std::string port = gui()->properties()->get<String>({"port"})->value();
	SmartStream::Ptr  vis(new SmartStream(name, host, port));
	return std::dynamic_pointer_cast<Visualizer>(vis);
}


} // FW

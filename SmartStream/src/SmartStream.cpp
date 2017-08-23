#include "SmartStream.h"

#include <mutex>

#include <Library/Colors/Generation.h>
using namespace GUI::Property;

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <set_algo.hpp>

typedef pcl::PointNormal point_t;
typedef pcl::PointCloud<point_t> cloud_t;

using namespace pcl_compress;


namespace FW {

template <typename T>
size_t hash_set(const std::set<T>& values) {
    auto it = values.begin();
    std::hash<T> hasher;
    size_t hash = hasher(*it);
    ++it;
    for(; it != values.end(); ++it) {
        hash ^= hasher(*it) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
    }
    return hash;
}

template <typename T>
size_t hash_vector(const std::vector<T>& values) {
    std::set<T> set_version(values.begin(), values.end());
    return hash_set<T>(set_version);
}


SmartStream::SmartStream(std::string id, const std::string& host, const std::string& port) : Visualizer(id), host_(host), port_(port), crt_face_(-1), tracking_(false) {
}

SmartStream::~SmartStream() {
}

void SmartStream::init() {
    short port = std::atoi(port_.c_str());
    client_ = std::make_shared<vis_client>(host_, port);
    client_->receive_global_scan_data();

    auto mesh = std::make_shared<harmont::triangles_object>(client_->mesh(), Eigen::Vector4f::Ones());
    mesh->init();
    addObject("mesh", mesh);
    client_->renderable_group()->add_object(mesh);

	addProperties();
	registerEvents();

    renderArrangement();

    // render crosshair
    auto crosshair = std::make_shared<harmont::crosshair_object>(Eigen::Vector3f::Zero(), Eigen::Vector4f(1.f, 0.f, 0.f, 1.f), 0.5f, 0.5f);
    crosshair->init();
    addObject("crosshair", crosshair);
}

void SmartStream::addProperties() {
    auto track_btn = gui()->properties()->add<ToggleButton>("Track Camera", "track");
    track_btn->setValue(false);
    track_btn->setCallback([&] (bool state) { tracking_ = state; });


    auto  showGroup = gui()->modes()->addGroup("showGroup");
    showGroup->addOption("showClip", "Enable Clipping", std::string(ICON_PREFIX) + "clipping.png");

    showGroup->setCallback([&] (std::string option, bool state) {
        if (option == "showClip") {
            auto grp = client_->renderable_group();
            if (grp) grp->set_clipping(state);
            //auto objs = objects();
            //for (auto& obj : objs) {
                //const char test[] = "room";
                //if (obj.first.length() >= 4 && strncmp(obj.first.c_str(), test, 4) == 0) {
                    //obj.second->set_clipping(state);
                //}
            //}
        }
    });

    auto  transformGroup = gui()->modes()->addGroup("TransformGroup");
    transformGroup->addOption("Clip", "Modify Clipping Plane", std::string(ICON_PREFIX) + "clipplane.png");

    auto omitMisc = gui()->properties()->add<Bool>("Omit Misc", "omit");
    omitMisc->setValue(false);

    auto omitBase = gui()->properties()->add<Bool>("Omit Base", "omit_base");
    omitBase->setValue(true);
    omitBase->setCallback([&] (bool state) { object("mesh")->set_active(state); });
}

void SmartStream::registerEvents() {
    fw()->events()->connect<void (int, int)>("LEFT_CLICK", [&] (int x, int y) {
        Eigen::Vector3f dir = -fw()->camera()->forward();
        if (1.f - fabs(dir[2]) < 0.01f) dir = fw()->camera()->up();
        auto he = client_->arrangement().getClosestHalfedgeForRay(fw()->camera()->position(), dir);
        if (he) {
            std::cout << (*he)->face()->data()->index << " , " << (*he)->twin()->face()->data()->index << "\n";
        }
        if (he && (*he)->face()->data()->index >= 0 && (*he)->twin()->face()->data()->index >= 0) {
            auto spline = client_->arrangement().getSplineForHalfedge(*he);
            std::cout << "found edge" << "\n";
            auto patches = spline->patches();
            uint32_t query_index = *std::min_element(patches.begin(), patches.end());
            auto cloud = client_->renderable(query_index);
            if (cloud) {
                cloud->set_point_colors(Eigen::Vector4f(1.f, 1.f, 1.f, 0.03f));
            }
        }
    });
    fw()->events()->connect<void (int, int, int, int)>("LEFT_DRAG", [&] (int dx, int dy, int, int) {
        bool clipping = gui()->modes()->group("TransformGroup")->option("Clip")->active();

        if (clipping) {
            auto grp = client_->renderable_group();
            if (grp) grp->delta_clipping_height(-dy * 0.01f);
            //auto objs = objects();
            //for (auto& obj : objs) {
                //const char test[] = "room";
                //if (obj.first.length() >= 4 && strncmp(obj.first.c_str(), test, 4) == 0) {
                    //obj.second->delta_clipping_height(-dy * 0.01f);
                //}
            //}
        }
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

void SmartStream::update(Arr::Face_handle pickedFace) {
    std::vector<request_t> priority_sets;
    std::vector<request_t> new_sets;
    std::vector<int> removable;
    computeNewSets(priority_sets, new_sets, removable, pickedFace);
    //for (const auto& idx : old_rooms) {
        //tryRemoveObject("room_" + std::to_string(idx));
    //}
    std::lock_guard<std::mutex> lg(request_queue_mutex_);
    //std::cout << "adding " << new_sets.size() << " sets" << "\n";

    request_queue_.insert(request_queue_.begin(), priority_sets.begin(), priority_sets.end());
    request_queue_.insert(request_queue_.end(), new_sets.begin(), new_sets.end());

    //for (const auto& idx : new_rooms) {
        //bool has_object = false;
        //try {
            //object("room_"+std::to_string(idx));
            //has_object = true;
        //} catch (...) {
        //}
        //if (!has_object) {
            //request_queue_.push_back(idx);
        //}
        ////client_->request_room(idx, this);
    //}
}

void SmartStream::preRender() {
    client_->render_poll();
    if (!client_->idle()) gui()->status()->set("Streaming...");
    else gui()->status()->set("Idle.");
    {
        request_queue_mutex_.lock();
        if (client_->idle() && !request_queue_.empty()) {
            request_t req = request_queue_.front();
            request_queue_.pop_front();
            request_queue_mutex_.unlock();
            client_->request(req, this);
        } else {
            request_queue_mutex_.unlock();
        }
    }
        //std::future_status status = current_thread_.wait_for(std::chrono::milliseconds(1));

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

        update(pickedFace.get());

        crt_face_ = pickedFace.get()->data()->index;
    }

}

std::set<uint32_t>
SmartStream::getPatches(int face_idx) {
    auto f = client_->arrangement().face(face_idx);
    std::set<uint32_t> patches(f->data()->patchIndices.begin(), f->data()->patchIndices.end());
    return patches;
}

std::set<uint32_t>
SmartStream::getWallPatches(int face_idx) {
    std::set<uint32_t> wall_patches;

    auto curCirc = client_->arrangement().face(face_idx)->outer_ccb();
    auto endCirc = curCirc;
    do {
        wall_patches.insert(curCirc->data()->patchIndices.begin(), curCirc->data()->patchIndices.end());

        ++curCirc;
    } while (curCirc != endCirc);

    return wall_patches;
}

void
SmartStream::computeNewSets(std::vector<request_t>& priority_sets,
                            std::vector<request_t>& new_sets,
                            std::vector<int>& removable,
                            const Arr::Face_handle& current_face) {
    int current_index = current_face->data()->index;
    if (current_index < 0) return;

    //Eigen::Vector4f col_spline0(1.f, 0.f, 1.f, 1.f);
    //Eigen::Vector4f col_misc0(1.f, 0.f, 0.f, 1.f);
    //Eigen::Vector4f col_spline1(0.f, 0.f, 1.f, 1.f);
    //Eigen::Vector4f col_misc1(0.f, 1.f, 1.f, 1.f);
    Eigen::Vector4f col_spline0(1.f, 1.f, 1.f, 1.f);
    Eigen::Vector4f col_misc0(1.f, 1.f, 1.f, 1.f);
    Eigen::Vector4f col_spline1(1.f, 1.f, 1.f, 1.f);
    Eigen::Vector4f col_misc1(1.f, 1.f, 1.f, 1.f);

    // compute face 1- and 2-ring
    std::set<int> face_ring_1, face_ring_2;
    auto ring1 = client_->arrangement().getOneRingFaces(current_face);
    for (auto f1 : ring1) {
        face_ring_1.insert(f1->data()->index);
        auto ring2 = client_->arrangement().getOneRingFaces(f1);
        for (const auto& f2 : ring2) {
            face_ring_2.insert(f2->data()->index);
        }
    }
    face_ring_2 = subtract(face_ring_2, face_ring_1);
    face_ring_2.erase(current_index);

    // get wall splines and patches for current face, ring 1 and ring 2
    std::set<std::shared_ptr<RoomArr::RoomArrangement::Spline>> splines_0, splines_1, splines_2;
    std::map<uint32_t, uint32_t> spline_indices_1, spline_indices_2;
    std::set<uint32_t> walls_0, walls_1, walls_2;
    // we also have to include the current room
    auto splines = client_->arrangement().getSplinesForFace(current_index);
    splines_0.insert(splines.begin(), splines.end());
    for (const auto& spline : splines) {
        std::vector<uint32_t> patches = spline->patches();
        walls_0.insert(patches.begin(), patches.end());
    }

    for (const auto& face_idx : face_ring_1) {
        auto splines = client_->arrangement().getSplinesForFace(face_idx);
        splines_1.insert(splines.begin(), splines.end());
        for (const auto& spline : splines) {
            spline_indices_1[spline->index()] = face_idx;
            std::vector<uint32_t> patches = spline->patches();
            walls_1.insert(patches.begin(), patches.end());
        }
    }
    //size_t hash_1 = hash_set(face_ring_1);

    for (const auto& face_idx : face_ring_2) {
        auto splines = client_->arrangement().getSplinesForFace(face_idx);
        splines_2.insert(splines.begin(), splines.end());
        for (const auto& spline : splines) {
            spline_indices_2[spline->index()] = face_idx;
            std::vector<uint32_t> patches = spline->patches();
            walls_2.insert(patches.begin(), patches.end());
        }
    }
    //size_t hash_2 = hash_set(face_ring_2);

    splines_1 = subtract(splines_1, splines_0);
    splines_2 = subtract(splines_2, splines_1);

    // get misc patches for current face, ring 1 and ring 2
    std::set<uint32_t> misc_0, misc_1, misc_2;
    // we also have to include the current room
    std::set<uint32_t> all_patches = getPatches(current_index);
    misc_0 = subtract(all_patches, walls_0);
    for (const auto& face_idx : face_ring_1) {
        std::set<uint32_t> all_patches = getPatches(face_idx);
        std::set<uint32_t> patches = subtract(all_patches, walls_1);
        misc_1.insert(patches.begin(), patches.end());
    }
    for (const auto& face_idx : face_ring_2) {
        std::set<uint32_t> all_patches = getPatches(face_idx);
        std::set<uint32_t> patches = subtract(all_patches, walls_2);
        misc_2.insert(patches.begin(), patches.end());
    }

    // build query sets
    new_sets.clear();
    priority_sets.clear();
    merged_global_data_t& gdata = client_->global_data();

    uint32_t count_0 = 0;

    bool omitBase = gui()->properties()->get<Bool>({"omit_base"})->value();
    if (!omitBase) {
        for (const auto& spline : splines_0) {
            if (!spline->patches().size()) continue;
            auto crt_hash = hash_vector(spline->patches());
            count_0 += spline->patches().size();
            priority_sets.push_back(request_t(col_spline0, crt_hash, set_type_t::walls, spline->patches()));
        }
    }

    std::vector<uint32_t> floors, rest;
    bool omitMisc = gui()->properties()->get<Bool>({"omit"})->value();
    for (uint32_t idx : misc_0) {
        vec3f_t n = gdata.bases[idx].col(2).normalized();
        if (1.f - fabs(n[2]) < 0.01f) {
            floors.push_back(idx);
        } else {
            rest.push_back(idx);
        }
    }
    count_0 += floors.size();
    count_0 += rest.size();
    if (!omitBase && floors.size()) {
        priority_sets.push_back(request_t(col_misc0, hash_vector(floors), set_type_t::interior, floors));
    }
    if (!omitMisc && rest.size()) {
        priority_sets.push_back(request_t(col_misc0, hash_vector(rest), set_type_t::interior, rest));
    }

    uint32_t count_1 = 0;
    if (!omitBase) {
        for (const auto& spline : splines_1) {
            if (!spline->patches().size()) continue;
            auto crt_hash = hash_vector(spline->patches());
            count_1 += spline->patches().size();
            new_sets.push_back(request_t(col_spline1, crt_hash, set_type_t::walls, spline->patches()));
        }
    }
    //for (const auto& spline : splines_2) {
        //if (!spline->patches().size()) continue;
        //auto crt_hash = hash_vector(spline->patches());
        //new_sets.push_back(request_t(true, crt_hash, set_type_t::walls, spline->patches()));
    //}

    floors.clear();
    rest.clear();
    for (uint32_t idx : misc_1) {
        vec3f_t n = gdata.bases[idx].col(2).normalized();
        if (1.f - fabs(n[2]) < 0.01f) {
            floors.push_back(idx);
        } else {
            rest.push_back(idx);
        }
    }
    count_1 += floors.size();
    count_1 += rest.size();
    if (!omitBase && floors.size()) {
        new_sets.push_back(request_t(col_misc1, hash_vector(floors), set_type_t::interior, floors));
    }
    if (!omitMisc && rest.size()) {
        new_sets.push_back(request_t(col_misc1, hash_vector(rest), set_type_t::interior, rest));
    }

    std::cout << count_0 << "\n";
    std::cout << count_1 << "\n";

    //floors.clear();
    //rest.clear();
    //for (uint32_t idx : misc_2) {
        //vec3f_t n = gdata.bases[idx].col(2).normalized();
        //if (1.f - fabs(n[2]) < 0.01f) {
            //floors.push_back(idx);
        //} else {
            //rest.push_back(idx);
        //}
    //}
    //new_sets.push_back(request_t(true, static_cast<uint32_t>(hash_2), set_type_t::interior, floors));
    //new_sets.push_back(request_t(true, static_cast<uint32_t>(hash_2), set_type_t::interior, rest));

    //std::set<int> new_scans_, removable_;
    //int current_index = current_face->data()->index;
    //if (current_index >= 0) {
        //std::set<int> current_scans;
        //current_scans.insert(current_index);
        //auto ring = client_->arrangement().getOneRingFaces(current_face);
        //std::cout << "Num one ring faces: " << ring.size() << std::endl;
        //for (auto f : ring) {
            //current_scans.insert(f->data()->index);
        //}

        //std::set_difference(current_scans.begin(), current_scans.end(),
                            //current_scans_.begin(), current_scans_.end(),
                            //std::inserter(new_scans_, new_scans_.end()));
        //std::set_difference(current_scans_.begin(), current_scans_.end(),
                            //current_scans.begin(), current_scans.end(),
                            //std::inserter(removable_, removable_.end()));

        //new_scans.clear();
        //removable.clear();
        //if (new_scans_.find(current_index) != new_scans_.end()) {
            //new_scans.push_back(current_index);
        //}
        //for (int idx : new_scans_) {
            //if (idx != current_index) new_scans.push_back(idx);
        //}
        //for (int idx : removable) {
            //removable.push_back(idx);
        //}

        ////query_patches.clear();
        ////for (const auto& idx : new_scans) {
            ////Arr::Face_handle f = face_map_[idx];
            ////query_patches.insert(f->data()->patchIndices.begin(),
                                 ////f->data()->patchIndices.end());
        ////}

        //current_scans_ = current_scans;
    //}
}

SmartStream::Factory::Factory() : FW::Factory() {
}

SmartStream::Factory::~Factory() {
}

void SmartStream::Factory::init() {
    //gui()->properties()->add<String>("Host", "host")->setValue("oglaroon.cc");
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

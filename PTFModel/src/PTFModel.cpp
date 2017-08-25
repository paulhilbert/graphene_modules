#include "PTFModel.h"

#include <chrono>
#include <Library/Colors/Generation.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/octree/octree_impl.h>

#define USE_CEREAL
#include <damogran/cereal.hpp>

#include <harmont/box_object.hpp>

using namespace GUI::Property;

#define CLOUD_SIZE_LIMIT 5000000


#define OPTIX_BUF_TYPE RTP_BUFFER_TYPE_HOST      // RTP_BUFFER_TYPE_CUDA_LINEAR



namespace FW {


PTFModel::PTFModel(std::string id, const fs::path& model_path, const fs::path& scene_path, float dist_step_count, float angle_step, float min_diameter) : Visualizer(id), model_path_(model_path), scene_path_(scene_path), discr_params_{dist_step_count, angle_step}, min_diameter_(min_diameter) {
}

PTFModel::~PTFModel() {
}

void PTFModel::init() {
    //gstate_ = std::make_shared<vs::gpu_state>();
    //score_ = std::make_unique<vs::score_functor<point_t,point_t>>(gstate_);


    // model
    cloud_t::Ptr model(new cloud_t());
    pcl::io::loadPCDFile(model_path_.string(), *model);
    model_obj_ = std::make_shared<harmont::pointcloud_object<cloud_t, boost::shared_ptr>>(model, true);
    model_obj_->init();

    //score_->set_model(model, 100);

    model_ = std::make_shared<tr::model<point_t>>(model, discr_params_);
    sample_params_.min_diameter_factor = min_diameter_;
    sample_params_.max_diameter_factor = 0.8f;
    sample_params_.min_orthogonality = 0.8f;
    sample_params_.redundancy_factor = 1.f;
    auto sync_model = model_->init(sample_params_);
    sync_model.wait();
    gui()->log()->info("model triplet count: " + std::to_string(model_->triplet_count()));

    std::vector<Eigen::Vector4f> model_colors(model->size(), Eigen::Vector4f(0.f, 0.4f, 1.f, 1.f));
    for (auto idx : model_->used_points()) {
        model_colors[idx] = Eigen::Vector4f(1.f, 1.f, 0.f, 1.f);
    }

    model_obj_->set_point_colors(model_colors);
    addObject("model", model_obj_);


    // scene
    cloud_t::Ptr scene(new cloud_t());
    pcl::io::loadPCDFile(scene_path_.string(), *scene);
    subset_.resize(scene->size());
    std::iota(subset_.begin(), subset_.end(), 0);
    scene_obj_ = std::make_shared<harmont::pointcloud_object<cloud_t, boost::shared_ptr>>(scene, true);
    scene_obj_->init();
    addObject("scene", scene_obj_);
    //score_->set_scene(scene);
    //scene_ = std::make_unique<tr::scene<point_t>>(scene);
    search_ = std::make_unique<tr::stratified_search<point_t>>(scene, sample_params_, model_->diameter(), 1.f);

    search_->set_model(model_);

    //model_->write_octave_density_maps("/tmp", "hist", "hash_density.m");


    /// OCTREE
    auto tree = search_->get_octree();
    std::vector<Eigen::Vector4f> voxel_colors = {
        Eigen::Vector4f(1.f, 0.f, 0.f, 1.f),
        Eigen::Vector4f(0.f, 1.f, 0.f, 1.f),
        Eigen::Vector4f(0.f, 0.f, 1.f, 1.f),
        Eigen::Vector4f(1.f, 1.f, 0.f, 1.f),
        Eigen::Vector4f(0.f, 1.f, 1.f, 1.f),
        Eigen::Vector4f(1.f, 0.f, 1.f, 1.f),
        Eigen::Vector4f(0.f, 0.f, 0.f, 1.f),
        Eigen::Vector4f(1.f, 1.f, 1.f, 1.f)
    };
    uint32_t col_idx = 0;
    octree_ = std::make_shared<harmont::renderable_group>(std::vector<harmont::renderable::ptr_t>());
    ranges::for_each(tr::octree<point_t>::leaf_traverse(tree->root()), [&] (auto const& node) {
        tr::leaf_node const& leaf = std::get<tr::leaf_node>(node);
        auto box = std::make_shared<harmont::box_object>(leaf.bbox.min(), leaf.bbox.max(), voxel_colors[(col_idx++) % 8], true);
        octree_->add_object(box);
    });
    octree_->init();
    octree_->set_active(false);
    addObjectGroup("octree", octree_);

    //
    //
    //std::vector<std::shared_ptr<harmont::renderable>> voxels;
    //auto& octree = search_->octree();
    //uint32_t max_depth = 0;//octree.getTreeDepth();
    //for(auto node_it = octree.breadth_begin(); node_it != octree.breadth_end(); ++node_it) {
        //uint32_t depth = node_it.getCurrentOctreeDepth();
        //if (depth > max_depth) break;
        //Eigen::Vector3f lower, upper;
        //octree.getVoxelBounds(node_it, lower, upper);
        //Eigen::Vector4f color = Eigen::Vector4f::Ones();
        //auto obj = std::make_shared<harmont::box_object>(lower, upper, color, true);
        //voxels.push_back(std::dynamic_pointer_cast<harmont::renderable>(obj));
        ////if (node_it.isLeafNode()) layers[depth].second += 1;
        ////else                      layers[depth].first += 1;
    //}
    //octree_ = std::make_shared<harmont::renderable_group>(voxels);
    //octree_->init();
    //octree_->set_active(false);
    //addObjectGroup("octree", octree_);


	addProperties();
	registerEvents();
}

void PTFModel::addProperties() {
    auto mcov = gui()->properties()->add<Number>("Match Coverage Threshold", "match_coverage");
    mcov->setMin(0.0).setMax(1.0).setDigits(2).setValue(0.5f);
    auto btn_upd = gui()->properties()->add<Button>("Update", "update");
    btn_upd->setCallback([&] () {
        //auto && [ts, subset] = search_->find_all(*model_, 0.9f, 0.5f);
        auto before = std::chrono::system_clock::now();
        auto && [ts, subsets] = search_->find_all(*model_, gui()->properties()->get<Number>({"match_coverage"})->value(), 0.5f);
        //auto subset = search_->find(*model_, 0.9f, 0.5f).second;
        auto after = std::chrono::system_clock::now();
        std::cout << "Found " << ts.size() << " transformations in " << std::chrono::duration_cast<std::chrono::seconds>(after - before).count() << "s " << "\n";

        /*
        model_obj_->set_transformation(t);
        */
        uint32_t n = scene_obj_->cloud()->size();
        std::vector<Eigen::Vector4f> colors(n, Eigen::Vector4f(1.f, 0.f, 0.f, 1.f));
        //std::vector<Eigen::Vector4f> hues = Colors::Generation::shuffledColorsRGBA(subsets.size());
        for (uint32_t sub = 0; sub < subsets.size(); ++sub) {
            for (const auto& idx : subsets[sub]) {
                //colors[idx] = hues[sub];
                colors[idx] = Eigen::Vector4f::Ones();
            }
        }

        //std::vector<uint32_t> residual;
        //std::set_difference(subset_.begin(), subset_.end(), subset.begin(), subset.end(), std::back_inserter(residual));
        //subset_ = residual;

        //std::vector<Eigen::Vector4f> colors(n, Eigen::Vector4f(1.f, 0.f, 0.f, 0.9f));
        //for (int idx : subset_) {
            //colors[idx] = Eigen::Vector4f(1.f, 1.f, 1.f, 0.9f);
        //}
        scene_obj_->set_point_colors(colors);
    });
    //auto btn_corr = gui()->properties()->add<Button>("Correspondences", "corr");
    //btn_corr->setCallback([&] () {
    //});
    auto sv = gui()->properties()->add<File>("Save subset", "save_subset");
    sv->setMode(File::SAVE);
    sv->setExtensions({"pcd"});
    sv->setCallback([&] (const fs::path& p) {
        if (subset_.empty()) return;
        cloud_t subset_cloud;
        subset_cloud.resize(subset_.size());
        uint32_t i=0;
        for (const auto& idx : subset_) {
            subset_cloud[i++] = scene_obj_->cloud()->points[idx];
        }
        pcl::io::savePCDFile(p.string(), subset_cloud);
    });

    auto osec = gui()->properties()->add<Section>("Octree", "octree");
    auto toggle_octree = gui()->properties()->add<Boolean>("Show Octree", "show_octree");
    toggle_octree->setValue(false);
    toggle_octree->setCallback([&] (bool state) { octree_->set_active(state); });

    //auto voxel_visible = gui()->properties()->add<Boolean>("Show Voxel Grid", "voxel_visible");
    //voxel_visible->setValue(false);
    //voxel_visible->setCallback([&] (bool state) { voxel_obj_->set_active(state); });
}

void PTFModel::registerEvents() {
	fw()->events()->connect<void (int, int)>("LEFT_DRAG_START", [&] (int x, int y) {
        start_x = x;
        start_y = y;
    });
	fw()->events()->connect<void (int, int)>("LEFT_DRAG_STOP", [&] (int x, int y) {
        Eigen::Matrix4f v = fw()->camera()->view_matrix();
        const Eigen::Matrix4f& p = fw()->camera()->projection_matrix();
        Eigen::Matrix<uint32_t, 4, 1> vp(0, 0, fw()->camera()->width(), fw()->camera()->height());

        subset_.clear();
        std::vector<Eigen::Vector4f> colors(scene_obj_->cloud()->size(), Eigen::Vector4f(0.2f, 0.2f, 0.2f, 1.f));
        int lower_x = std::min(start_x, x);
        int lower_y = std::min(start_y, y);
        int upper_x = std::max(start_x, x);
        int upper_y = std::max(start_y, y);
        for (uint32_t i = 0; i < scene_obj_->cloud()->size(); ++i) {
            Eigen::Vector2i c = harmont::project(scene_obj_->cloud()->points[i].getVector3fMap(), v, p, vp).head(2).template cast<int>();
            if (c[0] >= lower_x && c[1] >= lower_y && c[0] <= upper_x && c[1] <= upper_y) {
                subset_.push_back(i);
                colors[i] = Eigen::Vector4f(1.f, 0.8f, 0.f, 1.f);
            }
        }
        scene_obj_->set_point_colors(colors);
    });
}

PTFModel::Factory::Factory() : FW::Factory() {
}

PTFModel::Factory::~Factory() {
}

void PTFModel::Factory::init() {
    gui()->properties()->add<File>("Input Model PCD File", "model_path")->setExtensions({"pcd"});
    gui()->properties()->add<File>("Input Scene PCD File", "scene_path")->setExtensions({"pcd"});
    gui()->properties()->add<Number>("Distance Step Count", "dist_step_count")->setMin(1).setMax(100).setDigits(0).setValue(15);
    gui()->properties()->add<Number>("Angle Step", "angle_step")->setMin(0.1f).setMax(360.f).setDigits(1).setValue(12);
    gui()->properties()->add<Number>("Min Diameter Factor", "min_diameter")->setMin(0.0f).setMax(1.f).setDigits(2).setValue(0.4f);
}

Visualizer::Ptr PTFModel::Factory::addVisualizer() {
	std::string name = gui()->properties()->get<String>({"__name__"})->value();
    auto mpath = gui()->properties()->get<File>({"model_path"})->value();
    auto spath = gui()->properties()->get<File>({"scene_path"})->value();
    auto dist_step_count = gui()->properties()->get<Number>({"dist_step_count"})->value();
    auto angle_step = gui()->properties()->get<Number>({"angle_step"})->value();
    auto min_diameter = gui()->properties()->get<Number>({"min_diameter"})->value();
	PTFModel::Ptr  vis(new PTFModel(name, mpath, spath, dist_step_count, angle_step * static_cast<float>(M_PI) / 180.f, min_diameter));
	return std::dynamic_pointer_cast<Visualizer>(vis);
}


} // FW

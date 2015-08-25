#include "RenderE57Stream.h"

#include <mutex>

#include <Library/Colors/Generation.h>
using namespace GUI::Property;

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#define USE_CEREAL
#include <pcl_compress_client/client.hpp>
using namespace pcl_compress;

#include <boost/bind.hpp>

typedef pcl::PointNormal point_t;
typedef pcl::PointCloud<point_t> cloud_t;



class stream_client : public client {
public:
    typedef std::shared_ptr<stream_client> ptr_t;
    typedef std::weak_ptr<stream_client> wptr_t;
    typedef std::shared_ptr<const stream_client> const_ptr_t;
    typedef std::weak_ptr<const stream_client> const_wptr_t;

    typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> vertex_data_t;

public:
    stream_client(asio::io_context &io_context,
                  const asio::ip::tcp::resolver::results_type &endpoints,
                  uint32_t min_patch_count = 50)
        : client(io_context, endpoints),
          min_patch_count_(min_patch_count), last_patch_count_(0),
          patch_count_(0), range_start_(0), last_point_idx_(0), update_cloud_(false) {}

    virtual ~stream_client() {
    }

    void init_cloud(harmont::renderable::ptr_t& cloud, FW::RenderE57Stream* vis) {
        if (cloud || !patch_count_) return;
        cloud = std::make_shared<harmont::pointcloud_object<cloud_normal_t, boost::shared_ptr>>(global_data_.num_points);
        cloud->init();
        vis->addObject("cloud", cloud);
    }

    void update_cloud(harmont::renderable::ptr_t& cloud, FW::RenderE57Stream* vis) {
    //void update_cloud(harmont::renderable::ptr_t& cloud, std::shared_ptr<harmont::renderable::map_t> display_map, std::shared_ptr<harmont::renderable::map_t> shadow_map) {
        if (data_mutex_.try_lock()) {
            if (update_cloud_) {
                auto display_map = cloud->eigen_map_display_buffer();
                auto shadow_map = cloud->eigen_map_shadow_buffer();
                uint32_t count = last_point_idx_ - range_start_;
                display_map.block(range_start_, 0, count, 9) = vertex_data_.block(range_start_, 0, count, 9);
                shadow_map.block(range_start_, 0, count, 3) = vertex_data_.block(range_start_, 0, count, 3);
                range_start_ = last_point_idx_;
                cloud->unmap_display_buffer();
                cloud->unmap_shadow_buffer();
                //cloud->update_geometry(vertex_data_);
                cloud->set_bounding_box(bbox_);
                update_cloud_ = false;
            }
            data_mutex_.unlock();
        }
    }

protected:
    void on_global_data_ready(const global_data_t& data) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        global_data_ = data;
        vertex_data_ = harmont::renderable::vertex_data_t::Zero(global_data_.num_points, 9);
        float rgba = harmont::renderable::color_to_rgba(Eigen::Vector4f::Ones());
        //vertex_data_.block(0, 0, global_data_.num_points, 3) = vertex_data_t::Random(global_data_.num_points, 3);
        vertex_data_.col(3) = Eigen::VectorXf::Constant(global_data_.num_points, rgba);
        //vertex_data_.col(4) = Eigen::VectorXf::Ones(global_data_.num_points);
    }

    void on_patch_ready(cloud_normal_t::Ptr patch) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        for (const auto& point : *patch) {
            vec3f_t pos = point.getVector3fMap() - global_data_.scan_origin;
            vec3f_t nrm = point.getNormalVector3fMap();
            bbox_.extend(pos);
            vertex_data_.block(last_point_idx_, 0, 1, 3) = pos.transpose();
            vertex_data_.block(last_point_idx_, 4, 1, 3) = nrm.transpose();
            vertex_data_.block(last_point_idx_, 7, 1, 2) = Eigen::RowVector2f::Zero();
            vertex_data_(last_point_idx_, 3) = harmont::renderable::color_to_rgba(Eigen::Vector4f::Ones());
            ++last_point_idx_;
        }
        ++patch_count_;
        if (patch_count_ - last_patch_count_ >= min_patch_count_) {
            last_patch_count_ = patch_count_;
            update_cloud_ = true;
        }
    }

    void on_cloud_finished() {
        std::cout << "done" << "\n";
        update_cloud_ = (last_patch_count_ < global_data_.num_patches);
        do_close();
    }

protected:
    uint32_t min_patch_count_;
    uint32_t last_patch_count_;
    uint32_t patch_count_;
    uint32_t range_start_;
    uint32_t last_point_idx_;
    global_data_t global_data_;
    bool update_cloud_;
    std::mutex data_mutex_;
    std::mutex cloud_mutex_;
    vertex_data_t vertex_data_;
    bbox3f_t bbox_;
};


namespace FW {


RenderE57Stream::RenderE57Stream(std::string id, const std::string& host, const std::string& port) : Visualizer(id), host_(host), port_(port) {
}

RenderE57Stream::~RenderE57Stream() {
    stream_thread_->join();
    client_->close();
}

void RenderE57Stream::init() {
    asio::ip::tcp::resolver resolver(io_context_);
    asio::ip::tcp::resolver::results_type endpoints =
        resolver.resolve(host_.c_str(), port_.c_str());

    client_ = std::make_shared<stream_client>(io_context_, endpoints);
    stream_thread_ = std::make_shared<std::thread>(std::bind((size_t(asio::io_context::*)())&asio::io_context::run, &io_context_));

	addProperties();
	registerEvents();
}

void RenderE57Stream::addProperties() {
    //auto btn = gui()->properties()->add<Bool>("Active Obj", "act");
    //btn->setValue(true);
    //btn->setCallback([&] (bool state) {
        //if (renderable_) renderable_->set_active(state);
    //});
	//auto  showGroup = gui()->modes()->addGroup("showGroup");
	//showGroup->addOption("showClip", "Enable Clipping", std::string(ICON_PREFIX) + "clipping.png");

	//showGroup->setCallback([&] (std::string option, bool state) {
        //if (option == "showClip") {
            //group_->set_clipping(state);
        //}
    //});

	//auto  transformGroup = gui()->modes()->addGroup("TransformGroup");
	//transformGroup->addOption("Clip", "Modify Clipping Plane", std::string(ICON_PREFIX) + "clipplane.png");
}

void RenderE57Stream::registerEvents() {
	//fw()->events()->connect<void (int, int, int, int)>("LEFT_DRAG", [&] (int dx, int dy, int, int) {
        //bool clipping = gui()->modes()->group("TransformGroup")->option("Clip")->active();

        //if (clipping) {
            //group_->delta_clipping_height(-dy * 0.01f);
        //}
    //});
}

void RenderE57Stream::preRender() {
    //std::cout << "." << "\n";
    if (!renderable_) client_->init_cloud(renderable_, this);
    if (renderable_) {
        //if (!display_map_) {
            //typedef harmont::renderable::map_t map_t;
            //display_map_ = std::make_shared<map_t>(renderable_->eigen_map_display_buffer());
            //shadow_map_ = std::make_shared<map_t>(renderable_->eigen_map_shadow_buffer());
            //*display_map_ = renderable_->eigen_map_display_buffer();
            //*shadow_map_ = renderable_->eigen_map_shadow_buffer();
        //}
        client_->update_cloud(renderable_, this);
        //client_->update_cloud(renderable_, display_map_, shadow_map_);
    }
}

RenderE57Stream::Factory::Factory() : FW::Factory() {
}

RenderE57Stream::Factory::~Factory() {
}

void RenderE57Stream::Factory::init() {
    gui()->properties()->add<String>("Host", "host")->setValue("oglaroon.cc");
    gui()->properties()->add<String>("Port", "port")->setValue("9003");
}

Visualizer::Ptr RenderE57Stream::Factory::addVisualizer() {
	std::string name = gui()->properties()->get<String>({"__name__"})->value();
    std::string host = gui()->properties()->get<String>({"host"})->value();
    std::string port = gui()->properties()->get<String>({"port"})->value();
	RenderE57Stream::Ptr  vis(new RenderE57Stream(name, host, port));
	return std::dynamic_pointer_cast<Visualizer>(vis);
}


} // FW

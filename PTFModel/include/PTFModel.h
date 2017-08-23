#ifndef PTFModelVIS_H_
#define PTFModelVIS_H_

#include <boost/optional.hpp>
#include <boost/none.hpp>

#include <FW/FWVisualizer.h>
#include <FW/FWFactory.h>

#include <voxel_score/score_functor>
namespace vs = voxel_score;
#include <triplet_match/stratified_search>
namespace tr = triplet_match;

typedef pcl::PointNormal point_t;
typedef pcl::PointCloud<point_t> cloud_t;

namespace FW {


class PTFModel : public FW::Visualizer {
	public:
		typedef std::shared_ptr<PTFModel>    Ptr;
		typedef std::weak_ptr<PTFModel>      WPtr;

	public:
		class Factory;
        class Representation;

	public:
		PTFModel(std::string id, const fs::path& model_path, const fs::path& scene_path, float dist_step_count, float angle_step, float min_diameter);
		virtual ~PTFModel();

		void init();
        //void update();
		void addProperties();
		void registerEvents();

    protected:
        void createOptixStructure();
        boost::optional<uint32_t> selectObject(const Eigen::Vector3f& origin, const Eigen::Vector3f& dir);

    protected:
        fs::path   model_path_;
        fs::path   scene_path_;
        std::shared_ptr<harmont::pointcloud_object<cloud_t, boost::shared_ptr>> model_obj_;
        std::shared_ptr<harmont::pointcloud_object<cloud_t, boost::shared_ptr>> scene_obj_;
        tr::stratified_search<point_t>::uptr_t search_;
        //std::shared_ptr<harmont::pointcloud_object<cloud_t, boost::shared_ptr>> voxel_obj_;
        //vs::gpu_state::sptr_t gstate_;
        //vs::score_functor<point_t, point_t>::uptr_t score_;
        std::shared_ptr<tr::model<point_t>> model_;
        //std::unique_ptr<tr::scene<point_t>> scene_;
        cloud_t::Ptr scene_cloud_;
        Eigen::Matrix4f transform_;
        tr::sample_parameters sample_params_;
        tr::discretization_params discr_params_;
        float last_best_score_;
        int start_x;
        int start_y;
        std::vector<uint32_t> subset_;
        float min_diameter_;
        std::shared_ptr<harmont::renderable_group> octree_;
};


class PTFModel::Factory : public FW::Factory {
	public:
		Factory();
		virtual ~Factory();

		void init();
		Visualizer::Ptr addVisualizer();
};


} // FW

#endif /* PTFModelVIS_H_ */

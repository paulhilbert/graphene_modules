#include "Registration.h"

#include <Library/Colors/Generation.h>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Mesh/Traits.hh>
#include <OpenMesh/Core/Utils/color_cast.hh>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <duraark/turtle_input.hpp>
#include <duraark/extract_objects.hpp>
#include <duraark/pair_difference.hpp>

#include <Eigen/Geometry>
#include <Geometry/PCLTools.h>


namespace FW {


typedef pcl::PointNormal        Point;
typedef pcl::PointCloud<Point>  Cloud;


class Registration::Representation {
	public:
		typedef std::shared_ptr<Representation>           ptr;
		typedef std::weak_ptr<Representation>             wptr;
		typedef std::shared_ptr<const Representation>     const_ptr;
		typedef std::weak_ptr<const Representation>       const_wptr;
		typedef Eigen::Affine3f                           TransformType;
		typedef OpenMesh::Vec4f                           ColorType;
		typedef cartan::openmesh_t<ColorType>             Mesh;
		typedef std::shared_ptr<Mesh>                     MeshPtr;
		typedef harmont::mesh_object<Mesh>                RenderedMesh;
		typedef cartan::mesh_traits<Mesh>                 MeshTraits;
		typedef harmont::renderable::ptr_t                RenderablePtr;
		typedef std::vector<RenderablePtr>                Renderables;
		typedef std::function<void (int, RenderablePtr)>  AddRenderableCallback;

	public:
		Representation(const std::vector<fs::path>& paths);
		virtual ~Representation();

		void center();

		bool isCloud() const;
		bool isMesh() const;
		bool isIFC() const;

		Renderables&       renderables();
		const Renderables& renderables() const;
        harmont::renderable_group::ptr_t group();
        harmont::renderable_group::const_ptr_t group() const;
		Cloud::Ptr cloud();
		MeshPtr mesh();
		duraark::ifc_objects_t<ColorType>& objects();
		const duraark::ifc_objects_t<ColorType>& objects() const;
		const std::vector<uint32_t>& cloudSizes() const;
		const std::vector<Eigen::Vector3f>& cloudOrigins() const;

		TransformType transformation() const;
		void transform(const TransformType& t);

		void annotate(const std::vector<int32_t>& associations, const std::vector<uint32_t>& possible_hits, const std::vector<uint32_t>& actual_hits, float ratio_threshold, float points_per_square, Visualizer* vis, const Eigen::Vector4f& np_col, const Eigen::Vector4f& mp_col, const Eigen::Vector4f fp_col);

	protected:
		std::vector<fs::path>              m_paths;
		Renderables                        m_renderables;
		MeshPtr                            m_mesh;
		Cloud::Ptr                         m_cloud;
		duraark::ifc_objects_t<ColorType>  m_objects;
		std::vector<std::vector<int>>      m_vertexMap;
		std::vector<uint32_t>              m_cloudSizes;
		std::vector<Eigen::Vector3f>       m_cloudOrigins;
        harmont::renderable_group::ptr_t   m_group;
};


Registration::Registration(std::string id, const std::vector<fs::path>& paths0, const std::vector<fs::path>& paths1) : Visualizer(id), m_paths0(paths0), m_paths1(paths1) {
}

Registration::~Registration() {
}

void Registration::init() {
	try {
		m_rep0 = std::make_shared<Representation>(m_paths0);
	} catch (std::runtime_error& error) {
		gui()->log()->error(std::string("Could not load first representation: ") + error.what());
	}
	try {
		m_rep1 = std::make_shared<Representation>(m_paths1);
	} catch (std::runtime_error& error) {
		gui()->log()->error(std::string("Could not load second representation: ") + error.what());
	}

	m_rep0->center();
	m_rep1->center();
	uint32_t  idx = 0;
	for (auto& r : m_rep0->renderables()) {
		addObject("rep0_" + std::to_string(idx++), r);
	}
	idx = 0;
	for (auto& r : m_rep1->renderables()) {
		addObject("rep1_" + std::to_string(idx++), r);
	}

	addProperties();
	registerEvents();

	m_registration = std::make_shared<duraark::registration>();
}

void Registration::addProperties() {
	auto  showGroup = gui()->modes()->addGroup("showGroup");
	showGroup->addOption("showClip", "Enable Clipping", std::string(ICON_PREFIX) + "clipping.png");

	showGroup->setCallback([&] (std::string option, bool state) {
        if (option == "showClip") {
            for (auto& r: m_rep0->renderables()) {
                r->set_clipping(state);
            }
            for (auto& r: m_rep1->renderables()) {
                r->set_clipping(state);
            }
            if (m_rep0->isIFC()) {
                auto grp = m_rep0->group();
                if (grp) grp->set_clipping(state);
            }
            if (m_rep1->isIFC()) {
                auto grp = m_rep1->group();
                if (grp) grp->set_clipping(state);
            }
        }
    });

	auto  transformGroup = gui()->modes()->addGroup("TransformGroup");
	transformGroup->addOption("TransformA", "Transform A", std::string(ICON_PREFIX) + "transform_a_128.png");
	transformGroup->addOption("TransformB", "Transform B", std::string(ICON_PREFIX) + "transform_b_128.png");
	transformGroup->addOption("Clip", "Modify Clipping Plane", std::string(ICON_PREFIX) + "clipplane.png");


	// auto showGroup = gui()->modes()->addGroup("showGroup");
	// showGroup->addOption("showClip", "Enable Clipping", std::string(ICON_PREFIX)+"clipping.png");
	// auto editGroup = gui()->modes()->addGroup("editGroup");
	// editGroup->addOption("editClip", "Modify Clipping Plane", std::string(ICON_PREFIX)+"clipplane.png");

	auto  registrationSection = gui()->properties()->add<Section>("Registration", "Registration");
	registrationSection->add<Button>("Auto align A -> B", "AlignAB")->setCallback([&] () {
	                                                                                 gui()->status()->set("Computing alignment transformation...");
	                                                                                 setICPParameters_();
	                                                                                 ::duraark::registration::affine_t t;
	                                                                                 Eigen::Affine3f t0, t1;
	                                                                                 t0 = m_rep0->transformation();
	                                                                                 t1 = m_rep1->transformation();
	                                                                                 if (m_rep0->isCloud()) {
	                                                                                    if (m_rep1->isCloud()) {
	                                                                                       t = m_registration->match<Point>(m_rep0->cloud(), m_rep1->cloud(), t0, t1);
																													} else {
	                                                                                       t = m_registration->match<Point, OpenMesh::Vec4f>(m_rep0->cloud(), *(m_rep1->mesh()), t0, t1);
																													}
																												} else {
	                                                                                    if (m_rep1->isCloud()) {
	                                                                                       t = m_registration->match<OpenMesh::Vec4f, Point>(*(m_rep0->mesh()), m_rep1->cloud(), t0, t1);
																													} else {
	                                                                                       t = m_registration->match<OpenMesh::Vec4f>(*(m_rep0->mesh()), *(m_rep1->mesh()), t0, t1);
																													}
																												}
	                                                                                 m_rep0->transform(t);
	                                                                                 gui()->status()->set("Done computing alignment transformation.");
																											});
	registrationSection->add<Button>("Auto align B -> A", "AlignBA")->setCallback([&] () {
	                                                                                 gui()->status()->set("Computing alignment transformation...");
	                                                                                 setICPParameters_();
	                                                                                 ::duraark::registration::affine_t t;
	                                                                                 Eigen::Affine3f t0, t1;
	                                                                                 t0 = m_rep0->transformation();
	                                                                                 t1 = m_rep1->transformation();
	                                                                                 if (m_rep1->isCloud()) {
	                                                                                    if (m_rep0->isCloud()) {
	                                                                                       t = m_registration->match<Point>(m_rep1->cloud(), m_rep0->cloud(), t1, t0);
																													} else {
	                                                                                       t = m_registration->match<Point, OpenMesh::Vec4f>(m_rep1->cloud(), *(m_rep0->mesh()), t1, t0);
																													}
																												} else {
	                                                                                    if (m_rep0->isCloud()) {
	                                                                                       t = m_registration->match<OpenMesh::Vec4f, Point>(*(m_rep1->mesh()), m_rep0->cloud(), t1, t0);
																													} else {
	                                                                                       t = m_registration->match<OpenMesh::Vec4f>(*(m_rep1->mesh()), *(m_rep0->mesh()), t1, t0);
																													}
																												}
	                                                                                 m_rep1->transform(t);
	                                                                                 gui()->status()->set("Done computing alignment transformation.");
																											});

	auto  registrationAdvancedSection = registrationSection->add<Section>("Advanced settings", "Advanced");
	registrationAdvancedSection->add<Number>("Max. corresp. distance", "MaxCorrespondenceDistance")->setMin(0.0001).setMax(100.0).setDigits(4).setValue(0.01);
	registrationAdvancedSection->add<Number>("Transformation epsilon", "TransformationEpsilon")->setMin(0.00001).setMax(100.0).setDigits(5).setValue(0.00001);
	registrationAdvancedSection->add<Number>("Euclidean fitness epsilon", "EuclideanFitnessEpsilon")->setMin(0.00001).setMax(100.0).setDigits(5).setValue(0.00001);
	registrationAdvancedSection->add<Number>("Maximum iterations", "MaximumIterations")->setMin(1).setMax(1000).setDigits(0).setValue(50);
	registrationAdvancedSection->collapse();

	if ((m_rep0->isIFC() && m_rep1->isCloud()) || (m_rep0->isCloud() && m_rep1->isIFC())) {
		auto  differenceSection = gui()->properties()->add<Section>("Difference Detection", "diff_detect");
		differenceSection->add<Number>("Observed Point Threshold", "ratio_threshold")->setMin(0.0).setMax(1.0).setDigits(3).setValue(0.7);
		differenceSection->add<Number>("Min Points Per Square Unit Area", "points_per_square")->setMin(0.001).setMax(1000000).setDigits(3).setValue(1.0);
		differenceSection->add<Number>("Distance Epsilon", "dist_eps")->setMin(0.00001).setMax(100.0).setDigits(5).setValue(0.1);
        auto diffCol = differenceSection->add<Group>("Colors", "colors");
        diffCol->add<Color>("New Points", "new_points")->setValue(Eigen::Vector4f(0.2f, 0.2f, 0.8f, 1.f));
        diffCol->add<Color>("Missing IFC Parts", "missing_parts")->setValue(Eigen::Vector4f(1.f, 0.f, 0.f, 1.f));
        diffCol->add<Color>("Found IFC Parts", "found_parts")->setValue(Eigen::Vector4f(1.f, 1.f, 1.f, 1.f));
		differenceSection->add<Button>("Detect", "Detect")->setCallback([&] () {
		                                                                              gui()->status()->set("Detecting differences...");
		                                                                   std::vector<int32_t> associations;
		                                                                   float ratio_threshold = gui()->properties()->get<Number>({"diff_detect", "ratio_threshold"})->value();
		                                                                   float points_per_square = gui()->properties()->get<Number>({"diff_detect", "points_per_square"})->value();
		                                                                   float eps = gui()->properties()->get<Number>({"diff_detect", "dist_eps"})->value();
		                                                                   std::vector<uint32_t> possible_hits, actual_hits;

		                                                                   Eigen::Affine3f t0, t1;
		                                                                   t0 = m_rep0->transformation();
		                                                                   t1 = m_rep1->transformation();
		                                                                   if (m_rep0->isIFC()) {
                                                                              possible_hits.resize(m_rep1->cloud()->size(), 0);
                                                                              actual_hits.resize(m_rep1->cloud()->size(), 0);
                                                                              associations = duraark::associate_points<Point, OpenMesh::Vec4f>(m_rep0->objects(), m_rep1->cloud(), eps, possible_hits, actual_hits, m_rep1->cloudOrigins(), m_rep1->cloudSizes(), t0, t1);
                                                                           } else {
                                                                              possible_hits.resize(m_rep0->cloud()->size(), 0);
                                                                              actual_hits.resize(m_rep0->cloud()->size(), 0);
                                                                              associations = duraark::associate_points<Point, OpenMesh::Vec4f>(m_rep0->cloud(), m_rep1->objects(), eps, possible_hits, actual_hits, m_rep0->cloudOrigins(), m_rep0->cloudSizes(), t0, t1);
                                                                           }
                                                                           gui()->status()->set("Done detecting differences.");
                                                                           gui()->log()->info("Found matching objects: " + std::to_string(associations.size()));

                                                                           m_possibleHits = possible_hits;
                                                                           m_actualHits = actual_hits;
                                                                           m_associations = associations;

                                                                           Eigen::Vector4f np_col = gui()->properties()->get<Color>({"diff_detect", "colors", "new_points"})->value();
                                                                           Eigen::Vector4f mp_col = gui()->properties()->get<Color>({"diff_detect", "colors", "missing_parts"})->value();
                                                                           Eigen::Vector4f fp_col = gui()->properties()->get<Color>({"diff_detect", "colors", "found_parts"})->value();
		                                                                   m_rep0->annotate(associations, possible_hits, actual_hits, ratio_threshold, points_per_square, this, np_col, mp_col, fp_col);
		                                                                   m_rep1->annotate(associations, possible_hits, actual_hits, ratio_threshold, points_per_square, this, np_col, mp_col, fp_col);

                                                                           auto diffCol = gui()->properties()->get<Group>({"diff_detect", "colors"});
                                                                           bool has_button = true;
                                                                           try {
                                                                                diffCol->get<Button>({"update"});
                                                                           } catch(...) {
                                                                               has_button = false;
                                                                           }
                                                                           if (!has_button) {
                                                                               diffCol->add<Button>("Update Colors", "update")->setCallback([&] () {
                                                                                    Eigen::Vector4f np_col = gui()->properties()->get<Color>({"diff_detect", "colors", "new_points"})->value();
                                                                                    Eigen::Vector4f mp_col = gui()->properties()->get<Color>({"diff_detect", "colors", "missing_parts"})->value();
                                                                                    Eigen::Vector4f fp_col = gui()->properties()->get<Color>({"diff_detect", "colors", "found_parts"})->value();
                                                                                    float ratio_threshold = gui()->properties()->get<Number>({"diff_detect", "ratio_threshold"})->value();
                                                                                    float points_per_square = gui()->properties()->get<Number>({"diff_detect", "points_per_square"})->value();
                                                                                    m_rep0->annotate(m_associations, m_possibleHits, m_actualHits, ratio_threshold, points_per_square, this, np_col, mp_col, fp_col);
                                                                                    m_rep1->annotate(m_associations, m_possibleHits, m_actualHits, ratio_threshold, points_per_square, this, np_col, mp_col, fp_col);
                                                                               });
                                                                           }

		});
		// differenceSection->add<Button>("Add pos. from camera")->setCallback(
		// [&] () {
		// m_origin = std::make_shared<Rendered::Point>(fw()->transforms()->cameraPosition(), Colors::rgbaWhite(), 2);
		// }
		// );
	}
}

void Registration::registerEvents() {
	fw()->events()->connect<void (int, int, int, int)>("LEFT_DRAG", [&] (int dx, int dy, int, int) {
        bool transformingA = gui()->modes()->group("TransformGroup")->option("TransformA")->active();
        bool transformingB = gui()->modes()->group("TransformGroup")->option("TransformB")->active();
        bool clipping = gui()->modes()->group("TransformGroup")->option("Clip")->active();

        if (clipping) {
            for (auto& r: m_rep0->renderables()) {
                r->delta_clipping_height(-dy * 0.01f);
            }
            for (auto& r: m_rep1->renderables()) {
                r->delta_clipping_height(-dy * 0.01f);
            }
            if (m_rep0->isIFC() && m_rep0->group()) m_rep0->group()->delta_clipping_height(-dy * 0.01f);
            if (m_rep1->isIFC() && m_rep1->group()) m_rep1->group()->delta_clipping_height(-dy * 0.01f);
        }

        if (transformingA || transformingB) {
            Eigen::Affine3f trafo;

            if (!fw()->modifier()->shift()) {
                auto mv = fw()->camera()->view_matrix();
                Eigen::Vector3f right = mv.row(0).head(3);
                Eigen::Vector3f up = mv.row(1).head(3);
                float factor = fw()->modifier()->ctrl() ? 0.005f : 0.05f;
                Eigen::Vector3f delta = factor * dx * right - factor * dy * up;

                trafo = Eigen::Translation<float, 3>(delta);
            } else {
                trafo = Eigen::AngleAxis<float>(0.02f * dx, fw()->camera()->forward());
            }

            if (transformingA) {
                m_rep0->transform(trafo);
            } else if (transformingB) {
                m_rep1->transform(trafo);
            }
        }
    });
}

void Registration::setICPParameters_() {
	float  max_corr  = gui()->properties()->get<Number>({"Registration", "Advanced", "MaxCorrespondenceDistance"})->value();
	float  trans_eps = gui()->properties()->get<Number>({"Registration", "Advanced", "TransformationEpsilon"})->value();
	float  eucl_eps  = gui()->properties()->get<Number>({"Registration", "Advanced", "EuclideanFitnessEpsilon"})->value();
	int    max_iter  = static_cast<int>(gui()->properties()->get<Number>({"Registration", "Advanced", "MaximumIterations"})->value());
	m_registration->set_max_correspondence_distance(max_corr);
	m_registration->set_transformation_epsilon(trans_eps);
	m_registration->set_euclidean_fitness_epsilon(eucl_eps);
	m_registration->set_maximum_iterations(max_iter);
}

Registration::Factory::Factory() : FW::Factory() {
}

Registration::Factory::~Factory() {
}

void Registration::Factory::init() {
	gui()->properties()->add<Files>("Input File A", "path0")->setExtensions({"ifc", "obj", "pcd", "e57"});
	gui()->properties()->add<Files>("Input File B", "path1")->setExtensions({"ifc", "obj", "pcd", "e57"});
}

Visualizer::Ptr Registration::Factory::addVisualizer() {
	std::string        name   = gui()->properties()->get<String>({"__name__"})->value();
	auto               paths0 = gui()->properties()->get<Files>({"path0"})->value();
	auto               paths1 = gui()->properties()->get<Files>({"path1"})->value();
	Registration::Ptr  vis(new Registration(name, paths0, paths1));
	return std::dynamic_pointer_cast<Visualizer>(vis);
}

Registration::Representation::Representation(const std::vector<fs::path>& paths) : m_paths(paths) {
	const auto& path = paths[0];
	if (path.extension() == ".pcd" || path.extension() == ".e57") {
		m_cloudSizes.clear();
		if (!m_cloud) m_cloud = Cloud::Ptr(new Cloud());
		for (const auto& p : paths) {
			auto  tmp_cloud = Geometry::PCLTools<Point>::loadPointCloud(p);
			if (!tmp_cloud) {
				throw std::runtime_error("Could not open point cloud file \"" + p.string() + "\"");
			}
			m_cloud->insert(m_cloud->end(), tmp_cloud->begin(), tmp_cloud->end());
			m_cloudSizes.push_back(tmp_cloud->size());
			m_cloudOrigins.push_back(tmp_cloud->sensor_origin_.head(3));
		}
		harmont::pointcloud_object<Cloud, boost::shared_ptr>::ptr_t  cloud_obj(new harmont::pointcloud_object<Cloud, boost::shared_ptr>(m_cloud));
		cloud_obj->init();
		m_renderables.push_back(cloud_obj);
	} else if (path.extension() == ".obj") {
		m_mesh = MeshPtr(new Mesh());
		if (!MeshTraits::read(*m_mesh, path.string())) {
			throw std::runtime_error("Could not open mesh file \"" + path.string() + "\"");
		}
		m_mesh->request_vertex_normals();
		m_mesh->request_face_normals();
		m_mesh->update_face_normals();
		m_mesh->update_normals();
		harmont::mesh_object<Mesh>::ptr_t  mesh_obj(new harmont::mesh_object<Mesh>(m_mesh, false));
		mesh_obj->init();
		m_renderables.push_back(mesh_obj);
	} else if (path.extension() == ".ifc") {
		m_objects = duraark::extract_objects<ColorType>(path);
		m_mesh    = MeshPtr(new Mesh());
		*m_mesh   = duraark::extract_single_mesh<ColorType>(m_objects, &m_vertexMap);
		for (auto it = m_mesh->vertices_begin(); it != m_mesh->vertices_end(); ++it) {
			m_mesh->set_color(*it, Mesh::Color(1.f, 1.f, 1.f, 1.f));
		}
		harmont::mesh_object<Mesh>::ptr_t  mesh_obj(new harmont::mesh_object<Mesh>(m_mesh, false));
		mesh_obj->init();
		m_renderables.push_back(mesh_obj);
	} else {
		throw std::runtime_error("Unknown file type");
	}
}

Registration::Representation::~Representation() {
}

Registration::Representation::Renderables& Registration::Representation::renderables() {
	return m_renderables;
}

const Registration::Representation::Renderables& Registration::Representation::renderables() const {
	return m_renderables;
}

harmont::renderable_group::ptr_t Registration::Representation::group() {
    return m_group;
}

harmont::renderable_group::const_ptr_t Registration::Representation::group() const {
    return m_group;
}

void Registration::Representation::center() {
	Eigen::Vector3f  centroid = Eigen::Vector3f::Zero();
	if (isMesh() || isIFC()) {
		centroid = cartan::mesh_algorithms<Mesh>::centroid(*m_mesh);
	} else if (isCloud()) {
		Eigen::Vector4f  c;
		pcl::compute3DCentroid(*m_cloud, c);
		centroid = c.head(3);
	}
	Eigen::Matrix4f  trafo = Eigen::Matrix4f::Identity();
	trafo.block<3, 1>(0, 3) = -centroid;
	for (auto& r : m_renderables) {
		r->move(trafo);
	}
}

bool Registration::Representation::isCloud() const {
	return (bool)m_cloud;
}

bool Registration::Representation::isMesh() const {
	return (bool)m_mesh;
}

bool Registration::Representation::isIFC() const {
	return (bool)(m_objects.size());
}

Cloud::Ptr Registration::Representation::cloud() {
	return m_cloud;
}

Registration::Representation::MeshPtr Registration::Representation::mesh() {
	return m_mesh;
}

duraark::ifc_objects_t<Registration::Representation::ColorType>& Registration::Representation::objects() {
	return m_objects;
}

const duraark::ifc_objects_t<Registration::Representation::ColorType>& Registration::Representation::objects() const {
	return m_objects;
}

const std::vector<uint32_t>& Registration::Representation::cloudSizes() const {
	return m_cloudSizes;
}

const std::vector<Eigen::Vector3f>& Registration::Representation::cloudOrigins() const {
	return m_cloudOrigins;
}

Registration::Representation::TransformType Registration::Representation::transformation() const {
	TransformType  t;
	t = m_renderables[0]->transformation();
	return t;
}

void Registration::Representation::transform(const TransformType& t) {
	for (auto& r : m_renderables) {
		r->move(t.matrix());
	}
}

void Registration::Representation::annotate(const std::vector<int32_t>& associations, const std::vector<uint32_t>& possible_hits, const std::vector<uint32_t>& actual_hits, float ratio_threshold, float points_per_square, Visualizer* vis, const Eigen::Vector4f& np_col, const Eigen::Vector4f& mp_col, const Eigen::Vector4f fp_col) {
    if (m_cloud) {
        m_renderables[0]->set_active(false);
        if (m_renderables.size() > 1) {
            //vis->removeObject("cloud_annot_a");
            vis->removeObject("cloud_annot_b");
            m_renderables.erase(m_renderables.begin() + 1, m_renderables.end());
        }
        auto assoc = duraark::to_object_map(associations, 0);
        std::set<uint32_t> associated, iota;
        for (uint32_t i = 0; i < m_cloud->size(); ++i) {
            iota.insert(i);
        }
        for (const auto& a : assoc) {
            associated.insert(a.second.begin(), a.second.end());
        }
        std::vector<uint32_t> extra;
        std::set_difference(iota.begin(), iota.end(), associated.begin(), associated.end(), std::back_inserter(extra));
        std::vector<int> a(associated.begin(), associated.end()), b(extra.begin(), extra.end());

        //Cloud::Ptr cloudA(new Cloud(*m_cloud, a));
		//harmont::pointcloud_object<Cloud, boost::shared_ptr>::ptr_t  cloudObjA(new harmont::pointcloud_object<Cloud, boost::shared_ptr>(cloudA));
		//cloudObjA->init();
        //cloudObjA->set_transformation(m_renderables.front()->transformation());
        //cloudObjA->set_point_colors(Eigen::Vector4f(1.f, 1.f, 1.f, 0.f));
        //cloudObjA->set_clipping(m_renderables.front()->clipping());
        //cloudObjA->set_clipping_height(m_renderables.front()->clipping_height());
		//m_renderables.push_back(cloudObjA);
        //vis->addObject("cloud_annot_a", cloudObjA);

        Cloud::Ptr cloudB(new Cloud(*m_cloud, b));
		harmont::pointcloud_object<Cloud, boost::shared_ptr>::ptr_t  cloudObjB(new harmont::pointcloud_object<Cloud, boost::shared_ptr>(cloudB));
		cloudObjB->init();
        cloudObjB->set_transformation(m_renderables.front()->transformation());
        cloudObjB->set_point_colors(np_col);
        cloudObjB->set_clipping(m_renderables.front()->clipping());
        cloudObjB->set_clipping_height(m_renderables.front()->clipping_height());
		m_renderables.push_back(cloudObjB);
        vis->addObject("cloud_annot_b", cloudObjB);
    }
    if (m_objects.size()) {
        m_renderables[0]->set_active(false);
        if (m_group) {
            vis->removeObjectGroup("annot_");
            m_group.reset();
        }
        uint32_t debug_count = 0;

        std::vector<harmont::renderable::ptr_t> group;
        for (uint32_t i = 0; i < m_objects.size(); ++i) {
            MeshPtr mesh(new Mesh());
            *mesh = std::get<0>(m_objects[i]);

            // compute surface area
            float area = 0.f;
            for (auto face_iter = mesh->faces_begin(); face_iter != mesh->faces_end(); ++face_iter) {
                std::vector<Eigen::Vector3f> vertices;
                for (auto fv_iter = mesh->cfv_iter(*face_iter); fv_iter; ++fv_iter) {
                    vertices.push_back(Eigen::Vector3f(mesh->point(*fv_iter).data()));
                }
                float face_area = 0.5f * (vertices[1] - vertices[0]).cross(vertices[2]-vertices[0]).norm();
                area += face_area;
            }
            Mesh::Color color(mp_col.data());
            if (static_cast<float>(actual_hits[i]) / possible_hits[i] >= ratio_threshold && static_cast<float>(possible_hits[i]) / area >= points_per_square) {
            //if (possible_hits[i]) {
                color = Mesh::Color(fp_col.data());
                ++debug_count;
            } else {
                //if (static_cast<float>(actual_hits[i]) / possible_hits[i] < ratio_threshold) std::cout << "ratio_threshold" << "\n";
                //if (static_cast<float>(possible_hits[i]) / area < points_per_square) std::cout << "area (" << (static_cast<float>(possible_hits[i]) / area) << " / " << points_per_square << ")\n";
            }

            for (auto it = mesh->vertices_begin(); it != mesh->vertices_end(); ++it) {
                mesh->set_color(*it, color);
            }
            harmont::mesh_object<Mesh>::ptr_t mesh_obj(new harmont::mesh_object<Mesh>(mesh, false));
            mesh_obj->init();
            mesh_obj->set_transformation(m_renderables[0]->transformation());
            group.push_back(mesh_obj);
        }
        m_group = std::make_shared<harmont::renderable_group>(group);
        m_group->set_clipping(m_renderables.front()->clipping());
        m_group->set_clipping_height(m_renderables.front()->clipping_height());
        vis->addObjectGroup("annot_", m_group);
        std::cout << "associated " << debug_count << " of " << m_objects.size() << " mesh objects." << "\n";
     }
}

} // FW


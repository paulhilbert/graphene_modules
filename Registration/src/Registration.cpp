#include "Registration.h"

#include <Library/Colors/Generation.h>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Mesh/Traits.hh>
#include <OpenMesh/Core/Utils/color_cast.hh>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

//#include <duraark/turtle_input.hpp>
//#include <duraark/extract_objects.hpp>
//#include <duraark/pair_difference.hpp>
//#include <duraark/association.hpp>

#include <Eigen/Geometry>
#include <Geometry/PCLTools.h>

#include <cartan/pointcloud_tools.hpp>

#include <ifc_mesh_extract/extract_objects.hpp>

#include <duraark_rdf/entity.hpp>
#include <duraark_rdf/turtle_input_helper.hpp>
#include <duraark_rdf/turtle_output_helper.hpp>
using namespace duraark_rdf;
#include <duraark_assoc/associate_points.hpp>
#include <duraark_assoc/utils.hpp>

#include <e57_pcl/read.hpp>
#include <ifc_mesh_extract/extract_objects.hpp>

#include <duraark_vis/renderable_adapter.hpp>
using duraark_vis;

using namespace GUI::Property;


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
		Representation(const std::vector<fs::path>& paths, const Eigen::Vector4f& color);
		virtual ~Representation();

		void center();

        const std::vector<fs::path>& paths() const;

        std::string guid() const;

		bool isCloud() const;
		bool isMesh() const;
		bool isIFC() const;

		Renderables&       renderables();
		const Renderables& renderables() const;
        harmont::crosshair_object::ptr_t renderedOrigins();
        harmont::renderable_group::ptr_t group();
        harmont::renderable_group::const_ptr_t group() const;
		Cloud::Ptr cloud();
		MeshPtr mesh();
		ifc_mesh_extract::ifc_objects_t<ColorType>& objects();
		const ifc_mesh_extract::ifc_objects_t<ColorType>& objects() const;
		const std::vector<uint32_t>& cloudSizes() const;
		const std::vector<Eigen::Vector3f>& cloudOrigins() const;

        std::vector<std::vector<int>> splitToScans(const std::vector<int32_t>& associations) const;

		TransformType transformation() const;
		void transform(const TransformType& t);

		void annotate(const std::vector<int32_t>& associations, const std::vector<uint32_t>& possible_hits, const std::vector<uint32_t>& actual_hits, float ratio_threshold, float points_per_square, Visualizer* vis, const Eigen::Vector4f& np_col, const Eigen::Vector4f& mp_col, const Eigen::Vector4f fp_col);

	protected:
		std::vector<fs::path>              m_paths;
		Renderables                        m_renderables;
        std::string                        m_guid;
		MeshPtr                            m_mesh;
		Cloud::Ptr                         m_cloud;
		ifc_mesh_extract::ifc_objects_t<ColorType>  m_objects;
		std::vector<std::vector<int>>      m_vertexMap;
		std::vector<uint32_t>              m_cloudSizes;
		std::vector<Eigen::Vector3f>       m_cloudOrigins;
        harmont::renderable_group::ptr_t   m_group;
        harmont::crosshair_object::ptr_t   m_renderedOrigins;
};


Registration::Registration(std::string id, const std::vector<fs::path>& paths0, const std::vector<fs::path>& paths1, const Eigen::Vector4f& col0, const Eigen::Vector4f& col1) : Visualizer(id), m_paths0(paths0), m_paths1(paths1), m_col0(col0), m_col1(col1) {
}

Registration::~Registration() {
}

void Registration::init() {
	try {
		m_rep0 = std::make_shared<Representation>(m_paths0, m_col0);
	} catch (std::runtime_error& error) {
		gui()->log()->error(std::string("Could not load first representation: ") + error.what());
        return;
	}
	try {
		m_rep1 = std::make_shared<Representation>(m_paths1, m_col1);
	} catch (std::runtime_error& error) {
		gui()->log()->error(std::string("Could not load second representation: ") + error.what());
        return;
	}

    //m_rep0->center();
    //m_rep1->center();
	uint32_t  idx = 0;
	for (auto& r : m_rep0->renderables()) {
		addObject("rep0_" + std::to_string(idx++), r);
	}
	idx = 0;
	for (auto& r : m_rep1->renderables()) {
		addObject("rep1_" + std::to_string(idx++), r);
	}

    auto ch0 = m_rep0->renderedOrigins(), ch1 = m_rep1->renderedOrigins();
    if (ch0) {
        addObject("ch0", ch0);
    }
    if (ch1) {
        addObject("ch1", ch1);
    }

	addProperties();
	registerEvents();

	//m_registration = std::make_shared<duraark::registration>();
}

void Registration::addProperties() {
	auto  showGroup = gui()->modes()->addGroup("showGroup");
	auto  editGroup = gui()->modes()->addGroup("editGroup");

	showGroup->addOption("ShowClip", "Enable Clipping", std::string(ICON_PREFIX) + "clipping.png");

	editGroup->addOption("TransformA", "Transform A", std::string(ICON_PREFIX) + "transform_a_128.png");
	editGroup->addOption("EditClipA", "Modify Clipping of A", std::string(ICON_PREFIX) + "clipplane.png");
	editGroup->addOption("TransformB", "Transform B", std::string(ICON_PREFIX) + "transform_b_128.png");
	editGroup->addOption("EditClipB", "Modify Clipping of B", std::string(ICON_PREFIX) + "clipplane.png");

	showGroup->setCallback([&] (std::string option, bool state) {
        if (option == "ShowClip") {
            for (auto& r: m_rep0->renderables()) {
                r->set_clipping(state);
            }
            if (m_rep0->isIFC()) {
                auto grp = m_rep0->group();
                if (grp) grp->set_clipping(state);
            }
            for (auto& r: m_rep1->renderables()) {
                r->set_clipping(state);
            }
            if (m_rep1->isIFC()) {
                auto grp = m_rep1->group();
                if (grp) grp->set_clipping(state);
            }
        }
    });

    auto save_reg = gui()->properties()->add<File>("Save Registration", "save_reg");
    save_reg->setMode(File::SAVE);
    save_reg->setExtensions({"rdf"});
    save_reg->setCallback([&] (const fs::path& path) {
        Eigen::Matrix4f t = (m_rep1->transformation().inverse() * m_rep0->transformation()).matrix();

        entity::type_t t0 = m_rep0->isIFC() ? entity::type_t::IFC : entity::type_t::PC;
        entity::type_t t1 = m_rep1->isIFC() ? entity::type_t::IFC : entity::type_t::PC;
        auto e0 = std::make_shared<entity>(t0, m_rep0->paths()[0].stem().string(), m_rep0->guid());
        auto e1 = std::make_shared<entity>(t1, m_rep1->paths()[0].stem().string(), m_rep1->guid());

        duraark_rdf::turtle_output turtle(path.string());
        write_prologue(turtle);
        write_entity(turtle, *e0);
        write_entity(turtle, *e1);
        write_registration(turtle, *e0, *e1, t);
    });

    auto load_reg = gui()->properties()->add<File>("Load Registration", "load_reg");
    load_reg->setExtensions({"rdf"});
    load_reg->setCallback([&] (const fs::path& path) {
        Eigen::Affine3f t0 = m_rep0->transformation();
        Eigen::Affine3f t1 = m_rep1->transformation();
        m_rep0->transform(t0.inverse());
        m_rep1->transform(t1.inverse());

        duraark_rdf::turtle_input input(path.string());
        Eigen::Matrix4f m = duraark_rdf::parse_registration(input, m_rep0->guid(), m_rep1->guid());
        Eigen::Affine3f t;
        t = m;

        m_rep0->transform(t0 * t);
        m_rep1->transform(t0);
    });

	if ((m_rep0->isIFC() && m_rep1->isCloud()) || (m_rep0->isCloud() && m_rep1->isIFC())) {
        auto  differenceSection = gui()->properties()->add<Section>("Associations", "diff_detect");
		differenceSection->add<Number>("Epsilon", "dist_eps")->setMin(0.00001).setMax(100.0).setDigits(5).setValue(0.1);
		differenceSection->add<Number>("Rho", "ratio_threshold")->setMin(0.0).setMax(1.0).setDigits(3).setValue(0.4);
		differenceSection->add<Number>("Points Per Area", "points_per_square")->setMin(0.001).setMax(1000000).setDigits(3).setValue(1.0);

		//differenceSection->add<Bool>("Ignore Doors", "ignore_doors")->setValue(true);
		//differenceSection->add<Bool>("Ignore Windows", "ignore_windows")->setValue(false);

        auto load_assoc = differenceSection->add<File>("Load Associations", "load_assoc");
        load_assoc->setMode(File::OPEN);
        load_assoc->setCallback([&](const fs::path &p) {
            duraark_rdf::turtle_input input(p.string());
            std::set<std::string> entity_guids;
            std::map<std::string, uint32_t> possible_hits;
            auto assocs = parse_subset_associations(input, possible_hits, &entity_guids);

            auto cloud = m_rep0->isCloud() ? m_rep0->cloud() : m_rep1->cloud();
            const auto& objs = m_rep0->isIFC() ? m_rep0->objects() : m_rep1->objects();

            std::map<std::string, uint32_t> object_map;
            for (uint32_t i = 0; i < objs.size(); ++i) {
                object_map[std::get<1>(objs[i])] = i;
            }

            m_associations.resize(cloud->size(), -1);
            m_possibleHits.resize(objs.size());
            m_actualHits.resize(objs.size());

            for (const auto& assoc : assocs) {
                uint32_t obj_idx = object_map[assoc.ifc_object_guid];
                m_actualHits[obj_idx] = assoc.point_indices.size();
                for (const auto& idx : assoc.point_indices) {
                    m_associations[idx] = obj_idx;
                }
            }

            for (const auto& poss : possible_hits) {
                uint32_t obj_idx = object_map[poss.first];
                m_possibleHits[obj_idx] = poss.second;
            }

            gui()->properties()->get<Button>({"diff_detect", "detect"})->enable();
            gui()->properties()->get<File>({"diff_detect", "save_assoc"})->enable();
            gui()->properties()->get<Group>({"diff_detect", "colors"})->enable();
        });

                differenceSection->add<Button>("Compute Associations", "associate")->setCallback([&] () {
            gui()->status()->set("Computing Associations...");
            std::vector<int32_t> associations;
            float eps = gui()->properties()->get<Number>({"diff_detect", "dist_eps"})->value();
            std::vector<uint32_t> possible_hits, actual_hits;

            std::set<std::string> ignored_entity_types;
            //if (gui()->properties()->get<Bool>({"diff_detect", "ignore_doors"})->value()) ignored_entity_types.insert("ifcdoor");
            //if (gui()->properties()->get<Bool>({"diff_detect", "ignore_windows"})->value()) ignored_entity_types.insert("ifcwindow");
            ignored_entity_types.insert("ifcdoor");

            Eigen::Affine3f t0, t1;
            t0 = m_rep0->transformation();
            t1 = m_rep1->transformation();
            if (m_rep0->isIFC()) {
               possible_hits.resize(m_rep0->objects().size(), 0);
               actual_hits.resize(m_rep0->objects().size(), 0);
               associations = duraark_assoc::associate_points<Point, OpenMesh::Vec4f>(m_rep0->objects(), m_rep1->cloud(), eps, possible_hits, actual_hits, m_rep1->cloudOrigins(), m_rep1->cloudSizes(), t0, t1, ignored_entity_types);
            } else {
               possible_hits.resize(m_rep1->objects().size(), 0);
               actual_hits.resize(m_rep1->objects().size(), 0);
               associations = duraark_assoc::associate_points<Point, OpenMesh::Vec4f>(m_rep0->cloud(), m_rep1->objects(), eps, possible_hits, actual_hits, m_rep0->cloudOrigins(), m_rep0->cloudSizes(), t0, t1, ignored_entity_types);
            }
            gui()->status()->set("Done computing associations.");
            gui()->log()->info("Found matching objects: " + std::to_string(associations.size()));

            m_possibleHits = possible_hits;
            m_actualHits = actual_hits;
            m_associations = associations;

            gui()->properties()->get<Button>({"diff_detect", "detect"})->enable();
            gui()->properties()->get<File>({"diff_detect", "save_assoc"})->enable();
            gui()->properties()->get<Group>({"diff_detect", "colors"})->enable();
        });

        auto save_assoc = differenceSection->add<File>("Save Associations", "save_assoc");
        save_assoc->setMode(File::SAVE);
        save_assoc->setExtensions({"rdf"});
        save_assoc->setCallback([&] (const fs::path& p) {
            auto cloud_rep = m_rep0->isCloud() ? m_rep0 : m_rep1;
            auto ifc_rep = m_rep0->isIFC() ? m_rep0 : m_rep1;

            auto& objects = ifc_rep->objects();

            fs::path cloud_path = m_rep0->isCloud() ? m_paths0[0] : m_paths1[0];
            entity::sptr_t cloud_entity = std::make_shared<entity>(entity::type_t::PC, cloud_path.stem().string(), cloud_rep->guid());

            duraark_rdf::turtle_output turtle(p.string());
            write_prologue(turtle);
            write_entity(turtle, *cloud_entity);
            auto obj_map = duraark_assoc::to_object_map(m_associations, 1);
            for (const auto& m : obj_map) {
                entity sub_ent(cloud_entity, m.second, std::get<1>(objects[m.first]));
                write_subset_entity(turtle, sub_ent, m_possibleHits[m.first]);
            }
        });
        save_assoc->disable();

        auto diffCol = differenceSection->add<Group>("Difference Colors", "colors");
        diffCol->add<Color>("New Points", "new_points")->setValue(Eigen::Vector4f(0.2f, 0.2f, 0.8f, 1.f));
        diffCol->add<Color>("Missing IFC Parts", "missing_parts")->setValue(Eigen::Vector4f(1.f, 0.f, 0.f, 1.f));
        diffCol->add<Color>("Found IFC Parts", "found_parts")->setValue(Eigen::Vector4f(1.f, 1.f, 1.f, 1.f));
        diffCol->disable();

		differenceSection->add<Button>("Detect Differences", "detect")->setCallback([&] () {
            float ratio_threshold = gui()->properties()->get<Number>({"diff_detect", "ratio_threshold"})->value();
            float points_per_square = gui()->properties()->get<Number>({"diff_detect", "points_per_square"})->value();
            Eigen::Vector4f np_col = gui()->properties()->get<Color>({"diff_detect", "colors", "new_points"})->value();
            Eigen::Vector4f mp_col = gui()->properties()->get<Color>({"diff_detect", "colors", "missing_parts"})->value();
            Eigen::Vector4f fp_col = gui()->properties()->get<Color>({"diff_detect", "colors", "found_parts"})->value();
            m_rep0->annotate(m_associations, m_possibleHits, m_actualHits, ratio_threshold, points_per_square, this, np_col, mp_col, fp_col);
            m_rep1->annotate(m_associations, m_possibleHits, m_actualHits, ratio_threshold, points_per_square, this, np_col, mp_col, fp_col);

            auto diffCol = gui()->properties()->get<Group>({"diff_detect", "colors"});
            bool has_button = true;
            try {
                 diffCol->get<Button>({"update"});
            } catch(...) {
                has_button = false;
            }
            if (!has_button) {
                diffCol->add<Button>("Change Colors", "update")->setCallback([&] () {
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
        gui()->properties()->get<Button>({"diff_detect", "detect"})->disable();


		// differenceSection->add<Button>("Add pos. from camera")->setCallback(
		// [&] () {
		// m_origin = std::make_shared<Rendered::Point>(fw()->transforms()->cameraPosition(), Colors::rgbaWhite(), 2);
		// }
		// );
	}
}

void Registration::registerEvents() {
	fw()->events()->connect<void (int, int, int, int)>("LEFT_DRAG", [&] (int dx, int dy, int, int) {
        bool transformingA = gui()->modes()->group("editGroup")->option("TransformA")->active();
        bool transformingB = gui()->modes()->group("editGroup")->option("TransformB")->active();
        bool clippingA = gui()->modes()->group("editGroup")->option("EditClipA")->active();
        bool clippingB = gui()->modes()->group("editGroup")->option("EditClipB")->active();

        if (clippingA) {
            for (auto& r: m_rep0->renderables()) {
                float height = r->bounding_box().max()[2] - r->bounding_box().min()[2];
                r->delta_clipping_height((-dy / 40.f) / height);
            }
            if (m_rep0->isIFC() && m_rep0->group()) m_rep0->group()->delta_clipping_height(-dy * 0.01f);
        }

        if (clippingB) {
            for (auto& r: m_rep1->renderables()) {
                float height = r->bounding_box().max()[2] - r->bounding_box().min()[2];
                r->delta_clipping_height((-dy / 40.f) / height);
            }
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

//void Registration::setICPParameters_() {
	//float  max_corr  = gui()->properties()->get<Number>({"Registration", "Advanced", "MaxCorrespondenceDistance"})->value();
	//float  trans_eps = gui()->properties()->get<Number>({"Registration", "Advanced", "TransformationEpsilon"})->value();
	//float  eucl_eps  = gui()->properties()->get<Number>({"Registration", "Advanced", "EuclideanFitnessEpsilon"})->value();
	//int    max_iter  = static_cast<int>(gui()->properties()->get<Number>({"Registration", "Advanced", "MaximumIterations"})->value());
	//m_registration->set_max_correspondence_distance(max_corr);
	//m_registration->set_transformation_epsilon(trans_eps);
	//m_registration->set_euclidean_fitness_epsilon(eucl_eps);
	//m_registration->set_maximum_iterations(max_iter);
//}

Registration::Factory::Factory() : FW::Factory() {
}

Registration::Factory::~Factory() {
}

void Registration::Factory::init() {
	gui()->properties()->add<Files>("Input File A", "path0")->setExtensions({"ifc", "obj", "pcd", "e57"});
	gui()->properties()->add<Files>("Input File B", "path1")->setExtensions({"ifc", "obj", "pcd", "e57"});

    auto diffCol = gui()->properties()->add<Group>("Colors", "colors");
    diffCol->add<Color>("Color A", "col_a")->setValue(Eigen::Vector4f(1.f, 1.f, 1.f, 1.f));
    diffCol->add<Color>("Color B", "col_b")->setValue(Eigen::Vector4f(1.f, 1.f, 1.f, 1.f));
}

Visualizer::Ptr Registration::Factory::addVisualizer() {
	std::string        name   = gui()->properties()->get<String>({"__name__"})->value();
	auto               paths0 = gui()->properties()->get<Files>({"path0"})->value();
	auto               paths1 = gui()->properties()->get<Files>({"path1"})->value();
    auto               col0   = gui()->properties()->get<Color>({"colors", "col_a"})->value();
    auto               col1   = gui()->properties()->get<Color>({"colors", "col_b"})->value();
	Registration::Ptr  vis(new Registration(name, paths0, paths1, col0, col1));
	return std::dynamic_pointer_cast<Visualizer>(vis);
}

Registration::Representation::Representation(const std::vector<fs::path>& paths, const Eigen::Vector4f& color) : m_paths(paths) {
	const auto& path = paths[0];
    m_guid = std::to_string(std::hash<std::string>()(path.string()));
	if (path.extension() == ".e57n") {
        m_cloudSizes.clear();
        auto clouds = e57_pcl::load_e57_scans_with_normals(path.string(), m_guid);
        if (!m_cloud) m_cloud = Cloud::Ptr(new Cloud());
        for (auto c : clouds) {
            m_cloud->insert(m_cloud->end(), c->begin(), c->end());
            m_cloudSizes.push_back(c->size());
            m_cloudOrigins.push_back(c->sensor_origin_.head(3));
        }
        auto cloud_obj = duraark_vis::make_renderable(m_cloud, color);
        //harmont::pointcloud_object<Cloud, boost::shared_ptr>::ptr_t  cloud_obj(new harmont::pointcloud_object<Cloud, boost::shared_ptr>(m_cloud));
        //cloud_obj->init();
        //cloud_obj->set_point_colors(color);
        m_renderables.push_back(cloud_obj);
        m_renderedOrigins = std::make_shared<harmont::crosshair_object>(m_cloudOrigins, Eigen::Vector4f(1.f, 0.f, 0.f, 1.f), 0.5f, 0.5f);
        m_renderedOrigins->init();
	} else if (path.extension() == ".ifcmesh") {
        std::vector<ifc_mesh_extract::object_info> objects;
        std::ifstream in(path.string().c_str());
        if (!in.good()) {
            throw std::runtime_error("Unable to open file \"" + path.string() + "\" for reading.");
        }
        {
            cereal::JSONInputArchive ar(in);
            ar(cereal::make_nvp("guid", m_guid));
            ar(cereal::make_nvp("objects", objects));
        }
        in.close();
        for (const auto& obj : objects) {
            fs::path mesh_path = path.parent_path() / obj.path;
            ifc_mesh_extract::ifc_object_t<ColorType> ifc_obj;
            if (!MeshTraits::read(std::get<0>(ifc_obj), mesh_path.string())) {
                throw std::runtime_error("Could not open mesh file \"" + mesh_path.string() + "\"");
            }
            std::get<0>(ifc_obj).request_vertex_normals();
            std::get<0>(ifc_obj).request_face_normals();
            std::get<0>(ifc_obj).update_face_normals();
            std::get<0>(ifc_obj).update_normals();

            std::get<1>(ifc_obj) = obj.guid;
            std::get<2>(ifc_obj) = obj.type;

            m_objects.push_back(ifc_obj);
        }

		m_mesh    = MeshPtr(new Mesh());
		*m_mesh   = ifc_mesh_extract::extract_single_mesh<ColorType>(m_objects, &m_vertexMap);
		//for (auto it = m_mesh->vertices_begin(); it != m_mesh->vertices_end(); ++it) {
			//m_mesh->set_color(*it, Mesh::Color(1.f, 1.f, 1.f, 1.f));
		//}
        auto mesh_obj = duraark_vis::make_renderable(m_mesh, color);
		//harmont::mesh_object<Mesh>::ptr_t  mesh_obj(new harmont::mesh_object<Mesh>(m_mesh, false));
		//mesh_obj->init();
        //mesh_obj->set_vertex_colors(color);
		m_renderables.push_back(mesh_obj);
	} else if (path.extension() == ".ifc") {
		m_objects = ifc_mesh_extract::extract_objects<ColorType>(m_guid, path);
		m_mesh    = MeshPtr(new Mesh());
		*m_mesh   = ifc_mesh_extract::extract_single_mesh<ColorType>(m_objects, &m_vertexMap);
		for (auto it = m_mesh->vertices_begin(); it != m_mesh->vertices_end(); ++it) {
			m_mesh->set_color(*it, Mesh::Color(1.f, 1.f, 1.f, 1.f));
		}
		harmont::mesh_object<Mesh>::ptr_t  mesh_obj(new harmont::mesh_object<Mesh>(m_mesh, false));
		mesh_obj->init();
        mesh_obj->set_vertex_colors(color);
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

harmont::crosshair_object::ptr_t Registration::Representation::renderedOrigins() {
    return m_renderedOrigins;
}

harmont::renderable_group::ptr_t Registration::Representation::group() {
    return m_group;
}

harmont::renderable_group::const_ptr_t Registration::Representation::group() const {
    return m_group;
}

std::string Registration::Representation::guid() const {
    return m_guid;
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
        if (m_renderedOrigins) m_renderedOrigins->move(trafo);
	}
}

const std::vector<fs::path>& Registration::Representation::paths() const {
    return m_paths;
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

duraark_assoc::ifc_objects_t<Registration::Representation::ColorType>& Registration::Representation::objects() {
	return m_objects;
}

const duraark_assoc::ifc_objects_t<Registration::Representation::ColorType>& Registration::Representation::objects() const {
	return m_objects;
}

const std::vector<uint32_t>& Registration::Representation::cloudSizes() const {
	return m_cloudSizes;
}

const std::vector<Eigen::Vector3f>& Registration::Representation::cloudOrigins() const {
	return m_cloudOrigins;
}

std::vector<std::vector<int>> Registration::Representation::splitToScans(const std::vector<int32_t>& associations) const {
    uint32_t begin = 0;
    std::vector<std::vector<int>> result;
    for (uint32_t i = 0; i < m_cloudSizes.size(); ++i) {
        uint32_t end = begin + m_cloudSizes[i];
        std::vector<int> subset(end - begin);
        for (uint32_t j = 0; j < end-begin; ++j) {
            subset[j] = associations[begin + j];
        }
        begin = end;
        result.push_back(subset);
    }
    return result;
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
    for (auto& p : m_cloudOrigins) {
        p = t * p;
    }
    if (m_renderedOrigins) m_renderedOrigins->move(t.matrix());
}

void Registration::Representation::annotate(const std::vector<int32_t>& associations, const std::vector<uint32_t>& possible_hits, const std::vector<uint32_t>& actual_hits, float ratio_threshold, float points_per_square, Visualizer* vis, const Eigen::Vector4f& np_col, const Eigen::Vector4f& mp_col, const Eigen::Vector4f fp_col) {
    if (m_cloud) {
        m_renderables[0]->set_active(false);
        if (m_renderables.size() > 1) {
            vis->tryRemoveObject("cloud_annot_a");
            vis->tryRemoveObject("cloud_annot_b");
            m_renderables.erase(m_renderables.begin() + 1, m_renderables.end());
        }
        auto assoc = duraark_assoc::to_object_map(associations, 0);
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
        //cloudObjA->set_point_colors(fp_col);
        //cloudObjA->set_clipping(m_renderables.front()->clipping());
        //cloudObjA->set_clipping_height(m_renderables.front()->clipping_height());
        //m_renderables.push_back(cloudObjA);
        //vis->addObject("cloud_annot_a", cloudObjA);

        Cloud::Ptr cloudB(new Cloud(*m_cloud, b));
        //Cloud::Ptr cloudB(new Cloud(*m_cloud, a));
		harmont::pointcloud_object<Cloud, boost::shared_ptr>::ptr_t  cloudObjB(new harmont::pointcloud_object<Cloud, boost::shared_ptr>(cloudB));
		cloudObjB->init();
        cloudObjB->set_transformation(m_renderables.front()->transformation());
        cloudObjB->set_point_colors(np_col);
        //cloudObjB->set_point_colors(Eigen::Vector4f(1.f, 1.f, 1.f, 1.f));
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
                for (auto fv_iter = mesh->cfv_iter(*face_iter); fv_iter.is_valid(); ++fv_iter) {
                    vertices.push_back(Eigen::Vector3f(mesh->point(*fv_iter).data()));
                }
                float face_area = 0.5f * (vertices[1] - vertices[0]).cross(vertices[2]-vertices[0]).norm();
                area += face_area;
            }
            Mesh::Color color(mp_col.data());
            if (static_cast<float>(actual_hits[i]) / possible_hits[i] >= ratio_threshold && static_cast<float>(possible_hits[i]) / area >= points_per_square) {
                color = Mesh::Color(fp_col.data());
                ++debug_count;
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
     }
}

} // FW


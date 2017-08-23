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
using namespace duraark_vis;

using namespace GUI::Property;


namespace FW {



Registration::Registration(std::string id, const fs::path& path0, const fs::path& path1, const Eigen::Vector4f& col0, const Eigen::Vector4f& col1) : Visualizer(id), m_path0(path0), m_path1(path1), m_col0(col0), m_col1(col1) {
}

Registration::~Registration() {
}

void Registration::init() {
	try {
		m_rep0 = std::make_shared<representation>(m_path0, m_col0);
	} catch (std::runtime_error& error) {
		gui()->log()->error(std::string("Could not load first representation: ") + error.what());
        return;
	}
	try {
		m_rep1 = std::make_shared<representation>(m_path1, m_col1);
	} catch (std::runtime_error& error) {
		gui()->log()->error(std::string("Could not load second representation: ") + error.what());
        return;
	}

    //m_rep0->center();
    //m_rep1->center();
    addObject("rep0", m_rep0->renderable());
    addObject("rep1", m_rep1->renderable());

    //auto ch0 = m_rep0->renderedOrigins(), ch1 = m_rep1->renderedOrigins();
    //if (ch0) {
        //addObject("ch0", ch0);
    //}
    //if (ch1) {
        //addObject("ch1", ch1);
    //}

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
            object("rep0")->set_clipping(state);
            object("rep1")->set_clipping(state);
        }
    });

    auto save_reg = gui()->properties()->add<File>("Save Registration", "save_reg");
    save_reg->setMode(File::SAVE);
    save_reg->setExtensions({"rdf"});
    save_reg->setCallback([&] (const fs::path& path) {
        Eigen::Matrix4f t = (m_rep1->transformation().inverse() * m_rep0->transformation()).matrix();

        entity::type_t t0 = m_rep0->type() == representation::type_t::ifc ? entity::type_t::IFC : entity::type_t::PC;
        entity::type_t t1 = m_rep1->type() == representation::type_t::ifc ? entity::type_t::IFC : entity::type_t::PC;
        auto e0 = std::make_shared<entity>(t0, m_path0.stem().string(), m_rep0->guid());
        auto e1 = std::make_shared<entity>(t1, m_path1.stem().string(), m_rep1->guid());

        duraark_rdf::turtle_output turtle(path.string());
        write_prologue(turtle);
        write_entity(turtle, *e0);
        write_entity(turtle, *e1);
        write_registration(turtle, *e0, *e1, t);
    });

    auto load_reg = gui()->properties()->add<File>("Load Registration", "load_reg");
    load_reg->setExtensions({"rdf"});
    load_reg->setCallback([&] (const fs::path& path) {
        Eigen::Matrix4f t0 = m_rep0->transformation();
        m_rep0->set_transformation(Eigen::Matrix4f::Identity());
        m_rep1->set_transformation(Eigen::Matrix4f::Identity());

        duraark_rdf::turtle_input input(path.string());
        Eigen::Matrix4f m = duraark_rdf::parse_registration(input, m_rep0->guid(), m_rep1->guid());
        m_rep0->move(t0 * m);
        m_rep1->move(t0);
    });

	if ((m_rep0->type() == representation::type_t::ifc && m_rep1->type() == representation::type_t::pc) || (m_rep0->type() == representation::type_t::pc && m_rep1->type() == representation::type_t::ifc)) {
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

            auto cloud = m_rep0->type() == representation::type_t::pc ? m_rep0->cloud() : m_rep1->cloud();
            const auto& objs = m_rep0->type() == representation::type_t::ifc ? m_rep0->objects() : m_rep1->objects();

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
            if (m_rep0->type() == representation::type_t::ifc) {
               possible_hits.resize(m_rep0->objects().size(), 0);
               actual_hits.resize(m_rep0->objects().size(), 0);
               associations = duraark_assoc::associate_points<point_normal_t, OpenMesh::Vec4f>(m_rep0->objects(), m_rep1->cloud(), eps, possible_hits, actual_hits, m_rep1->cloud_metadata().scan_origins, m_rep1->cloud_metadata().scan_sizes, t0, t1, ignored_entity_types);
            } else {
               possible_hits.resize(m_rep1->objects().size(), 0);
               actual_hits.resize(m_rep1->objects().size(), 0);
               associations = duraark_assoc::associate_points<point_normal_t, OpenMesh::Vec4f>(m_rep0->cloud(), m_rep1->objects(), eps, possible_hits, actual_hits, m_rep0->cloud_metadata().scan_origins, m_rep0->cloud_metadata().scan_sizes, t0, t1, ignored_entity_types);
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
            auto cloud_rep = m_rep0->type() == representation::type_t::pc ? m_rep0 : m_rep1;
            auto ifc_rep = m_rep0->type() == representation::type_t::ifc ? m_rep0 : m_rep1;

            auto& objects = ifc_rep->objects();

            fs::path cloud_path = m_rep0->type() == representation::type_t::pc ? m_path0 : m_path1;
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
            annotate(ratio_threshold, points_per_square, np_col, mp_col, fp_col);
            annotate(ratio_threshold, points_per_square, np_col, mp_col, fp_col);

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
                     annotate(ratio_threshold, points_per_square, np_col, mp_col, fp_col);
                     annotate(ratio_threshold, points_per_square, np_col, mp_col, fp_col);
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
            auto r = m_rep0->renderable();
            float height = r->bounding_box().max()[2] - r->bounding_box().min()[2];
            r->delta_clipping_height((-dy / 40.f) / height);
        }

        if (clippingB) {
            auto r = m_rep1->renderable();
            float height = r->bounding_box().max()[2] - r->bounding_box().min()[2];
            r->delta_clipping_height((-dy / 40.f) / height);
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
                m_rep0->move(trafo.matrix());
            } else if (transformingB) {
                m_rep1->move(trafo.matrix());
            }
        }
    });
}

void Registration::annotate(float ratio_threshold, float points_per_square, const Eigen::Vector4f& np_col, const Eigen::Vector4f& mp_col, const Eigen::Vector4f fp_col) {
    representation::sptr_t cloud_rep, ifc_rep;
    cloud_rep = m_rep0->type() == representation::type_t::pc ? m_rep0 : m_rep1;
    ifc_rep = m_rep1->type() == representation::type_t::ifc ? m_rep1 : m_rep0;

    m_rep0->renderable()->set_active(false);
    m_rep1->renderable()->set_active(false);

    tryRemoveObject("cloud_annot_a");
    tryRemoveObject("cloud_annot_b");
    removeObjectGroup("annot_");

    // cloud
    auto assoc = duraark_assoc::to_object_map(m_associations, 0);
    std::set<uint32_t> associated, iota;
    for (uint32_t i = 0; i < cloud_rep->cloud()->size(); ++i) {
        iota.insert(i);
    }
    for (const auto& a : assoc) {
        associated.insert(a.second.begin(), a.second.end());
    }
    std::vector<uint32_t> extra;
    std::set_difference(iota.begin(), iota.end(), associated.begin(), associated.end(), std::back_inserter(extra));
    std::vector<int> a(associated.begin(), associated.end()), b(extra.begin(), extra.end());

    //cloud_normal_t::Ptr cloudA(new cloud_normal_t(*cloud_rep->cloud(), a));
    //harmont::pointcloud_object<cloud_normal_t, boost::shared_ptr>::ptr_t  cloudObjA(new harmont::pointcloud_object<cloud_normal_t, boost::shared_ptr>(cloudA));
    //cloudObjA->init();
    //cloudObjA->set_transformation(cloud_rep->renderable()->transformation());
    //cloudObjA->set_point_colors(fp_col);
    //cloudObjA->set_clipping(cloud_rep->renderable()->clipping());
    //cloudObjA->set_clipping_height(cloud_rep->renderable()->clipping_height());
    //addObject("cloud_annot_a", cloudObjA);

    cloud_normal_t::Ptr cloudB(new cloud_normal_t(*cloud_rep->cloud(), b));
    harmont::pointcloud_object<cloud_normal_t, boost::shared_ptr>::ptr_t cloudObjB(new harmont::pointcloud_object<cloud_normal_t, boost::shared_ptr>(cloudB));
    cloudObjB->init();
    cloudObjB->set_transformation(cloud_rep->renderable()->transformation());
    cloudObjB->set_point_colors(np_col);
    cloudObjB->set_clipping(cloud_rep->renderable()->clipping());
    cloudObjB->set_clipping_height(cloud_rep->renderable()->clipping_height());
    addObject("cloud_annot_b", cloudObjB);

    // ifc
    std::vector<harmont::renderable::ptr_t> group;
    for (uint32_t i = 0; i < ifc_rep->objects().size(); ++i) {
        std::shared_ptr<mesh_t> mesh(new mesh_t());
        *mesh = std::get<0>(ifc_rep->objects()[i]);

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
        mesh_t::Color color(mp_col.data());
        if (static_cast<float>(m_actualHits[i]) / m_possibleHits[i] >= ratio_threshold && static_cast<float>(m_possibleHits[i]) / area >= points_per_square) {
            color = mesh_t::Color(fp_col.data());
        }
        for (auto it = mesh->vertices_begin(); it != mesh->vertices_end(); ++it) {
            mesh->set_color(*it, color);
        }
        harmont::mesh_object<mesh_t>::ptr_t mesh_obj(new harmont::mesh_object<mesh_t>(mesh, false));
        mesh_obj->init();
        mesh_obj->set_transformation(ifc_rep->renderable()->transformation());
        group.push_back(mesh_obj);
    }
    auto obj_group = std::make_shared<harmont::renderable_group>(group);
    obj_group->set_clipping(ifc_rep->renderable()->clipping());
    obj_group->set_clipping_height(ifc_rep->renderable()->clipping_height());
    addObjectGroup("annot_", obj_group);
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
	gui()->properties()->add<File>("Input File A", "path0")->setExtensions({"ifc", "e57n", "ifcmesh"});
	gui()->properties()->add<File>("Input File B", "path1")->setExtensions({"ifc", "e57n", "ifcmesh"});

    auto diffCol = gui()->properties()->add<Group>("Colors", "colors");
    diffCol->add<Color>("Color A", "col_a")->setValue(Eigen::Vector4f(1.f, 1.f, 1.f, 1.f));
    diffCol->add<Color>("Color B", "col_b")->setValue(Eigen::Vector4f(1.f, 1.f, 1.f, 1.f));
}

Visualizer::Ptr Registration::Factory::addVisualizer() {
	std::string        name   = gui()->properties()->get<String>({"__name__"})->value();
	auto               path0 = gui()->properties()->get<File>({"path0"})->value();
	auto               path1 = gui()->properties()->get<File>({"path1"})->value();
    auto               col0   = gui()->properties()->get<Color>({"colors", "col_a"})->value();
    auto               col1   = gui()->properties()->get<Color>({"colors", "col_b"})->value();
	Registration::Ptr  vis(new Registration(name, path0, path1, col0, col1));
	return std::dynamic_pointer_cast<Visualizer>(vis);
}

} // FW


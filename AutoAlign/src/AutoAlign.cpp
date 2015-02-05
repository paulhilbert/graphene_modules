#include "AutoAlign.h"

#include <Library/Colors/Generation.h>
#include <duraark/traverse.hpp>

using namespace GUI::Property;


#define OPTIX_BUF_TYPE RTP_BUFFER_TYPE_HOST      // RTP_BUFFER_TYPE_CUDA_LINEAR


namespace FW {


AutoAlign::AutoAlign(std::string id, const fs::path& path) : Visualizer(id), m_path(path), m_lastHovered(boost::none) {
}

AutoAlign::~AutoAlign() {
}

void AutoAlign::init() {
    m_ifcObjects = duraark::extract_objects<Color>(m_path, true);
    Eigen::Vector3f center = Eigen::Vector3f::Zero();
    uint32_t idx = 0;
    for (auto& ifcObject : m_ifcObjects) {
        auto& mesh = std::get<0>(ifcObject);
        for (auto it = mesh.vertices_begin(); it != mesh.vertices_end(); ++it) {
            mesh.set_color(*it, Mesh::Color(1.f, 1.f, 1.f, 1.f));
        }
        std::shared_ptr<Mesh> mesh_ptr(new Mesh());
        *mesh_ptr = mesh;
        auto object = std::make_shared<Object>(mesh_ptr, false);
        object->init();
        m_objects.push_back(object);
        addObject(std::get<1>(ifcObject), object);

        center *= static_cast<float>(idx++);
        center += object->bounding_box().center();
        center /= static_cast<float>(idx);

        std::string type = std::get<2>(ifcObject);
        if (m_classMap.find(type) == m_classMap.end()) m_classMap[type] = std::vector<uint32_t>();
        m_classMap[type].push_back(m_objects.size() - 1);
    }

    Eigen::Matrix4f translation = Eigen::Matrix4f::Identity();
    translation.block<3,1>(0, 3) = -center;
    for (auto obj : m_objects) {
        obj->move(translation);
    }

    createOptixStructure();

	addProperties();
	registerEvents();
}

void AutoAlign::addProperties() {
    auto classVisSection = gui()->properties()->add<Section>("Class Visibility", "class_vis");
    for (const auto& v : m_classMap) {
        std::string type = v.first;
        auto prop = classVisSection->add<Bool>(type, "bool_"+type);
        prop->setValue(true);
        prop->setCallback(
            [&, type] (bool state) {
                for (auto obj_idx : m_classMap[type]) {
                    m_objects[obj_idx]->set_active(state);
                }
            }
        );
    }

	auto  showGroup = gui()->modes()->addGroup("showGroup");
	showGroup->addOption("showClip", "Enable Clipping", std::string(ICON_PREFIX) + "clipping.png");

	showGroup->setCallback([&] (std::string option, bool state) {
        if (option == "showClip") {
            for (auto& obj: m_objects) {
                obj->set_clipping(state);
            }
        }
    });

	auto  transformGroup = gui()->modes()->addGroup("TransformGroup");
	transformGroup->addOption("Clip", "Modify Clipping Plane", std::string(ICON_PREFIX) + "clipplane.png");

    gui()->properties()->add<Button>("Extract Wall Planes", "extract_planes")->setCallback([&] () { extractWallPlanes(); });
}

void AutoAlign::registerEvents() {
	fw()->events()->connect<void (int, int, int, int)>("LEFT_DRAG", [&] (int dx, int dy, int, int) {
        bool clipping = gui()->modes()->group("TransformGroup")->option("Clip")->active();

        if (clipping) {
            for (auto& obj: m_objects) {
                obj->delta_clipping_height(-dy * 0.01f);
            }
        }
    });

	fw()->events()->connect<void (int, int, int, int)>("MOVE", [&] (int, int, int x, int y) {
        Eigen::Vector3f origin, dir;
        std::tie(origin, dir) = fw()->camera()->pick_ray(x, y);
        boost::optional<uint32_t> hovered = selectObject(origin, dir);
        if (m_lastHovered) {
            m_objects[m_lastHovered.get()]->set_vertex_colors(Eigen::Vector4f(1.f, 1.f, 1.f, 1.f));
        }
        if (hovered) {
            std::string guid, type;
            std::tie(std::ignore, guid, type) = m_ifcObjects[*hovered];
            gui()->status()->set("Selected object \"" + guid + "\" :: " + type);
            m_objects[hovered.get()]->set_vertex_colors(Eigen::Vector4f(0.5f, 0.f, 0.f, 0.3f));
        } else {
            gui()->status()->clear();
        }
        m_lastHovered = hovered;
    });
}

void AutoAlign::createOptixStructure() {
    int32_t object_idx = 0, triangle_idx = 0;
    for (const auto& object : m_ifcObjects) {
        Eigen::Affine3f trans;
        trans = m_objects[object_idx]->transformation();
        uint32_t num_faces = std::get<0>(object).n_faces();
        std::vector<int32_t> tmp(num_faces, object_idx++);
        m_optixObjectMap.insert(m_optixObjectMap.end(), tmp.begin(), tmp.end());
        m_optixTriangles.resize(m_optixTriangles.size() + num_faces * 9);
        for (auto it = std::get<0>(object).faces_begin(); it != std::get<0>(object).faces_end(); ++it) {
            for (auto fvIt = std::get<0>(object).cfv_iter(*it); fvIt.is_valid(); ++fvIt) {
                auto pnt = std::get<0>(object).point(*fvIt);
                Eigen::Vector3f eigen_pnt(pnt[0], pnt[1], pnt[2]);
                eigen_pnt = trans * eigen_pnt;
                m_optixTriangles[triangle_idx++] = eigen_pnt[0];
                m_optixTriangles[triangle_idx++] = eigen_pnt[1];
                m_optixTriangles[triangle_idx++] = eigen_pnt[2];
            }
        }
    }

    try {
        m_optixContext = optix::prime::Context::create(RTP_CONTEXT_TYPE_CUDA);
    } catch (optix::prime::Exception& e) {
        m_optixContext = optix::prime::Context::create(RTP_CONTEXT_TYPE_CPU);
    }

    m_optixModel = m_optixContext->createModel();
    m_optixModel->setTriangles(m_optixTriangles.size() / 9, OPTIX_BUF_TYPE, m_optixTriangles.data());
    m_optixModel->update(0);
    m_optixModel->finish();
}

boost::optional<uint32_t> AutoAlign::selectObject(const Eigen::Vector3f& origin, const Eigen::Vector3f& dir) {
    float* ray = new float[8];
    int*   hit = new   int[4];

    for (uint32_t j = 0; j < 3; ++j) {
        ray[    j] = origin[j];
        ray[4 + j] = dir[j];
    }
    ray[3] = 0.f;
    ray[7] = 1000.f;

    try {
        optix::prime::Query query = m_optixModel->createQuery(RTP_QUERY_TYPE_CLOSEST);
		query->setRays(1, RTP_BUFFER_FORMAT_RAY_ORIGIN_TMIN_DIRECTION_TMAX, OPTIX_BUF_TYPE, ray);
		query->setHits(1, RTP_BUFFER_FORMAT_HIT_T_TRIID_U_V, OPTIX_BUF_TYPE, hit);
		query->execute(0);
    } catch (optix::prime::Exception& e) {
        delete [] ray;
        delete [] hit;
        throw std::runtime_error("Prime Runtime Exception: " + e.getErrorString());
    }


    int triHit = hit[1];
    if (triHit >= 0 && m_objects[m_optixObjectMap[triHit]]->active()) {
        return static_cast<uint32_t>(m_optixObjectMap[triHit]);
    }

    delete [] ray;
    delete [] hit;

    return boost::none;
}

//void AutoAlign::extractWallPlanes() {
    //for (const auto& obj : m_ifcObjects) {
        //std::string type = std::get<2>(obj);
        //if (type != "ifcwall" && type != "ifcwallstandardcase") continue;
        //const Mesh& mesh = std::get<0>(obj);
        //std::set<uint32_t> todo;
        //for (uint32_t i = 0; i < mesh.n_faces(); ++i) {
            //todo.insert(i);
        //}

        //// precompute normals
        //std::vector<Eigen::Vector3f> normals(mesh.n_faces());
        //uint32_t idx = 0;
        //for (auto it = mesh.faces_begin(); it != mesh.faces_end(); ++it, ++idx) {
            //auto v_it = mesh.cfv_iter(*it);
            //Eigen::Vector3f p0(mesh.point(*(v_it++)).data());
            //Eigen::Vector3f p1(mesh.point(*(v_it++)).data());
            //Eigen::Vector3f p2(mesh.point(*(v_it++)).data());
            //Eigen::Vector3f nrm = (p1-p0).cross(p2-p0).normalized();
            //normals[idx] = nrm;
        //}

        //std::vector<std::set<uint32_t>> components;
        //while (todo.size()) {
            //uint32_t start_idx = *(todo.begin());
            //todo.erase(start_idx);

            //auto it = mesh.faces_begin();
            //std::advance(it, start_idx);
            //const Eigen::Vector3f& nrm = normals[start_idx];
            //if (nrm[2] < 0.01f) continue;

            //// grow
            //std::deque<uint32_t> queue(1, start_idx);
            //std::set<uint32_t> component({start_idx});
            //while (queue.size()) {
                //uint32_t crt_idx = queue.front();
                //queue.pop_front();
                //std::set<uint32_t> neighbors;
                //auto crt_it = mesh.faces_begin();
                //std::advance(crt_it, crt_idx);
                //for (auto n_it = mesh.cff_iter(*crt_it); n_it.is_valid(); ++n_it) {
                    //uint32_t n_idx = n_it->idx();
                    //if (fabs(normals[n_idx].dot(nrm)) > 0.001f) continue;
                    //if (component.count(n_idx) > 0) continue;
                    //component.insert(n_idx);
                    //queue.push_back(n_idx);
                    //todo.erase(n_idx);
                //}
            //}
            //components.push_back(component);
        //}
        //m_wallComponents.push_back(components);
    //}

    //for (uint32_t i = 0; i < m_wallComponents.size(); ++i) {
        //typedef cartan::mesh_traits<Mesh> mt;

        //const auto& components = m_wallComponents[i];
        //const auto& mesh = std::get<0>(m_ifcObjects[i]);

        //for (uint32_t c = 0; c < components.size(); ++c) {
            //if (!components[c].size()) continue;
            //std::vector<uint32_t> subset(components[c].begin(), components[c].end());
            //auto component_mesh = mt::mesh_from_face_subset(mesh, subset);
            //auto rendered = std::make_shared<Object>(component_mesh, false);
            //rendered->init();
            //addObject("obj_"+std::to_string(i)+"_component_"+std::to_string(c), rendered);
            //break;
        //}
        //break;
    //}
//}

void AutoAlign::extractWallPlanes() {
    typedef cartan::mesh_traits<Mesh> mt;
    typedef mt::face_handle_t entity_t;
    typedef duraark::connected_component_t<entity_t> component_t;
    typedef duraark::connected_components_t<entity_t> components_t;

    m_wallComponents.clear();
    for (const auto& obj : m_ifcObjects) {
        std::string type = std::get<2>(obj);
        if (type != "ifcwall" && type != "ifcwallstandardcase") {
            m_wallComponents.push_back(std::vector<std::set<uint32_t>>());
            continue;
        }
        const Mesh& mesh = std::get<0>(obj);

        auto faces = mt::face_handles(mesh);

        // precompute face normals
        //std::vector<mt::eigen_vec3_t> normals(mt::num_faces(mesh));
        //std::transform(faces.begin(), faces.end(), normals.begin(), [&] (mt::face_handle_t f) { return mt::eigen_face_normal(mesh, f); });

        auto get_neighbors = [&] (entity_t face) { return mt::adjacent_faces(mesh, face); };
        components_t components = duraark::connected_components(
            faces,
            get_neighbors,
            [&] (entity_t face, entity_t start_face) {
                mt::eigen_vec3_t nrm = mt::eigen_face_normal(mesh, face);
                mt::eigen_vec3_t start_nrm = mt::eigen_face_normal(mesh, start_face);
                // not in component if not vertical...
                if (fabs(nrm[2]) > 0.01f) return false;
                // ...or if not colinear to start face
                if (1.f - fabs(nrm.dot(start_nrm)) > Eigen::NumTraits<float>::dummy_precision()) return false;
                return true;
            },
            [&] (entity_t start_face) {
                mt::eigen_vec3_t nrm = mt::eigen_face_normal(mesh, start_face);
                return fabs(nrm[2]) < 0.01f;
            }
        );

        // convert to indices
        std::vector<std::set<uint32_t>> result(components.size());
        std::transform(
            components.begin(),
            components.end(),
            result.begin(),
            [&] (const component_t& comp) {
                std::set<uint32_t> idx_comp;
                for (const auto& entity : comp) {
                    uint32_t idx = entity.idx();
                    idx_comp.insert(idx);
                }
                return idx_comp;
            }
        );

        m_wallComponents.push_back(result);
    }

    for (uint32_t i = 0; i < m_wallComponents.size(); ++i) {
        typedef cartan::mesh_traits<Mesh> mt;

        const auto& components = m_wallComponents[i];
        const auto& mesh = std::get<0>(m_ifcObjects[i]);

        uint32_t plane_count = std::count_if(components.begin(), components.end(), [&] (auto c) { return c.size() > 0; });
        std::vector<RGBA> colors = Colors::Generation::shuffledColorsRGBA(plane_count, {0.f, 2.f*M_PI}, {1.f, 1.f}, {1.f, 1.f}, 1.f);
        uint32_t color_idx = 0;

        for (uint32_t c = 0; c < components.size(); ++c) {
            if (components[c].size() < 1) continue;
            std::vector<uint32_t> subset(components[c].begin(), components[c].end());
            auto component_mesh = mt::mesh_from_face_subset(mesh, subset);
            auto rendered = std::make_shared<Object>(component_mesh, false);
            rendered->init();
            rendered->set_face_colors(colors[color_idx++]);
            rendered->set_transformation(m_objects[c]->transformation());
            addObject("obj_"+std::to_string(i)+"_component_"+std::to_string(c), rendered);
            //break;
        }
        //break;
    }
}

AutoAlign::Factory::Factory() : FW::Factory() {
}

AutoAlign::Factory::~Factory() {
}

void AutoAlign::Factory::init() {
    gui()->properties()->add<File>("Input IFC File", "path")->setExtensions({"ifc"});
}

Visualizer::Ptr AutoAlign::Factory::addVisualizer() {
	std::string name = gui()->properties()->get<String>({"__name__"})->value();
    auto path = gui()->properties()->get<File>({"path"})->value();
	AutoAlign::Ptr  vis(new AutoAlign(name, path));
	return std::dynamic_pointer_cast<Visualizer>(vis);
}


} // FW
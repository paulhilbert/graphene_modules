#include "RenderIFC.h"

#include <Library/Colors/Generation.h>

using namespace GUI::Property;


#define OPTIX_BUF_TYPE RTP_BUFFER_TYPE_HOST      // RTP_BUFFER_TYPE_CUDA_LINEAR


namespace FW {


RenderIFC::RenderIFC(std::string id, const fs::path& path) : Visualizer(id), m_path(path), m_lastHovered(boost::none) {
}

RenderIFC::~RenderIFC() {
}

void RenderIFC::init() {
    m_ifcObjects = duraark::extract_objects<Color>(m_path);
    m_colors = Colors::Generation::shuffledColorsRGBA(m_ifcObjects.size(), {0.f, static_cast<float>(2*M_PI)}, {1.f, 1.f});
    Eigen::Vector3f center = Eigen::Vector3f::Zero();
    uint32_t idx = 0;

    std::vector<harmont::renderable::ptr_t> objects;

    for (auto& ifcObject : m_ifcObjects) {
        auto& mesh = std::get<0>(ifcObject);
        for (auto it = mesh.vertices_begin(); it != mesh.vertices_end(); ++it) {
            mesh.set_color(*it, Mesh::Color(1.f, 1.f, 1.f, 1.f));
            //mesh.set_color(*it, Mesh::Color(m_colors[idx].data()));
        }
        std::shared_ptr<Mesh> mesh_ptr(new Mesh());
        *mesh_ptr = mesh;
        auto object = std::make_shared<Object>(mesh_ptr, false);
        object->init();
        objects.push_back(object);
        addObject(std::get<1>(ifcObject), object);

        center *= static_cast<float>(idx++);
        center += object->bounding_box().center();
        center /= static_cast<float>(idx);

        std::string type = std::get<2>(ifcObject);
        if (m_classMap.find(type) == m_classMap.end()) m_classMap[type] = std::vector<uint32_t>();
        m_classMap[type].push_back(m_ifcObjects.size() - 1);
    }

    m_objects = std::make_shared<harmont::renderable_group>(objects);

    Eigen::Matrix4f translation = Eigen::Matrix4f::Identity();
    translation.block<3,1>(0, 3) = -center;
    for (auto obj : m_objects->objects()) {
        obj->move(translation);
        obj->set_clipping_normal(Eigen::Vector3f::UnitY());
    }

    createOptixStructure();

	addProperties();
	registerEvents();
}

void RenderIFC::addProperties() {
    auto classVisSection = gui()->properties()->add<Section>("Class Visibility", "class_vis");
    for (const auto& v : m_classMap) {
        std::string type = v.first;
        auto prop = classVisSection->add<Bool>(type, "bool_"+type);
        prop->setValue(true);
        prop->setCallback(
            [&, type] (bool state) {
                m_objects->set_active(state);
            }
        );
    }

	auto  showGroup = gui()->modes()->addGroup("showGroup");
	showGroup->addOption("showClip", "Enable Clipping", std::string(ICON_PREFIX) + "clipping.png");

	showGroup->setCallback([&] (std::string option, bool state) {
        if (option == "showClip") {
            m_objects->set_clipping(state);
        }
    });

	auto  transformGroup = gui()->modes()->addGroup("TransformGroup");
	transformGroup->addOption("Clip", "Modify Clipping Plane", std::string(ICON_PREFIX) + "clipplane.png");
}

void RenderIFC::registerEvents() {
	fw()->events()->connect<void (int, int, int, int)>("LEFT_DRAG", [&] (int dx, int dy, int, int) {
        bool clipping = gui()->modes()->group("TransformGroup")->option("Clip")->active();

        if (clipping) {
            m_objects->delta_clipping_height(-dy * 0.01f);
        }
    });

	fw()->events()->connect<void (int, int, int, int)>("MOVE", [&] (int, int, int x, int y) {
        Eigen::Vector3f origin, dir;
        std::tie(origin, dir) = fw()->camera()->pick_ray(x, y);
        boost::optional<uint32_t> hovered = selectObject(origin, dir);
        if (m_lastHovered) {
            //std::dynamic_pointer_cast<Object>(m_objects->objects()[m_lastHovered.get()])->set_vertex_colors(m_colors[m_lastHovered.get()]);
            std::dynamic_pointer_cast<Object>(m_objects->objects()[m_lastHovered.get()])->set_vertex_colors(Eigen::Vector4f::Ones());
        }
        if (hovered) {
            std::string guid, type;
            std::tie(std::ignore, guid, type) = m_ifcObjects[*hovered];
            gui()->status()->set("Selected object \"" + guid + "\" :: " + type);
            std::dynamic_pointer_cast<Object>(m_objects->objects()[hovered.get()])->set_vertex_colors(Eigen::Vector4f(0.5f, 0.5f, 0.5f, 0.4f));
        } else {
            gui()->status()->clear();
        }
        m_lastHovered = hovered;
    });
}

void RenderIFC::createOptixStructure() {
    int32_t object_idx = 0, triangle_idx = 0;
    for (const auto& object : m_ifcObjects) {
        Eigen::Affine3f trans;
        trans = m_objects->objects()[object_idx]->transformation();
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

boost::optional<uint32_t> RenderIFC::selectObject(const Eigen::Vector3f& origin, const Eigen::Vector3f& dir) {
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
    if (triHit >= 0 && m_objects->objects()[m_optixObjectMap[triHit]]->active()) {
        return static_cast<uint32_t>(m_optixObjectMap[triHit]);
    }

    delete [] ray;
    delete [] hit;

    return boost::none;
}

RenderIFC::Factory::Factory() : FW::Factory() {
}

RenderIFC::Factory::~Factory() {
}

void RenderIFC::Factory::init() {
    gui()->properties()->add<File>("Input IFC File", "path")->setExtensions({"ifc"});
}

Visualizer::Ptr RenderIFC::Factory::addVisualizer() {
	std::string name = gui()->properties()->get<String>({"__name__"})->value();
    auto path = gui()->properties()->get<File>({"path"})->value();
	RenderIFC::Ptr  vis(new RenderIFC(name, path));
	return std::dynamic_pointer_cast<Visualizer>(vis);
}


} // FW

#include "RenderIFC.h"

#include <Library/Colors/Generation.h>

using namespace GUI::Property;


namespace FW {


RenderIFC::RenderIFC(std::string id, const fs::path& path) : Visualizer(id), m_path(path) {
}

RenderIFC::~RenderIFC() {
}

void RenderIFC::init() {
    m_ifcObjects = duraark::extract_objects<Color>(m_path);
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
}

void RenderIFC::registerEvents() {
	fw()->events()->connect<void (int, int, int, int)>("LEFT_DRAG", [&] (int dx, int dy, int, int) {
        bool clipping = gui()->modes()->group("TransformGroup")->option("Clip")->active();

        if (clipping) {
            for (auto& obj: m_objects) {
                obj->delta_clipping_height(-dy * 0.01f);
            }
        }
    });
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

#include <MeshRender.h>


namespace FW {

MeshRender::MeshRender(std::string id, fs::path meshFile) : Visualizer(id), m_meshFile(meshFile) {
}

MeshRender::~MeshRender() {
}

void MeshRender::init() {
    if (!fs::exists(m_meshFile)) {
        gui()->log()->error("Mesh file \"" + m_meshFile.string() + "\" does not exist.");
        return;
    }
    std::cout << "load mesh" << "\n";
    try {
        m_mesh = std::make_shared<RenderedMeshT>(m_meshFile.string(), false);
    } catch (std::exception& e) {
        std::cout << e.what() << "\n";
        return;
    }
    std::cout << "load transp mesh" << "\n";
    m_meshTransp = std::make_shared<RenderedMeshT>(m_meshFile.string(), false);
	gui()->log()->info("Loaded mesh with "+lexical_cast<std::string>(m_mesh->mesh()->n_vertices())+" vertices and "+lexical_cast<std::string>(m_mesh->mesh()->n_faces())+" faces.");

    std::cout << "mesh init" << "\n";
    m_mesh->init();
    m_meshTransp->init();
    std::cout << "toggle invert" << "\n";
    m_meshTransp->toggle_invert_clipping();
    std::cout << "set colors" << "\n";
    m_meshTransp->set_vertex_colors(Eigen::Vector4f(1.f, 0.f, 0.f, 1.0f));

    std::cout << "add objects" << "\n";
    addObject("main mesh", m_mesh);
    addObject("transp mesh", m_meshTransp);

    std::cout << "add props" << "\n";
	addProperties();
    std::cout << "add modes" << "\n";
    addModes();
    std::cout << "register events" << "\n";
	registerEvents();
}

void MeshRender::addProperties() {
    auto shadows = gui()->properties()->add<Bool>("Cast Shadows", "casts_shadows");
    shadows->setValue(true);
    shadows->setCallback([&] (bool state) { m_mesh->set_casts_shadows(state); });
}

void MeshRender::addModes() {
	auto showGroup = gui()->modes()->addGroup("showGroup");
	showGroup->addOption("showClip", "Enable Clipping", std::string(ICON_PREFIX)+"clipping.png");
	auto editGroup = gui()->modes()->addGroup("editGroup");
	editGroup->addOption("editClip", "Modify Clipping Plane", std::string(ICON_PREFIX)+"clipplane.png");

    showGroup->setCallback([&] (std::string option, bool state) {
        if (option == "showClip") {
            m_mesh->set_clipping(state);
            m_meshTransp->set_clipping(state);
        }
    });
}

void MeshRender::registerEvents() {
    fw()->events()->connect<void (int, int, int, int)>("LEFT_DRAG", [&] (int, int dy, int, int) {
        if (!gui()->modes()->group("editGroup")->option("editClip")->active()) return;
        m_mesh->delta_clipping_height(-dy * 0.01f);
        m_meshTransp->delta_clipping_height(-dy * 0.01f);
    });
}

MeshRender::Factory::Factory() : FW::Factory() {
}

MeshRender::Factory::~Factory() {
}

void MeshRender::Factory::init() {
	auto path = gui()->properties()->add<File>("Input mesh", "path");
	path->setExtensions({"obj", "off", "ply"});
}

Visualizer::Ptr MeshRender::Factory::addVisualizer() {
	std::string name = gui()->properties()->get<String>({"__name__"})->value();
	auto path = gui()->properties()->get<File>({"path"})->value();
	MeshRender::Ptr vis(new MeshRender(name, path));
	return std::dynamic_pointer_cast<Visualizer>(vis);
}

} // FW


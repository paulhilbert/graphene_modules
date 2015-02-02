#ifndef RenderIFCVIS_H_
#define RenderIFCVIS_H_

#include <FW/FWVisualizer.h>
#include <FW/FWFactory.h>

#include <cartan/openmesh_traits.hpp>
#include <cartan/mesh_algorithms.hpp>
#include <duraark/extract_objects.hpp>


namespace FW {


class RenderIFC : public FW::Visualizer {
	public:
		typedef std::shared_ptr<RenderIFC>    Ptr;
		typedef std::weak_ptr<RenderIFC>      WPtr;
        typedef OpenMesh::Vec4f               Color;
        typedef cartan::openmesh_t<Color>     Mesh;
        typedef harmont::mesh_object<Mesh>    Object;
        typedef duraark::ifc_object_t<Color>  IfcObject;
        typedef duraark::ifc_objects_t<Color> IfcObjects;

	public:
		class Factory;
        class Representation;

	public:
		RenderIFC(std::string id, const fs::path& path);
		virtual ~RenderIFC();

		void init();
		void addProperties();
		void registerEvents();

    protected:
        fs::path                                      m_path;
        IfcObjects                                    m_ifcObjects;
        std::vector<Object::ptr_t>                    m_objects;
        std::map<std::string, std::vector<uint32_t>>  m_classMap;
};


class RenderIFC::Factory : public FW::Factory {
	public:
		Factory();
		virtual ~Factory();

		void init();
		Visualizer::Ptr addVisualizer();
};


} // FW

#endif /* RenderIFCVIS_H_ */

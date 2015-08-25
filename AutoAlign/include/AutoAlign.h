#ifndef AUTOALIGNVIS_H_
#define AUTOALIGNVIS_H_

#include <vector>
#include <set>

#include <boost/optional.hpp>
#include <boost/none.hpp>

#include <optix_prime/optix_primepp.h>

#include <FW/FWVisualizer.h>
#include <FW/FWFactory.h>

#include <cartan/openmesh_traits.hpp>
#include <cartan/mesh_algorithms.hpp>
#include <duraark/extract_objects.hpp>
#include <duraark/align.hpp>


namespace FW {


class AutoAlign : public FW::Visualizer {
	public:
		typedef std::shared_ptr<AutoAlign>    Ptr;
		typedef std::weak_ptr<AutoAlign>      WPtr;
        typedef OpenMesh::Vec4f               Color;
        typedef cartan::openmesh_t<Color>     Mesh;
        typedef harmont::mesh_object<Mesh>    Object;
        typedef duraark::ifc_object_t<Color>  IfcObject;
        typedef duraark::ifc_objects_t<Color> IfcObjects;

	public:
		class Factory;
        class Representation;

	public:
		AutoAlign(std::string id, const fs::path& path);
		virtual ~AutoAlign();

		void init();
		void addProperties();
		void registerEvents();

    protected:
        void createOptixStructure();
        boost::optional<uint32_t> selectObject(const Eigen::Vector3f& origin, const Eigen::Vector3f& dir);
        void extractWallPlanes();
        std::vector<Eigen::Vector3f> planesTo2DLines(const std::vector<duraark::plane_t>& planes);

    protected:
        fs::path                                      m_path;
        IfcObjects                                    m_ifcObjects;
        harmont::renderable_group::ptr_t              m_objectGroup;
        std::map<std::string, std::vector<uint32_t>>  m_classMap;
        optix::prime::Context                         m_optixContext;
        optix::prime::Model                           m_optixModel;
        std::vector<float>                            m_optixTriangles;
        std::vector<int32_t>                          m_optixObjectMap;
        boost::optional<uint32_t>                     m_lastHovered;
        std::vector<std::vector<std::set<uint32_t>>>  m_wallComponents;
};


class AutoAlign::Factory : public FW::Factory {
	public:
		Factory();
		virtual ~Factory();

		void init();
		Visualizer::Ptr addVisualizer();
};


} // FW

#endif /* AUTOALIGNVIS_H_ */

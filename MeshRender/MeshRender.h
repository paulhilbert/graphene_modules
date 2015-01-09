#ifndef MESHRENDERVIS_H_
#define MESHRENDERVIS_H_

#include <FW/FWVisualizer.h>
#include <FW/FWFactory.h>

#include <cartan/openmesh_traits.hpp>

namespace FW {

class MeshRender : public Visualizer {
	public:
		typedef std::shared_ptr<MeshRender> Ptr;
		typedef std::weak_ptr<MeshRender>   WPtr;

        typedef OpenMesh::Vec4f             ColorT;
		typedef cartan::openmesh_t<ColorT>  MeshT;
        typedef harmont::mesh_object<MeshT> RenderedMeshT;

	public:
		class Factory;

	public:
		MeshRender(std::string id, fs::path meshFile);
		virtual ~MeshRender();

		void init();
		void addProperties();
        void addModes();
		void registerEvents();

	protected:
		fs::path              m_meshFile;
        RenderedMeshT::ptr_t  m_mesh;
        RenderedMeshT::ptr_t  m_meshTransp;
};


class MeshRender::Factory : public FW::Factory {
	public:
		Factory();
		virtual ~Factory();

		void init();
		Visualizer::Ptr addVisualizer();
};


} // FW

#endif /* MESHRENDERVIS_H_ */

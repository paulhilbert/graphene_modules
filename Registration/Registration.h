#ifndef REGISTRATIONVIS_H_
#define REGISTRATIONVIS_H_

#include <FW/FWVisualizer.h>
#include <FW/FWFactory.h>
#include <cartan/openmesh_traits.hpp>
#include <cartan/mesh_algorithms.hpp>
#include <duraark/registration.hpp>

#include <harmont/pcl_traits.hpp>
#include <harmont/crosshair_object.hpp>

namespace FW {


class Registration : public FW::Visualizer {
	public:
		typedef std::shared_ptr<Registration> Ptr;
		typedef std::weak_ptr<Registration>   WPtr;

	public:
		class Factory;
        class Representation;

	public:
		Registration(std::string id, const std::vector<fs::path>& paths0, const std::vector<fs::path>& paths1);
		virtual ~Registration();

		void init();
		void addProperties();
		void registerEvents();

    protected:
        void setICPParameters_();

    protected:
        std::vector<fs::path> m_paths0;
        std::vector<fs::path> m_paths1;
        std::shared_ptr<Representation> m_rep0;
        std::shared_ptr<Representation> m_rep1;
        duraark::registration::ptr_t    m_registration;
        //Rendered::Point::Ptr            m_origin;
		float                           m_clippingHeight;
};


class Registration::Factory : public FW::Factory {
	public:
		Factory();
		virtual ~Factory();

		void init();
		Visualizer::Ptr addVisualizer();
};


} // FW

#endif /* REGISTRATIONVIS_H_ */

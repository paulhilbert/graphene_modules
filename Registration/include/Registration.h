#ifndef REGISTRATIONVIS_H_
#define REGISTRATIONVIS_H_

#include <FW/FWVisualizer.h>
#include <FW/FWFactory.h>
#include <cartan/openmesh_traits.hpp>
#include <cartan/mesh_algorithms.hpp>
//#include <duraark/registration.hpp>

#include <harmont/pcl_traits.hpp>
#include <harmont/crosshair_object.hpp>
#include <duraark_vis/representation.hpp>

namespace FW {

class Registration : public FW::Visualizer {
	public:
		typedef std::shared_ptr<Registration> Ptr;
		typedef std::weak_ptr<Registration>   WPtr;

	public:
		class Factory;
        //class Representation;

	public:
		Registration(std::string id, const fs::path& path0, const fs::path& path1, const Eigen::Vector4f& col0, const Eigen::Vector4f& col1);
		virtual ~Registration();

		void init();
		void addProperties();
		void registerEvents();

    protected:
        void annotate(float ratio_threshold, float points_per_square, const Eigen::Vector4f& np_col, const Eigen::Vector4f& mp_col, const Eigen::Vector4f fp_col);
        //void setICPParameters_();

    protected:
        fs::path m_path0;
        fs::path m_path1;
        Eigen::Vector4f       m_col0;
        Eigen::Vector4f       m_col1;
        duraark_vis::representation::sptr_t m_rep0;
        duraark_vis::representation::sptr_t m_rep1;
        //std::shared_ptr<Representation> m_rep0;
        //std::shared_ptr<Representation> m_rep1;
        //duraark::registration::ptr_t    m_registration;
		float                           m_clippingHeight;
		std::vector<uint32_t>           m_possibleHits;
        std::vector<uint32_t>           m_actualHits;
        std::vector<int32_t>            m_associations;
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

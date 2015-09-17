#ifndef SmartStreamVIS_H_
#define SmartStreamVIS_H_

#include <thread>
//#include <asio.hpp>

#include <FW/FWVisualizer.h>
#include <FW/FWFactory.h>

#define USE_PCL_COMPRESS
#include <pcl_compress_stream/smart_client.hpp>

#include "mutable_pointcloud.hpp"

#include "types.hpp"
#include "vis_client.hpp"

//class vis_client;

namespace FW {

class SmartStream : public FW::Visualizer {
	public:
		typedef std::shared_ptr<SmartStream>    Ptr;
		typedef std::weak_ptr<SmartStream>      WPtr;

	public:
		class Factory;
        class Representation;

	public:
		SmartStream(std::string id, const std::string& host, const std::string& port);
		virtual ~SmartStream();

		void init();
		void addProperties();
		void registerEvents();

        void renderArrangement();

        void update(Arr::Face_handle pickedFace);

        void preRender();

    protected:
        std::set<uint32_t> getPatches(int face_idx);
        std::set<uint32_t> getWallPatches(int face_idx);

        void computeNewSets(std::vector<request_t>& priority_sets,
                            std::vector<request_t>& new_sets,
                            std::vector<int>& removable,
                            const Arr::Face_handle& current_face);

    protected:
        //std::vector<fs::path>  cloud_paths_;
        //harmont::renderable_group::ptr_t group_;
        //bool center_;
        std::string host_;
        std::string port_;
        std::shared_ptr<vis_client> client_;
        mutable_pointcloud::ptr_t mcloud_;
        //std::shared_ptr<std::thread> stream_thread_;
        std::set<int> current_scans_;
        int crt_face_;

        std::deque<request_t> request_queue_;
        std::mutex request_queue_mutex_;

        bool tracking_;
        std::set<int> covered_faces_;

        //harmont::renderable::ptr_t renderable_;
        //std::shared_ptr<harmont::renderable::map_t> display_map_;
        //std::shared_ptr<harmont::renderable::map_t> shadow_map_;
};


class SmartStream::Factory : public FW::Factory {
	public:
		Factory();
		virtual ~Factory();

		void init();
		Visualizer::Ptr addVisualizer();
};


} // FW

#endif /* SmartStreamVIS_H_ */

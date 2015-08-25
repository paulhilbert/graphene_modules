#ifndef RenderE57StreamVIS_H_
#define RenderE57StreamVIS_H_

#include <thread>
#include <asio.hpp>

#include <FW/FWVisualizer.h>
#include <FW/FWFactory.h>

class stream_client;

namespace FW {


class RenderE57Stream : public FW::Visualizer {
	public:
		typedef std::shared_ptr<RenderE57Stream>    Ptr;
		typedef std::weak_ptr<RenderE57Stream>      WPtr;

	public:
		class Factory;
        class Representation;

	public:
		RenderE57Stream(std::string id, const std::string& host, const std::string& port);
		virtual ~RenderE57Stream();

		void init();
		void addProperties();
		void registerEvents();

        void preRender();

    protected:
        std::vector<fs::path>  cloud_paths_;
        harmont::renderable_group::ptr_t group_;
        bool center_;
        std::string host_;
        std::string port_;
        asio::io_context io_context_;
        std::shared_ptr<stream_client> client_;
        std::shared_ptr<std::thread> stream_thread_;
        harmont::renderable::ptr_t renderable_;
        std::shared_ptr<harmont::renderable::map_t> display_map_;
        std::shared_ptr<harmont::renderable::map_t> shadow_map_;
};


class RenderE57Stream::Factory : public FW::Factory {
	public:
		Factory();
		virtual ~Factory();

		void init();
		Visualizer::Ptr addVisualizer();
};


} // FW

#endif /* RenderE57StreamVIS_H_ */

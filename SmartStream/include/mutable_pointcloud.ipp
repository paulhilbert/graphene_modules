namespace FW {

template <typename F>
inline void
mutable_pointcloud::with_vertex_data(F&& func) const {
    std::lock_guard<std::mutex> g(cloud_mutex_);
    func(std::cref(this->vertex_data_));
}

} // FW

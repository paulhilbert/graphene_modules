template <class PointT>
inline typename PCLTools<PointT>::CloudType::Ptr PCLTools<PointT>::loadPointCloud(fs::path cloudPath, optional<std::vector<Vector4f>&> colors) {
	std::string extension = cloudPath.extension().string();
	if (extension == ".pcd") return loadPointCloudFromPCD(cloudPath, colors);
	if (extension == ".obj") return loadPointCloudFromOBJ(cloudPath, colors);
#ifdef USE_E57
	if (extension == ".e57") return loadPointCloudFromE57(cloudPath, colors);
#endif // USE_E57
	throw std::runtime_error("Could not load point cloud \""+cloudPath.string()+"\". Unknown file extension.");
}

template <>
inline typename PCLTools<pcl::PointXYZRGBNormal>::CloudType::Ptr PCLTools<pcl::PointXYZRGBNormal>::loadPointCloudFromPCD(fs::path cloudPath, optional<std::vector<Vector4f>&> colors) {
	typename CloudType::Ptr cloud(new CloudType());
	if (pcl::io::loadPCDFile<pcl::PointXYZRGBNormal>(cloudPath.string(), *cloud) == -1) {
		throw std::runtime_error("Could not load point cloud \""+cloudPath.string()+"\"");
	}
	if (colors) {
		colors.get().resize(cloud->size());
        for (uint32_t i = 0; i < cloud->size(); ++i) {
            float r = static_cast<float>(cloud->points[i].r) / 255.f;
            float g = static_cast<float>(cloud->points[i].g) / 255.f;
            float b = static_cast<float>(cloud->points[i].b) / 255.f;
            //float a = static_cast<float>(cloud->points[i].a) / 255.f;
            colors.get()[i] = Vector4f(r, g, b, 1.f);
        }
		//std::fill(colors.get().begin(), colors.get().end(), Vector4f(0.5f, 0.5f, 0.5f, 1.f));
	}
	unsigned int pointCount = cloud->size();
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
    pcl::removeNaNNormalsFromPointCloud(*cloud, *cloud, indices);
	if (cloud->size() != pointCount) {
		std::cout << "Dropped " << (pointCount - cloud->size()) << " points due to NaN values." << "\n";
	}
	return cloud;
}

template <class PointT>
inline typename PCLTools<PointT>::CloudType::Ptr PCLTools<PointT>::loadPointCloudFromPCD(fs::path cloudPath, optional<std::vector<Vector4f>&> colors) {
	typename CloudType::Ptr cloud(new CloudType());
	if (pcl::io::loadPCDFile<PointT>(cloudPath.string(), *cloud) == -1) {
		throw std::runtime_error("Could not load point cloud \""+cloudPath.string()+"\"");
	}
	if (colors) {
		colors.get().resize(cloud->size());
		std::fill(colors.get().begin(), colors.get().end(), Vector4f(0.5f, 0.5f, 0.5f, 1.f));
	}
	unsigned int pointCount = cloud->size();
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
    pcl::removeNaNNormalsFromPointCloud(*cloud, *cloud, indices);
	if (cloud->size() != pointCount) {
		std::cout << "Dropped " << (pointCount - cloud->size()) << " points due to NaN values." << "\n";
	}
	return cloud;
}

template <class PointT>
inline typename PCLTools<PointT>::CloudType::Ptr PCLTools<PointT>::loadPointCloudFromOBJ(fs::path cloudPath, optional<std::vector<Vector4f>&> colors) {
	typename CloudType::Ptr cloud(new CloudType());
	std::ifstream in(cloudPath.string().c_str());
	if (!in.good()) {
		throw std::runtime_error("Bad input stream");
	}

	std::string line;
	std::vector<std::tuple<float,float,float>> points, normals;
	char type[3];
	float v0, v1, v2;
	std::string typeStr;
	while (std::getline(in, line)) {
		if (sscanf(line.c_str(), "%2s %f %f %f", type, &v0, &v1, &v2) == 4) {
			typeStr = type;
			if (typeStr == "v") {
				points.push_back(std::make_tuple(v0,v1,v2));
			} else if (typeStr == "vn") {
				normals.push_back(std::make_tuple(v0,v1,v2));
			}
		}
	}
	in.close();

	PointT p;
	for (unsigned int i=0; i<points.size(); ++i) {
		std::tie(p.x, p.y, p.z) = points[i];
#ifndef OMIT_NORMALS
		std::tie(p.normal[0], p.normal[1], p.normal[2]) = normals[i];
#endif
		cloud->push_back(p);
	}

	if (colors) {
		colors.get().resize(cloud->size());
		std::fill(colors.get().begin(), colors.get().end(), Vector4f(0.5f, 0.5f, 0.5f, 1.f));
	}

	unsigned int pointCount = cloud->size();
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
    pcl::removeNaNNormalsFromPointCloud(*cloud, *cloud, indices);
	if (cloud->size() != pointCount) {
		std::cout << "Dropped " << (cloud->size() - pointCount) << " points due to NaN values." << "\n";
	}
	return cloud;
}

#ifdef USE_E57
template <class PointT>
inline typename PCLTools<PointT>::CloudType::Ptr PCLTools<PointT>::loadPointCloudFromE57(fs::path cloudPath, optional<std::vector<Vector4f>&> colors) {
	try {
		e57::Reader eReader(cloudPath.string());

		int scanCount = eReader.GetData3DCount();
		int imgCount = eReader.GetImage2DCount();

		if (colors) colors.get().clear();

		typename CloudType::Ptr cloud(new CloudType());
		for (int scanIndex = 0; scanIndex < scanCount; ++scanIndex) {
			e57::Data3D scanHeader;
			eReader.ReadData3D(scanIndex, scanHeader);

			Eigen::Vector4f translation(scanHeader.pose.translation.x, scanHeader.pose.translation.y, scanHeader.pose.translation.z, 0.f);
			Eigen::Quaternionf rotation(scanHeader.pose.rotation.w, scanHeader.pose.rotation.x, scanHeader.pose.rotation.y, scanHeader.pose.rotation.z);

			e57::Image2D imgHeader;
			char* imgBuffer = nullptr;
			std::function<Eigen::Vector2i (const Vector3f&)> project = nullptr;
			e57::Image2DProjection proj;
			fipImage* img = nullptr;
			if (colors && scanIndex < imgCount) {
				eReader.ReadImage2D(scanIndex, imgHeader);
				e57::Image2DType type, maskType, visType;
				int64_t w, h, size;
				eReader.GetImage2DSizes(scanIndex, proj, type, w, h, size, maskType, visType);
				imgBuffer = new char[size];
				eReader.ReadImage2DData(scanIndex, proj, type, (void*)imgBuffer, 0, size);
				fipMemoryIO memIO((BYTE*)imgBuffer, size);
				img = new fipImage(FIT_BITMAP, w, h, 3);
				img->loadFromMemory(memIO, type == e57::E57_JPEG_IMAGE ? FIF_JPEG : FIF_PNG);
				if (proj == e57::E57_PINHOLE) {
					e57::PinholeRepresentation rep = imgHeader.pinholeRepresentation;
					project = [&] (const Vector3f& p)->Eigen::Vector2i {
						if (std::abs(p[2]) < std::numeric_limits<float>::epsilon()) return Eigen::Vector2i(0, 0);
						int x = rep.principalPointX - (p[0] / p[2]) * (rep.focalLength / rep.pixelWidth);
						int y = rep.principalPointY - (p[1] / p[2]) * (rep.focalLength / rep.pixelHeight);
						return Eigen::Vector2i(x, y);
					};
				} else if (proj == e57::E57_SPHERICAL) {
					e57::SphericalRepresentation rep = imgHeader.sphericalRepresentation;
					project = [&] (const Vector3f& p)->Eigen::Vector2i {
						auto proj = p;//rotation * p + translation.head(3);
						float r = proj.norm();
						float theta = std::atan2(proj[1], proj[0]);
						float phi = std::asin(proj[2] / r);
						int x = rep.imageWidth / 2 - (theta / rep.pixelWidth);
						int y = rep.imageHeight / 2 - (phi / rep.pixelHeight);
						return Eigen::Vector2i(x, y);
					};
				} else if (proj == e57::E57_CYLINDRICAL) {
				} else {
				}
			}

			// get size info
			int64_t nColumn = 0, nRow = 0, nPointsSize = 0, nGroupsSize = 0, nCounts = 0; bool bColumnIndex = 0;
			eReader.GetData3DSizes( scanIndex, nRow, nColumn, nPointsSize, nGroupsSize, nCounts, bColumnIndex);

			int64_t nSize = (nRow > 0) ? nRow : 1024;

			double *xData = new double[nSize], *yData = new double[nSize], *zData = new double[nSize];//, *intensity = new double[nSize];

			auto dataReader = eReader.SetUpData3DPointsData(scanIndex, nSize, xData, yData, zData, nullptr, nullptr);

			unsigned long size = 0;
			typename CloudType::Ptr scan(new CloudType());
			while((size = dataReader.read()) > 0) {
				for(unsigned long i = 0; i < size; i++) {
					PointT p;
					p.x = xData[i];
					p.y = yData[i];
					p.z = zData[i];
					scan->push_back(p);
					RGBQUAD col;
					if (colors && imgBuffer && img && project) {
						auto imgCoords = project(p.getVector3fMap());
						img->getPixelColor(imgCoords[0], imgCoords[1], &col);
						colors.get().push_back(Vector4f(static_cast<float>(col.rgbRed) / 255.f, static_cast<float>(col.rgbGreen) / 255.f, static_cast<float>(col.rgbBlue) / 255.f, 1.f));
					}
				}
			}
			dataReader.close();
			scan->sensor_origin_ = translation;
			pcl::transformPointCloud(*scan, *scan, translation.head(3), rotation);
			cloud->insert(cloud->end(), scan->begin(), scan->end());

			delete [] xData;
			delete [] yData;
			delete [] zData;

			if (imgBuffer) {
				delete [] imgBuffer;
			}
			if (img) {
				delete img;
			}
		}


		unsigned int pointCount = cloud->size();
		std::vector<int> indices;
		pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
		//pcl::removeNaNNormalsFromPointCloud(*cloud, *cloud, indices);
		if (cloud->size() != pointCount) {
			std::cout << "Dropped " << (cloud->size() - pointCount) << " points due to NaN values." << "\n";
		}
		return cloud;
	} catch (...) {
		throw std::runtime_error("Could not load point cloud \""+cloudPath.string()+"\"");
	}
}
#endif // USE_E57

template <class PointT>
inline void PCLTools<PointT>::adjust(typename CloudType::Ptr cloud, std::string upAxis, float scale, bool recenter) {
	if (scale == 1.f && upAxis == "Z" && !recenter) return;

	Eigen::Matrix4f tC;
	if (recenter) {
		// compute center
		float cx = 0.f, cy = 0.f, cz = 0.f;
		int i=0;
		std::for_each(cloud->begin(), cloud->end(), [&] (PointT& p) {
			cx *= i; cy *= i; cz *= i++;
			cx += p.x; cy += p.y; cz += p.z;
			cx /= static_cast<float>(i); cy /= static_cast<float>(i); cz /= static_cast<float>(i);
		});
		tC << 1.f, 0.f, 0.f, -cx,
				0.f, 1.f, 0.f, -cy,
				0.f, 0.f, 1.f, -cz,
				0.f, 0.f, 0.f, 1.f;
	} else {
		tC = Matrix4f::Identity();
	}

	Eigen::Matrix4f tR;
	if (upAxis == "Y") {
		tR << 1.f, 0.f,  0.f, 0.f,
			   0.f, 0.f, -1.f, 0.f,
				0.f, 1.f,  0.f, 0.f,
				0.f, 0.f,  0.f, 1.f;
	} else if (upAxis == "X") {
		tR << 0.f, 0.f, -1.f, 0.f,
			   0.f, 1.f,  0.f, 0.f,
				1.f, 0.f,  0.f, 0.f,
				0.f, 0.f,  0.f, 1.f;
	} else if (upAxis == "Z") {
		tR = Eigen::Matrix4f::Identity();
	} else {
		// Unrecognized mesh orientation given, assuming Z axis
		tR = Eigen::Matrix4f::Identity();
	}

	pcl::transformPointCloudWithNormals(*cloud, *cloud, tR*tC);

	if (scale != 1.f) {
		Eigen::Matrix4f tS = Eigen::Matrix4f::Identity();
		tS.block<3,3>(0,0) *= scale;
		pcl::transformPointCloud(*cloud, *cloud, tS);
	}
}

template <class PointT>
inline typename PCLTools<PointT>::IdxSet PCLTools<PointT>::sampleUniform(typename CloudType::ConstPtr cloud, float leafSize, typename PCLTools<PointT>::SearchType::Ptr search) {
	pcl::UniformSampling<PointT> us;
	us.setInputCloud(cloud);
	if (search) us.setSearchMethod(search);
	us.setRadiusSearch(leafSize);
	pcl::PointCloud<int> subset;
	us.compute(subset);
	std::sort(subset.points.begin (), subset.points.end ());
	return IdxSet(subset.points.begin(), subset.points.end());
}

template <class PointT>
inline void PCLTools<PointT>::crop(typename CloudType::Ptr cloud, const IdxSet& subset) {
	Algorithm::removeIdx(*cloud, [&] (int idx) { return !std::binary_search(subset.begin(), subset.end(), idx); });
}

template <class PointT>
inline void PCLTools<PointT>::resample(typename CloudType::Ptr cloud, float leafSize) {
	PCLTools<PointT>::crop(cloud, PCLTools<PointT>::sampleUniform(cloud, leafSize));
}

template <class PointT>
inline float PCLTools<PointT>::resolution(typename CloudType::ConstPtr cloud, typename SearchType::ConstPtr search) {
	std::vector<int> indices(2);
	std::vector<float> sqrDists(2), dists(cloud->size());
	int idx = 0;
	for (const auto& p : *cloud) {
		search->nearestKSearch(p, 2, indices, sqrDists);
		dists[idx++] = sqrt(sqrDists[1]);
	}
	std::sort(dists.begin(), dists.end());
	return dists[dists.size() / 2];
}

template <class PointT>
typename PCLTools<PointT>::QuadricType::Ptr PCLTools<PointT>::fitQuadric(typename CloudType::ConstPtr cloud, const PointT& pos, NeighborQuery<PointT>& nq, Eigen::Matrix<float,3,3>* localBase) {
	Matrix3f transform;
	if (localBase) {
		transform = *localBase;
	} else {
		Eigen::Matrix<float, 3, 1> normal = pos.getNormalVector3fMap();
		transform.col(2) = normal;
		transform.col(0) = (Eigen::Matrix<float, 3, 3>::Identity() - normal*normal.transpose()).col(0).normalized();
		transform.col(1) = transform.col(2).cross(transform.col(1));
	}
	Vector3f params = localQuadricParams(cloud, nq, pos, transform);
	typename QuadricType::AMat Q;
	Q << params[0], 0.5f*params[2], 0.f,
	0.5f*params[2], params[1], 0.f,
	0.f, 0.f, 0.f;
	typename QuadricType::Ptr quadric(new QuadricType(Q, QuadricType::AVec::Zero(), 0.f));
	return quadric;
}

template <class PointT>
float PCLTools<PointT>::meanCurvature(typename CloudType::Ptr cloud, const NQ& nq, const PointT& pos, const IdxSet& subset) {
	// get nearest neighbors
	std::vector<int>  neighbors;
	std::vector<float> sqrDists;
	NQSearchVisitor nqVisitor(pos, nq.search, neighbors, sqrDists);
	boost::apply_visitor(nqVisitor, nq.param);
	if (subset.size()) {
		Algorithm::remove(neighbors, [&](int idx) { return !std::binary_search(subset.begin(), subset.end(), idx); });
	}
	if (!neighbors.size()) {
		std::cout << "Empty neighbor query" << "\n";
		return 0.f;
	}

	pcl::PrincipalCurvaturesEstimation<PointT, PointT> pce;
	auto pIt = cloud->insert(cloud->end(), pos);
	float x,y,z,p1,p2;
	pce.computePointPrincipalCurvatures(*cloud, cloud->size()-1, neighbors, x, y, z, p1, p2);
	cloud->erase(pIt);
	return 0.5f * (p1+p2);
	//Vector3f params = localQuadricParams(cloud, nq, pos, localBase);
	//return 0.5f * (params[0] + params[1]);
}

template <class PointT>
typename PCLTools<PointT>::Points PCLTools<PointT>::getNeighbors(typename CloudType::ConstPtr cloud, const PointT& pos, const NQ& nq) {
	IdxSet  neighbors = getNeighborIndices(cloud, pos, nq);
	std::vector<PointT> result(neighbors.size());
	std::transform(neighbors.begin(), neighbors.end(), result.begin(), [&] (unsigned int idx) { return cloud->points[idx]; });
	return result;
}

template <class PointT>
typename PCLTools<PointT>::IdxSet PCLTools<PointT>::getNeighborIndices(typename CloudType::ConstPtr cloud, const PointT& pos, const NQ& nq) {
	std::vector<int>  neighbors;
	std::vector<float> sqrDists;
	NQSearchVisitor nqVisitor(pos, nq.search, neighbors, sqrDists);
	boost::apply_visitor(nqVisitor, nq.param);
	return neighbors;
}

template <class PointT>
float PCLTools<PointT>::diameter(typename CloudType::ConstPtr cloud) {
	Eigen::AlignedBox<float,3> bb;
	for (const auto& p : *cloud) bb.extend(p.getVector3fMap());
	return bb.diagonal().norm();
}

template <class PointT>
Vector3f PCLTools<PointT>::localQuadricParams(typename CloudType::ConstPtr cloud, NeighborQuery<PointT>& nq, const PointT& pos, const Eigen::Matrix3f& localBase) {
	auto neighbors = getNeighborIndices(cloud, pos, nq);
	Vector3f vec = pos.getVector3fMap();
	// remove points too near the input position
	Algorithm::remove(neighbors, [&] (Idx idx) {
		Vector3f pnt = cloud->points[idx].getVector3fMap();
		return (std::isnan(pnt[0]) || std::isnan(pnt[1]) || std::isnan(pnt[2]) || (pnt - vec).norm() < 0.00001f);
	});
	if (!neighbors.size()) return Vector3f::Zero();

	// setup problem
	Eigen::MatrixXf A(neighbors.size(), 3);
	Eigen::VectorXf b(neighbors.size());
	unsigned int row = 0;
	for (const auto& p : neighbors) {
		Vector3f point = cloud->points[p].getVector3fMap();
		Vector3f proj = localBase*(point-vec);
		Vector3f r(proj[0]*proj[0], proj[1]*proj[1], proj[0]*proj[1]);
		if (std::isinf(r[0]) || std::isinf(r[1]) || std::isinf(r[2])) std::cout << "inf" << "\n";
		A.row(row) = r;
		b[row++] = proj[2];
	}
	// return solution
	auto solution = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
	return solution;
}

template <class PointT>
typename PCLTools<PointT>::IdxSet PCLTools<PointT>::cloudIndices(typename CloudType::ConstPtr cloud) {
	IdxSet all(cloud->size());
	std::iota(all.begin(), all.end(), 0);
	return all;
}


/// PCLTools::NQSearchVisitor ///

template <class PointT>
PCLTools<PointT>::NQSearchVisitor::NQSearchVisitor(const PointT& query, typename SearchType::Ptr search, std::vector<int>& indices, std::vector<float>& sqrDists) : boost::static_visitor<>(), m_query(query), m_search(search), m_indices(indices), m_sqrDists(sqrDists) {
}

template <class PointT>
PCLTools<PointT>::NQSearchVisitor::~NQSearchVisitor() {
}

template <class PointT>
void PCLTools<PointT>::NQSearchVisitor::operator()(float param) {
	m_search->radiusSearch(m_query, param, m_indices, m_sqrDists);
}

template <class PointT>
void PCLTools<PointT>::NQSearchVisitor::operator()(unsigned int param) {
	m_indices.resize(param);
	m_sqrDists.resize(param);
	m_search->nearestKSearch(m_query, param, m_indices, m_sqrDists);
}

/// PCLTools::NQSetVisitor ///

template <class PointT>
template <class Algo>
PCLTools<PointT>::NQSetVisitor<Algo>::NQSetVisitor(typename SearchType::Ptr search, Algo& algo) : boost::static_visitor<>(), m_search(search), m_algo(algo) {
}

template <class PointT>
template <class Algo>
PCLTools<PointT>::NQSetVisitor<Algo>::~NQSetVisitor() {
}

template <class PointT>
template <class Algo>
void PCLTools<PointT>::NQSetVisitor<Algo>::operator()(float param) {
	m_algo.setSearchMethod(m_search);
	m_algo.setRadiusSearch(param);
}

template <class PointT>
template <class Algo>
void PCLTools<PointT>::NQSetVisitor<Algo>::operator()(unsigned int param) {
	m_algo.setSearchMethod(m_search);
	m_algo.setKSearch(param);
}

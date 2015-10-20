template <class Scalar, unsigned int Dim>
Quadric<Scalar, Dim>::Quadric(const AMat& Q, const AVec& P, const Scalar& R) : m_Q(Q), m_P(P), m_R(R) {
}

template <class Scalar, unsigned int Dim>
Quadric<Scalar, Dim>::~Quadric() {
}

template <class Scalar, unsigned int Dim>
inline typename Quadric<Scalar, Dim>::AMat& Quadric<Scalar, Dim>::Q() {
	return m_Q;
}

template <class Scalar, unsigned int Dim>
inline typename Quadric<Scalar, Dim>::AVec& Quadric<Scalar, Dim>::P() {
	return m_P;
}

template <class Scalar, unsigned int Dim>
inline Scalar& Quadric<Scalar, Dim>::R() {
	return m_R;
}

template <class Scalar, unsigned int Dim>
inline const typename Quadric<Scalar, Dim>::AMat& Quadric<Scalar, Dim>::Q() const {
	return m_Q;
}

template <class Scalar, unsigned int Dim>
inline const typename Quadric<Scalar, Dim>::AVec& Quadric<Scalar, Dim>::P() const {
	return m_P;
}

template <class Scalar, unsigned int Dim>
inline const Scalar& Quadric<Scalar, Dim>::R() const {
	return m_R;
}

template <class Scalar, unsigned int Dim>
inline Scalar Quadric<Scalar, Dim>::operator()(const AVec& vec) const {
	return vec.dot(m_Q*vec) + m_P.dot(vec) + m_R;
}

template <class Scalar, unsigned int Dim>
inline typename Quadric<Scalar, Dim>::PrincipalCurvatures Quadric<Scalar, Dim>::principalCurvatures() const {
	DMat mat = m_Q.block(0,0,Dim,Dim);
	Eigen::SelfAdjointEigenSolver<DMat> solver(mat);
	return {solver.eigenvalues(), solver.eigenvectors()};
}

template <class Scalar, unsigned int Dim>
inline Scalar Quadric<Scalar, Dim>::meanCurvature() const {
	static_assert(Dim == 2, "Mean curvature may only be computed for 2D-quadrics");
	return principalCurvatures().curvatures.mean();
}

template <class Scalar, unsigned int Dim>
inline Scalar Quadric<Scalar, Dim>::gaussCurvature() const {
	static_assert(Dim == 2, "Gauss curvature may only be computed for 2D-quadrics");
	return principalCurvatures().curvatures.prod();
}

#ifndef COMMONSQUADRIC_H_
#define COMMONSQUADRIC_H_

#include "../Testing/asserts.h"
#include <Eigen/Dense>

namespace Geometry {

template <class Scalar = float, unsigned int Dim = 2>
class Quadric {
	public:
		typedef std::shared_ptr<Quadric> Ptr;
		typedef std::weak_ptr<Quadric>   WPtr;

		static const unsigned int ADim = Dim + 1;
		//constexpr unsigned int ADim = Dim+1;
		typedef Eigen::Matrix<Scalar, ADim, ADim> AMat;
		typedef Eigen::Matrix<Scalar,  Dim,  Dim> DMat;
		typedef Eigen::Matrix<Scalar, ADim,    1> AVec;
		typedef Eigen::Matrix<Scalar,  Dim,    1> DVec;
		struct PrincipalCurvatures {
			DVec curvatures;
			DMat direction;
		};

	public:
		Quadric(const AMat& Q, const AVec& P, const Scalar& R);
		virtual ~Quadric();

		AMat&   Q();
		AVec&   P();
		Scalar& R();
		const AMat&   Q() const;
		const AVec&   P() const;
		const Scalar& R() const;

		Scalar operator()(const AVec& vec) const;

		PrincipalCurvatures principalCurvatures() const;
		Scalar              meanCurvature() const;
		Scalar              gaussCurvature() const;

	protected:
		AMat    m_Q;
		AVec    m_P;
		Scalar  m_R;

};

#include "Quadric.inl"

} // Geometry

#endif /* COMMONSQUADRIC_H_ */

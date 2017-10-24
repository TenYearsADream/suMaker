#include "half_space_box.h"
#include "assign_scalar.h"
#include <CGAL/Point_3.h>
#include <CGAL/Vector_3.h>

template <typename DerivedV>
IGL_INLINE void igl::copyleft::cgal::half_space_box(
  const CGAL::Plane_3<CGAL::Epeck> & P,
  const Eigen::PlainObjectBase<DerivedV> & V,
  Eigen::Matrix<CGAL::Epeck::FT,8,3> & BV,
  Eigen::Matrix<int,12,3> & BF)
{
  typedef CGAL::Plane_3<CGAL::Epeck> Plane;
  typedef CGAL::Point_3<CGAL::Epeck> Point;
  typedef CGAL::Vector_3<CGAL::Epeck> Vector;
  typedef CGAL::Epeck::FT EScalar;
  Eigen::RowVector3d avg = V.colwise().mean();
  Point o3(avg(0),avg(1),avg(2));
  Point o2 = P.projection(o3);
  Vector u;
  EScalar max_sqrd = -1;
  for(int v = 0;v<V.rows();v++)
  {
    Vector v2 = P.projection(Point(V(v,0),V(v,1),V(v,2))) - o2;
    const EScalar sqrd = v2.squared_length();
    if(max_sqrd<0 || sqrd < max_sqrd)
    {
      u = v2;
      max_sqrd = sqrd;
    }
  }
  // L1 bbd 
  const EScalar bbd = 
    (EScalar(V.col(0).maxCoeff())- EScalar(V.col(0).minCoeff())) + 
    (EScalar(V.col(1).maxCoeff())- EScalar(V.col(1).minCoeff())) + 
    (EScalar(V.col(2).maxCoeff())- EScalar(V.col(2).minCoeff()));
  Vector n = P.orthogonal_vector();
  // now we have a center o2 and a vector u to the farthest point on the plane 
  std::vector<Point> vBV;vBV.reserve(8);
  Vector v = CGAL::cross_product(u,n);
  // Scale u,v,n to be longer than bbd
  const auto & longer_than = [](const EScalar min_sqr, Vector & x)
  {
    assert(x.squared_length() > 0);
    while(x.squared_length() < min_sqr)
    {
      x = 2.*x;
    }
  };
  longer_than(bbd*bbd,u);
  longer_than(bbd*bbd,v);
  longer_than(bbd*bbd,n);
  vBV.emplace_back( o2 + u + v);
  vBV.emplace_back( o2 - u + v);
  vBV.emplace_back( o2 - u - v);
  vBV.emplace_back( o2 + u - v);
  vBV.emplace_back( o2 + u + v - n);
  vBV.emplace_back( o2 - u + v - n);
  vBV.emplace_back( o2 - u - v - n);
  vBV.emplace_back( o2 + u - v - n);
  BV.resize(8,3);
  for(int b = 0;b<8;b++)
  {
    igl::copyleft::cgal::assign_scalar(vBV[b].x(),BV(b,0));
    igl::copyleft::cgal::assign_scalar(vBV[b].y(),BV(b,1));
    igl::copyleft::cgal::assign_scalar(vBV[b].z(),BV(b,2));
  }
  BF.resize(12,3);
  BF<<
    1,0,2,
    2,0,3,
    4,5,6,
    4,6,7,
    0,1,4,
    4,1,5,
    1,2,5,
    5,2,6,
    2,3,6,
    6,3,7,
    3,0,7,
    7,0,4;
}

template <typename Derivedp, typename Derivedn, typename DerivedV>
IGL_INLINE void igl::copyleft::cgal::half_space_box(
  const Eigen::PlainObjectBase<Derivedp> & p,
  const Eigen::PlainObjectBase<Derivedn> & n,
  const Eigen::PlainObjectBase<DerivedV> & V,
  Eigen::Matrix<CGAL::Epeck::FT,8,3> & BV,
  Eigen::Matrix<int,12,3> & BF)
{
  typedef CGAL::Plane_3<CGAL::Epeck> Plane;
  typedef CGAL::Point_3<CGAL::Epeck> Point;
  typedef CGAL::Vector_3<CGAL::Epeck> Vector;
  Plane P(Point(p(0),p(1),p(2)),Vector(n(0),n(1),n(2)));
  return half_space_box(P,V,BV,BF);
}

template <typename Derivedequ, typename DerivedV>
IGL_INLINE void igl::copyleft::cgal::half_space_box(
  const Eigen::PlainObjectBase<Derivedequ> & equ,
  const Eigen::PlainObjectBase<DerivedV> & V,
  Eigen::Matrix<CGAL::Epeck::FT,8,3> & BV,
  Eigen::Matrix<int,12,3> & BF)
{
  typedef CGAL::Plane_3<CGAL::Epeck> Plane;
  Plane P(equ(0),equ(1),equ(2),equ(3));
  return half_space_box(P,V,BV,BF);
}
#ifdef IGL_STATIC_LIBRARY
// Explicit template specialization
template void igl::copyleft::cgal::half_space_box<Eigen::Matrix<double, 1, 3, 1, 1, 3>, Eigen::Matrix<double, 1, 3, 1, 1, 3>, Eigen::Matrix<double, -1, -1, 0, -1, -1> >(Eigen::PlainObjectBase<Eigen::Matrix<double, 1, 3, 1, 1, 3> > const&, Eigen::PlainObjectBase<Eigen::Matrix<double, 1, 3, 1, 1, 3> > const&, Eigen::PlainObjectBase<Eigen::Matrix<double, -1, -1, 0, -1, -1> > const&, Eigen::Matrix<CGAL::Lazy_exact_nt<CGAL::Gmpq>, 8, 3, 0, 8, 3>&, Eigen::Matrix<int, 12, 3, 0, 12, 3>&);
#endif

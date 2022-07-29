#include <iostream>
#include <vector>
#include "MeshDefinition.h"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SVD>
#include <Eigen/LU>
#include <Eigen/QR>

#include <boost/date_time/gregorian/gregorian.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <CGAL/Simple_cartesian.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_triangle_primitive.h>

using namespace Eigen;
using namespace std;
using namespace boost::gregorian;
using namespace boost::posix_time;

typedef CGAL::Simple_cartesian<double> K;

typedef K::FT FT;
typedef K::Ray_3 Ray;
typedef K::Line_3 Line;
typedef K::Point_3 Point;
typedef K::Triangle_3 Triangle;
typedef std::list<Triangle>::iterator Iterator;
typedef CGAL::AABB_triangle_primitive<K, Iterator> Primitive;
typedef CGAL::AABB_traits<K, Primitive> AABB_triangle_traits;
typedef CGAL::AABB_tree<AABB_triangle_traits> Tree;

class Remeshing
{
public:
	Remeshing();
	~Remeshing();

	double get_Dist_2_of_Points(Mesh::Point point_1, Mesh::Point point_2);
	double get_Dist_of_Points(Mesh::Point point_1, Mesh::Point point_2);
	
	double get_Norm_of_Point(Mesh::Point point);
	double get_Norm_of_Vector(std::vector<double> x);

	bool is_In_Set(std::vector<int> set, int data);

	double get_Radian_of_Three_Points(Mesh::Point point_1, Mesh::Point point_mid, Mesh::Point point_2);

	double get_Cot_of_Three_Points(Mesh::Point point_1, Mesh::Point point_mid, Mesh::Point point_2);

	double get_Area_of_Three_Points(Mesh::Point point_1, Mesh::Point point_2, Mesh::Point point_3);

	double get_Area_of_Four_Points(Mesh::Point point_1, Mesh::Point point_2, Mesh::Point point_3, Mesh::Point point_4);

	double get_Area_of_Face_by_Face_Idx(Mesh& mesh, int fh_idx);

	std::vector<double> get_Norm_Vector_of_Triangle_by_Three_Points(Mesh::Point point_1, Mesh::Point point_2, Mesh::Point point_3);

	std::vector<double> get_Norm_Vector_of_Triangle_by_Face_Idx(Mesh& mesh, int fh_idx);

	std::vector<double> get_Center_of_Triangle_by_Face_Idx(Mesh& mesh, int fh_idx);

	std::vector<int> get_Neighboor_Face_Idx_Set(Mesh& mesh, int fh_idx);

	std::vector<std::vector<int>> get_FVH_Idx_Set(Mesh& mesh);

	std::vector<std::vector<int>> get_VFH_Idx_Set(Mesh& mesh);

	std::vector<double> get_Face_Area_Set(Mesh& mesh);

	std::vector<double> get_Local_Average_Area_Set_of_VH(Mesh& mesh);

	void get_Face_Local_Basis_Set(Mesh& mesh);

	void get_Face_Bary_Center_Set(Mesh& mesh);

	void get_Global_Edge_Scale(Mesh& mesh);

	Matrix<double, 1, 3> get_Vector3d_from_VH(Mesh& mesh, int vh_idx);

	MatrixXd random_constraints(const Matrix<double, 1, 3>& b1, const Matrix<double, 1, 3>& b2, int n);

	bool split_long_edges(Mesh& mesh, double high);

	bool collapse_short_edges(Mesh& mesh, double low, double high);

	bool equalize_valences(Mesh& mesh);

	bool tangential_relaxation(Mesh& mesh);

	bool projecct_to_surface(Mesh& mesh, Tree& tree);

	bool get_Remeshing_Result(Mesh& mesh, double target_edge_length);

public:
	std::vector<std::vector<Matrix<double, 1, 3>>> face_local_basis_set;
	std::vector<Matrix<double, 1, 3>> face_bary_center_set;
	double global_edge_scale;
	std::vector<int> combinations;
};
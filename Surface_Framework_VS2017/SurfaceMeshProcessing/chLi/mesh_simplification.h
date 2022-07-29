#include <iostream>
#include <vector>
#include "MeshDefinition.h"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SVD>
#include <Eigen/LU>
#include <Eigen/QR>

using namespace Eigen;

class Mesh_Simplification
{
public:
	Mesh_Simplification();
	~Mesh_Simplification();

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

	std::vector<Vector4d> get_Norm_Tuta_Set(Mesh& mesh);

	std::vector<Matrix4d> get_Face_Quadratic_error_Matrix_Set(Mesh& mesh);

	std::vector<Matrix4d> get_Vertex_Quadratic_error_Matrix_Set(Mesh& mesh);

	std::vector<Matrix4d> get_Edge_Quadratic_error_Matrix_Set(Mesh& mesh);

	std::vector<double> get_Edge_Quadratic_error_Set(Mesh& mesh);

	std::vector<double> get_Edge_Quadratic_error_Set(Mesh& mesh, std::vector<Matrix4d> edge_quadratic_error_matrix_set);

	bool get_Mesh_Simplification_Result(Mesh& mesh, int target_vertex_num);
};
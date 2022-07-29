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

class Mesh_Parameterization_2
{
public:
	Mesh_Parameterization_2();
	~Mesh_Parameterization_2();

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

	std::vector<std::vector<double>> get_Parameterization_2D(Mesh& mesh);

	std::vector<std::vector<std::vector<double>>> get_Source_Triangle_Position_2D(Mesh& mesh);

	std::vector<std::vector<std::vector<double>>> get_Face_Transformation_Matrix(Mesh& mesh, std::vector<std::vector<double>> param_2d, std::vector<std::vector<std::vector<double>>> source_triangle_position_2d);

	std::vector<std::vector<std::vector<double>>> get_Face_Target_Transformation_Matrix(std::vector<std::vector<std::vector<double>>> face_transformation_matrix);

	std::vector<std::vector<std::vector<double>>> get_Source_Triangle_Position_Matrix_Inverse(std::vector<std::vector<std::vector<double>>> source_triangle_position_2d);

	std::vector<std::vector<int>> get_FVH_Idx_Set(Mesh& mesh);

	std::vector<std::vector<int>> get_VFH_Idx_Set(Mesh& mesh);

	std::vector<double> get_Face_Area_Set(Mesh& mesh);

	std::vector<std::vector<double>> Update_VH_Position(Mesh& mesh, int iteration_num);
};
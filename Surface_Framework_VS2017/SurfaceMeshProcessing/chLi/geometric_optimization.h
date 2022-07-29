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
using namespace std;

class Geometric_Optimization
{
public:
	Geometric_Optimization();
	~Geometric_Optimization();

	double get_Dist_2_of_Points(Mesh::Point point_1, Mesh::Point point_2);
	double get_Dist_of_Points(Mesh::Point point_1, Mesh::Point point_2);

	double get_Dist_2_of_Vector(VectorXd& vector_1, VectorXd& vector_2);
	double get_Dist_of_Vector(VectorXd& vector_1, VectorXd& vector_2);

	double get_Dist_2_of_Vector(RowVectorXd& vector_1, RowVectorXd& vector_2);
	double get_Dist_of_Vector(RowVectorXd& vector_1, RowVectorXd& vector_2);
	
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

	vector<MatrixXd> get_Face_Target_USwUT_Matrix(std::vector<std::vector<std::vector<double>>> face_transformation_matrix);

	vector<MatrixXd> get_Source_Triangle_Position_Matrix_Inverse(std::vector<std::vector<std::vector<double>>> source_triangle_position_2d);

	std::vector<std::vector<int>> get_FVH_Idx_Set(Mesh& mesh);

	std::vector<std::vector<int>> get_VFH_Idx_Set(Mesh& mesh);

	std::vector<double> get_Face_Area_Set(Mesh& mesh);

	std::vector<double> get_Local_Average_Area_Set_of_VH(Mesh& mesh);

	Matrix<double, 1, 3> get_Vector3d_from_VH(Mesh& mesh, int vh_idx);

	MatrixXd get_Circum_Circle_Center(Mesh& mesh);

	bool update_Delaunay_Triangulation(Mesh& mesh);

	bool update_Vertex_Position_By_Circum_Center(Mesh& mesh);

	std::vector<std::vector<double>> Update_VH_Position(Mesh& mesh, int iteration_num);
};
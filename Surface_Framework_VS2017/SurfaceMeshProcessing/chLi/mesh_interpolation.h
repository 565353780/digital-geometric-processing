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

class Mesh_Interpolation
{
public:
	Mesh_Interpolation();
	~Mesh_Interpolation();

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

	std::vector<std::vector<double>> get_Cot_Weight_Set(Mesh& mesh);

	bool update_Source_Mesh(Mesh& mesh);

	bool update_Target_Mesh(Mesh& mesh);

	bool update_Matrix();

	bool get_Matrix_and_Radian(std::vector<std::vector<int>> fvh_idx_set);

	bool get_A_t(double t);

	std::vector<std::vector<double>> get_Mesh_Interpolation_Result(Mesh& mesh, int method, double t);

private:
	std::vector<std::vector<double>> vh_source_mesh_interpolation_set;
	std::vector<std::vector<double>> vh_target_mesh_interpolation_set;

	std::vector<Matrix2d> Matrix_A_set;
	std::vector<Matrix2d> Matrix_U_set;
	std::vector<Matrix2d> Matrix_V_set;
	std::vector<Matrix2d> Matrix_E_set;
	std::vector<Matrix2d> Matrix_R_set;
	std::vector<Matrix2d> Matrix_S_set;
	std::vector<double> Rotation_Radian_set;

	std::vector<Matrix2d> Matrix_A_t_set;
	std::vector<Matrix3d> Matrix_P_inverse_set;

	bool need_to_update_matrix;
	bool need_to_update_matrix_set;
	MatrixXd A, x, U, V, E, R, S;
	VectorXd Sigma;
	MatrixXd A_t_inverse;
};
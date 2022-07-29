#include <iostream>
#include <vector>
#include "MeshDefinition.h"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SVD>
#include <Eigen/LU>
#include <Eigen/QR>
#include <Eigen/Geometry>
#include <unsupported/Eigen/SparseExtra>

using namespace Eigen;

class Cross_Fields
{
public:
	Cross_Fields();
	~Cross_Fields();

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

	VectorXd get_K(Mesh& mesh);

	void get_General_Coeff_Constraints(Mesh& mesh, const VectorXi& isConstrained, const Matrix<double, Dynamic, Dynamic>& cfW, int k, Matrix<std::complex<double>, Dynamic, 1>& Ck);

	void get_General_Coeff_Constraints(Mesh& mesh, const VectorXi& isConstrained, const Matrix<double, Dynamic, Dynamic>& cfW, int k, const VectorXi& rootsIndex, Matrix<std::complex<double>, Dynamic, 1>& Ck);

	void doCombs(int offset, int k, int N, std::vector<std::vector<int>>& allCombs);

	void get_Coefficient_Laplacian(Mesh& mesh, int n, VectorXd K, SparseMatrix<std::complex<double>>& D);

	void slice(const SparseMatrix<std::complex<double>>& X, const Matrix<int, Dynamic, 1>& R, const Matrix<int, Dynamic, 1>& C, SparseMatrix<std::complex<double>>& Y);

	void min_Quad_With_Known_Mini(const SparseMatrix<std::complex<double>>& Q, const SparseMatrix<std::complex<double>>& f, const VectorXi isConstrained, const Matrix<std::complex<double>, Dynamic, 1>& xknown, Matrix<std::complex<double>, Dynamic, 1>& x);

	void set_Field_From_General_Coefficients(Mesh& mesh, int num, const std::vector<Matrix<std::complex<double>, Dynamic, 1>>& coeffs, std::vector<Matrix<double, Eigen::Dynamic, 2>>& pv);

	bool get_Cross_Fields_Result(Mesh& mesh, MatrixXd& output, int num);

	bool get_General_Cross_Fields_Result(Mesh& mesh, MatrixXd& output, int num);

public:
	std::vector<std::vector<Matrix<double, 1, 3>>> face_local_basis_set;
	std::vector<Matrix<double, 1, 3>> face_bary_center_set;
	double global_edge_scale;
	std::vector<int> combinations;
};
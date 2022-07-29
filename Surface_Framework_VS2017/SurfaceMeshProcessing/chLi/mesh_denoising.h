#include <iostream>
#include <vector>
#include "MeshDefinition.h"

class Mesh_Denoising
{
public:
	Mesh_Denoising();
	~Mesh_Denoising();

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

	std::vector<double> update_Face_Normal_by_Face_Idx(Mesh& mesh, int fh_idx, double sigma_space, double sigma_normal);

	std::vector<std::vector<double>> update_Face_Normal(Mesh& mesh, int fh_normal_iteration_num, double sigma_space, double sigma_normal);

	bool update_Vertex_Position(Mesh& mesh, int fh_normal_iteration_num, int vh_position_iteration_num, double sigma_space, double sigma_normal);
};
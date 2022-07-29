#include <iostream>
#include <vector>
#include "MeshDefinition.h"  

enum SolveMode
{
	Free = 0,
	MeanCurvature = 1,
	AbsoluteMeanCurvature = 2,
	GaussianCurvature = 3
};

class Discrete_Curvature
{
public:
	Discrete_Curvature();
	~Discrete_Curvature();

	double get_Dist_2_of_Points(Mesh::Point point_1, Mesh::Point point_2);
	double get_Dist_of_Points(Mesh::Point point_1, Mesh::Point point_2);
	
	double get_Norm_of_Point(Mesh::Point point);
	double get_Norm_of_Vector(std::vector<double> x);

	bool is_In_Set(std::vector<int> set, int data);

	double get_Radian_of_Three_Points(Mesh::Point point_1, Mesh::Point point_mid, Mesh::Point point_2);

	double get_Cot_of_Three_Points(Mesh::Point point_1, Mesh::Point point_mid, Mesh::Point point_2);

	double get_Area_of_Three_Points(Mesh::Point point_1, Mesh::Point point_2, Mesh::Point point_3);

	double get_Area_of_Four_Points(Mesh::Point point_1, Mesh::Point point_2, Mesh::Point point_3, Mesh::Point point_4);

	double get_Mean_Curvature_of_Vertex(Mesh& mesh, int vh_idx);

	double get_Absolute_Mean_Curvature_of_Vertex(Mesh& mesh, int vh_idx);

	double get_Gaussian_Curvature_of_Vertex(Mesh& mesh, int vh_idx);

	std::vector<double> get_RGB_of_Color_Bar_Jet(double value, double value_min, double value_max);
};
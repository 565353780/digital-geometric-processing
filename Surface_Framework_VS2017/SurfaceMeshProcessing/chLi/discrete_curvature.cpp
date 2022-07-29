#include "discrete_curvature.h"

Discrete_Curvature::Discrete_Curvature()
{

}

Discrete_Curvature::~Discrete_Curvature()
{

}

double Discrete_Curvature::get_Dist_2_of_Points(Mesh::Point point_1, Mesh::Point point_2)
{
	double dist_2_of_points = 0;

	dist_2_of_points += (point_1.data()[0] - point_2.data()[0]) * (point_1.data()[0] - point_2.data()[0]);
	dist_2_of_points += (point_1.data()[1] - point_2.data()[1]) * (point_1.data()[1] - point_2.data()[1]);
	dist_2_of_points += (point_1.data()[2] - point_2.data()[2]) * (point_1.data()[2] - point_2.data()[2]);

	return dist_2_of_points;
}

double Discrete_Curvature::get_Dist_of_Points(Mesh::Point point_1, Mesh::Point point_2)
{
	return sqrt(get_Dist_2_of_Points(point_1, point_2));
}

double Discrete_Curvature::get_Norm_of_Point(Mesh::Point point)
{
	Mesh::Point zero(0, 0, 0);

	return sqrt(get_Dist_2_of_Points(point, zero));
}

double Discrete_Curvature::get_Norm_of_Vector(std::vector<double> x)
{
	double dist_to_zero = x[0] * x[0] + x[1] * x[1] + x[2] * x[2];

	return sqrt(dist_to_zero);
}

bool Discrete_Curvature::is_In_Set(std::vector<int> set, int data)
{
	for (int i = 0; i < set.size(); ++i)
	{
		if (set[i] == data)
		{
			return true;
		}
	}

	return false;
}

double Discrete_Curvature::get_Radian_of_Three_Points(Mesh::Point point_1, Mesh::Point point_mid, Mesh::Point point_2)
{
	std::vector<double> x_1;
	std::vector<double> x_2;

	x_1.emplace_back(point_1.data()[0] - point_mid.data()[0]);
	x_1.emplace_back(point_1.data()[1] - point_mid.data()[1]);
	x_1.emplace_back(point_1.data()[2] - point_mid.data()[2]);

	x_2.emplace_back(point_2.data()[0] - point_mid.data()[0]);
	x_2.emplace_back(point_2.data()[1] - point_mid.data()[1]);
	x_2.emplace_back(point_2.data()[2] - point_mid.data()[2]);

	double dot = x_1[0] * x_2[0] + x_1[1] * x_2[1] + x_1[2] * x_2[2];

	double x_1_norm = get_Norm_of_Vector(x_1);
	double x_2_norm = get_Norm_of_Vector(x_2);

	if (x_1_norm == 0 || x_2_norm == 0)
	{
		return 0;
	}

	return acos(dot / x_1_norm / x_2_norm);
}

double Discrete_Curvature::get_Cot_of_Three_Points(Mesh::Point point_1, Mesh::Point point_mid, Mesh::Point point_2)
{
	double tan_of_three_points = tan(get_Radian_of_Three_Points(point_1, point_mid, point_2));

	if (tan_of_three_points != 0)
	{
		return 1.0 / tan_of_three_points;
	}

	std::cout << "tan is zero!" << std::endl;

	return 0;
}

double Discrete_Curvature::get_Area_of_Three_Points(Mesh::Point point_1, Mesh::Point point_2, Mesh::Point point_3)
{
	std::vector<double> x_1;
	std::vector<double> x_2;
	std::vector<double> x_3;

	x_1.emplace_back(point_2.data()[0] - point_3.data()[0]);
	x_1.emplace_back(point_2.data()[1] - point_3.data()[1]);
	x_1.emplace_back(point_2.data()[2] - point_3.data()[2]);

	x_2.emplace_back(point_1.data()[0] - point_3.data()[0]);
	x_2.emplace_back(point_1.data()[1] - point_3.data()[1]);
	x_2.emplace_back(point_1.data()[2] - point_3.data()[2]);

	x_3.emplace_back(point_2.data()[0] - point_1.data()[0]);
	x_3.emplace_back(point_2.data()[1] - point_1.data()[1]);
	x_3.emplace_back(point_2.data()[2] - point_1.data()[2]);

	double x_1_len = get_Norm_of_Vector(x_1);
	double x_2_len = get_Norm_of_Vector(x_2);
	double x_3_len = get_Norm_of_Vector(x_3);

	double p = (x_1_len + x_2_len + x_3_len) / 2.0;

	return sqrt(p * (p - x_1_len) * (p - x_2_len) * (p - x_3_len));
}

double Discrete_Curvature::get_Area_of_Four_Points(Mesh::Point point_1, Mesh::Point point_2, Mesh::Point point_3, Mesh::Point point_4)
{
	return get_Area_of_Three_Points(point_1, point_2, point_3) + get_Area_of_Three_Points(point_1, point_3, point_4);
}

double Discrete_Curvature::get_Mean_Curvature_of_Vertex(Mesh& mesh, int vh_idx)
{
	Mesh::VertexHandle current_vh = mesh.vertex_handle(vh_idx);

	std::vector<int> vh_neighboor_idx_set;

	Mesh::VertexVertexIter vvit;

	bool finished = false;

	for (vvit = mesh.vv_iter(current_vh); vvit->is_valid(); ++vvit)
	{
		if (vvit->idx() == mesh.vv_iter(current_vh)->idx())
		{
			if (!finished)
			{
				finished = true;
			}
			else
			{
				break;
			}
		}

		vh_neighboor_idx_set.emplace_back(vvit->idx());
	}

	int vh_neighboor_idx_set_size = vh_neighboor_idx_set.size();

	std::vector<double> laplace;
	laplace.resize(3);
	for (int i = 0; i < laplace.size(); ++i)
	{
		laplace[i] = 0;
	}

	double area = 0;

	for (int i = 0; i < vh_neighboor_idx_set_size; ++i)
	{
		Mesh::VertexHandle vh_1 = mesh.vertex_handle(vh_neighboor_idx_set[(i - 1 + vh_neighboor_idx_set_size) % vh_neighboor_idx_set_size]);
		Mesh::VertexHandle vh_2 = mesh.vertex_handle(vh_neighboor_idx_set[i % vh_neighboor_idx_set_size]);
		Mesh::VertexHandle vh_3 = mesh.vertex_handle(vh_neighboor_idx_set[(i + 1) % vh_neighboor_idx_set_size]);

		/*double current_vh_radian = get_Radian_of_Three_Points(mesh.point(vh_2), mesh.point(current_vh), mesh.point(vh_3));
		double vh_3_radian = get_Radian_of_Three_Points(mesh.point(current_vh), mesh.point(vh_3), mesh.point(vh_2));
		double vh_2_radian = get_Radian_of_Three_Points(mesh.point(vh_3), mesh.point(vh_2), mesh.point(current_vh));

		if (current_vh_radian > M_PI_2)
		{

		}*/

		Mesh::Point average_point_to_vh_2 = (mesh.point(current_vh) + mesh.point(vh_3)) / 2.0;
		Mesh::Point average_point_of_three_points = (mesh.point(current_vh) + mesh.point(vh_2) + mesh.point(vh_3)) / 3.0;
		Mesh::Point average_point_to_vh_3 = (mesh.point(current_vh) + mesh.point(vh_2)) / 2.0;

		area += get_Area_of_Four_Points(mesh.point(current_vh), average_point_to_vh_2, average_point_of_three_points, average_point_to_vh_3);

		double cot_sum = get_Cot_of_Three_Points(mesh.point(current_vh), mesh.point(vh_1), mesh.point(vh_2)) + get_Cot_of_Three_Points(mesh.point(current_vh), mesh.point(vh_3), mesh.point(vh_2));

		for (int j = 0; j < laplace.size(); ++j)
		{
			laplace[j] += cot_sum * (mesh.point(vh_2)[j] - mesh.point(current_vh)[j]);
		}
	}

	if (area == 0)
	{
		std::cout << "area is zero!" << std::endl;

		return 0;
	}

	std::vector<double> vh_norm_vector;
	vh_norm_vector.resize(3);

	Mesh::Point current_point = mesh.point(current_vh);

	for (int i = 0; i < vh_neighboor_idx_set_size; ++i)
	{
		Mesh::HalfedgeHandle current_heh = mesh.find_halfedge(current_vh, mesh.vertex_handle(vh_neighboor_idx_set[i]));
		Mesh::HalfedgeHandle next_heh = mesh.next_halfedge_handle(current_heh);

		if (!mesh.is_boundary(current_heh) || !mesh.is_boundary(next_heh))
		{
			Mesh::Point point_1 = mesh.point(mesh.to_vertex_handle(current_heh));
			Mesh::Point point_2 = mesh.point(mesh.to_vertex_handle(next_heh));

			std::vector<double> v_1, v_2;
			v_1.resize(3);
			v_2.resize(3);

			for (int j = 0; j < 3; ++j)
			{
				v_1[j] = (point_1 - current_point).data()[j];
				v_2[j] = (point_2 - point_1).data()[j];
			}

			for (int j = 0; j < 3; ++j)
			{
				vh_norm_vector[j] += v_1[(j + 1) % 3] * v_2[(j + 2) % 3] - v_1[(j + 2) % 3] * v_2[(j + 1) % 3];
			}
		}
	}

	double vh_norm_vector_norm = vh_norm_vector[0] * vh_norm_vector[0] + vh_norm_vector[1] * vh_norm_vector[1] + vh_norm_vector[2] * vh_norm_vector[2];

	vh_norm_vector_norm = sqrt(vh_norm_vector_norm);

	double mean_curvature = laplace[0] * vh_norm_vector[0] + laplace[1] * vh_norm_vector[1] + laplace[2] * vh_norm_vector[2];

	mean_curvature /= vh_norm_vector_norm;

	return mean_curvature / 2.0 / area;
}

double Discrete_Curvature::get_Absolute_Mean_Curvature_of_Vertex(Mesh& mesh, int vh_idx)
{
	return abs(get_Mean_Curvature_of_Vertex(mesh, vh_idx));
}

double Discrete_Curvature::get_Gaussian_Curvature_of_Vertex(Mesh& mesh, int vh_idx)
{
	Mesh::VertexHandle current_vh = mesh.vertex_handle(vh_idx);

	std::vector<int> vh_neighboor_idx_set;

	Mesh::VertexVertexIter vvit;

	bool finished = false;

	for (vvit = mesh.vv_iter(current_vh); vvit->is_valid(); ++vvit)
	{
		if (vvit->idx() == mesh.vv_iter(current_vh)->idx())
		{
			if (!finished)
			{
				finished = true;
			}
			else
			{
				break;
			}
		}

		vh_neighboor_idx_set.emplace_back(vvit->idx());
	}

	int vh_neighboor_idx_set_size = vh_neighboor_idx_set.size();

	double radian_sum = 0;
	double area = 0;

	for (int i = 0; i < vh_neighboor_idx_set_size; ++i)
	{
		Mesh::VertexHandle vh_2 = mesh.vertex_handle(vh_neighboor_idx_set[i % vh_neighboor_idx_set_size]);
		Mesh::VertexHandle vh_3 = mesh.vertex_handle(vh_neighboor_idx_set[(i + 1) % vh_neighboor_idx_set_size]);

		Mesh::Point average_point_to_vh_2 = (mesh.point(current_vh) + mesh.point(vh_3)) / 2.0;
		Mesh::Point average_point_of_three_points = (mesh.point(current_vh) + mesh.point(vh_2) + mesh.point(vh_3)) / 3.0;
		Mesh::Point average_point_to_vh_3 = (mesh.point(current_vh) + mesh.point(vh_2)) / 2.0;

		area += get_Area_of_Four_Points(mesh.point(current_vh), average_point_to_vh_2, average_point_of_three_points, average_point_to_vh_3);

		radian_sum += get_Radian_of_Three_Points(mesh.point(vh_2), mesh.point(current_vh), mesh.point(vh_3));
	}

	if (area == 0)
	{
		std::cout << "area is zero!" << std::endl;

		return 0;
	}

	return (2.0 * M_PI - radian_sum) / area;
}

std::vector<double> Discrete_Curvature::get_RGB_of_Color_Bar_Jet(double value, double value_min, double value_max)
{
	std::vector<double> rgb;
	rgb.resize(3);

	if (value_min == value_max)
	{
		std::cout << "Color Bar 区间长度为0!" << std::endl;
		return rgb;
	}

	if (!(value_min <= value) && !(value <= value_max))
	{
		std::cout << "Color Bar 输入数值超出总区间范围!" << std::endl;
		return rgb;
	}

	double colorBarLength = value_max - value_min;

	double value_position = value - value_min;

	double tempLength = colorBarLength / 4.0;

	if (value_position < 0.5 * tempLength)
	{
		rgb[0] = 0;
		rgb[1] = 0;
		rgb[2] = (0.5 * tempLength + value_position) / tempLength;
	}
	else if (value_position < 1.5 * tempLength)
	{
		rgb[0] = 0;
		rgb[1] = (value_position - 0.5 * tempLength) / tempLength;
		rgb[2] = 1;
	}
	else if (value_position < 2.5 * tempLength)
	{
		rgb[0] = (value_position - 1.5 * tempLength) / tempLength;
		rgb[1] = 1;
		rgb[2] = (2.5 * tempLength - value_position) / tempLength;
	}
	else if (value_position < 3.5 * tempLength)
	{
		rgb[0] = 1;
		rgb[1] = (3.5 * tempLength - value_position) / tempLength;
		rgb[2] = 0;
	}
	else
	{
		rgb[0] = (colorBarLength - value_position + 0.5 * tempLength) / tempLength;
		rgb[1] = 0;
		rgb[2] = 0;
	}

	return rgb;
}
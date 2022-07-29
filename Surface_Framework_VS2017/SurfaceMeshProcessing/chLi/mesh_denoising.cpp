#include "mesh_denoising.h"

Mesh_Denoising::Mesh_Denoising()
{

}

Mesh_Denoising::~Mesh_Denoising()
{

}

double Mesh_Denoising::get_Dist_2_of_Points(Mesh::Point point_1, Mesh::Point point_2)
{
	double dist_2_of_points = 0;

	dist_2_of_points += (point_1.data()[0] - point_2.data()[0]) * (point_1.data()[0] - point_2.data()[0]);
	dist_2_of_points += (point_1.data()[1] - point_2.data()[1]) * (point_1.data()[1] - point_2.data()[1]);
	dist_2_of_points += (point_1.data()[2] - point_2.data()[2]) * (point_1.data()[2] - point_2.data()[2]);

	return dist_2_of_points;
}

double Mesh_Denoising::get_Dist_of_Points(Mesh::Point point_1, Mesh::Point point_2)
{
	return sqrt(get_Dist_2_of_Points(point_1, point_2));
}

double Mesh_Denoising::get_Norm_of_Point(Mesh::Point point)
{
	Mesh::Point zero(0, 0, 0);

	return sqrt(get_Dist_2_of_Points(point, zero));
}

double Mesh_Denoising::get_Norm_of_Vector(std::vector<double> x)
{
	double dist_to_zero = x[0] * x[0] + x[1] * x[1] + x[2] * x[2];

	return sqrt(dist_to_zero);
}

bool Mesh_Denoising::is_In_Set(std::vector<int> set, int data)
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

double Mesh_Denoising::get_Radian_of_Three_Points(Mesh::Point point_1, Mesh::Point point_mid, Mesh::Point point_2)
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

double Mesh_Denoising::get_Cot_of_Three_Points(Mesh::Point point_1, Mesh::Point point_mid, Mesh::Point point_2)
{
	double tan_of_three_points = tan(get_Radian_of_Three_Points(point_1, point_mid, point_2));

	if (tan_of_three_points != 0)
	{
		return 1.0 / tan_of_three_points;
	}

	std::cout << "tan is zero!" << std::endl;

	return 0;
}

double Mesh_Denoising::get_Area_of_Three_Points(Mesh::Point point_1, Mesh::Point point_2, Mesh::Point point_3)
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

double Mesh_Denoising::get_Area_of_Four_Points(Mesh::Point point_1, Mesh::Point point_2, Mesh::Point point_3, Mesh::Point point_4)
{
	return get_Area_of_Three_Points(point_1, point_2, point_3) + get_Area_of_Three_Points(point_1, point_3, point_4);
}

double Mesh_Denoising::get_Area_of_Face_by_Face_Idx(Mesh& mesh, int fh_idx)
{
	std::vector<int> vh_idx_set;

	Mesh::FaceHandle fh = mesh.face_handle(fh_idx);

	Mesh::FaceVertexIter fv_it;

	bool finished = false;

	for (fv_it = mesh.fv_iter(fh); fv_it->is_valid(); ++fv_it)
	{
		if (fv_it->idx() == mesh.fv_iter(fh)->idx())
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

		vh_idx_set.emplace_back(fv_it->idx());
	}

	if (vh_idx_set.size() == 3)
	{
		Mesh::Point point_1 = mesh.point(mesh.vertex_handle(vh_idx_set[0]));
		Mesh::Point point_2 = mesh.point(mesh.vertex_handle(vh_idx_set[1]));
		Mesh::Point point_3 = mesh.point(mesh.vertex_handle(vh_idx_set[2]));

		return get_Area_of_Three_Points(point_1, point_2, point_3);
	}
	else
	{
		std::cout << "get_Norm_Vector_of_Triangle_by_Face_Idx -> 面上多于三个点!" << std::endl;
	}

	return 0;
}

std::vector<double> Mesh_Denoising::get_Norm_Vector_of_Triangle_by_Three_Points(Mesh::Point point_1, Mesh::Point point_2, Mesh::Point point_3)
{
	std::vector<double> face_norm_vector;
	face_norm_vector.resize(3);

	std::vector<double> v_1, v_2;
	v_1.resize(3);
	v_2.resize(3);

	for (int i = 0; i < 3; ++i)
	{
		v_1[i] = (point_1 - point_3).data()[i];
		v_2[i] = (point_2 - point_1).data()[i];
	}

	for (int i = 0; i < 3; ++i)
	{
		face_norm_vector[i] = v_1[(i + 1) % 3] * v_2[(i + 2) % 3] - v_1[(i + 2) % 3] * v_2[(i + 1) % 3];
	}

	double face_norm_vector_norm = face_norm_vector[0] * face_norm_vector[0] + face_norm_vector[1] * face_norm_vector[1] + face_norm_vector[2] * face_norm_vector[2];

	face_norm_vector_norm = sqrt(face_norm_vector_norm);

	for (int i = 0; i < 3; ++i)
	{
		face_norm_vector[i] /= face_norm_vector_norm;
	}

	return face_norm_vector;
}

std::vector<double> Mesh_Denoising::get_Norm_Vector_of_Triangle_by_Face_Idx(Mesh& mesh, int fh_idx)
{
	std::vector<int> vh_idx_set;

	Mesh::FaceHandle fh = mesh.face_handle(fh_idx);

	Mesh::FaceVertexIter fv_it;

	bool finished = false;

	for (fv_it = mesh.fv_iter(fh); fv_it->is_valid(); ++fv_it)
	{
		if (fv_it->idx() == mesh.fv_iter(fh)->idx())
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

		vh_idx_set.emplace_back(fv_it->idx());
	}

	/*std::vector<int> test_set;

	Mesh::HalfedgeHandle heh = mesh.halfedge_handle(fh);

	int test_vh = mesh.from_vertex_handle(heh).idx();

	test_set.emplace_back(test_vh);

	while (mesh.to_vertex_handle(heh).idx() != test_vh)
	{
		test_set.emplace_back(mesh.to_vertex_handle(heh).idx());

		heh = mesh.next_halfedge_handle(heh);
	}

	for (int i = 0; i < vh_idx_set.size(); ++i)
	{
		std::cout << vh_idx_set[i] << ",";
	}
	std::cout << std::endl;
	for (int i = 0; i < test_set.size(); ++i)
	{
		std::cout << test_set[i] << ",";
	}
	std::cout << std::endl << "----------------------------------" << std::endl;*/

	if (vh_idx_set.size() == 3)
	{
		Mesh::Point point_1 = mesh.point(mesh.vertex_handle(vh_idx_set[0]));
		Mesh::Point point_2 = mesh.point(mesh.vertex_handle(vh_idx_set[1]));
		Mesh::Point point_3 = mesh.point(mesh.vertex_handle(vh_idx_set[2]));

		return get_Norm_Vector_of_Triangle_by_Three_Points(point_1, point_2, point_3);
	}
	else
	{
		std::cout << "get_Norm_Vector_of_Triangle_by_Face_Idx -> 面上多于三个点!" << std::endl;
	}

	std::vector<double> protect_data;

	return protect_data;
}

std::vector<double> Mesh_Denoising::get_Center_of_Triangle_by_Face_Idx(Mesh& mesh, int fh_idx)
{
	std::vector<int> vh_idx_set;

	Mesh::FaceHandle fh = mesh.face_handle(fh_idx);

	Mesh::FaceVertexIter fv_it;

	bool finished = false;

	for (fv_it = mesh.fv_iter(fh); fv_it->is_valid(); ++fv_it)
	{
		if (fv_it->idx() == mesh.fv_iter(fh)->idx())
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

		vh_idx_set.emplace_back(fv_it->idx());
	}

	if (vh_idx_set.size() == 3)
	{
		Mesh::Point point_1 = mesh.point(mesh.vertex_handle(vh_idx_set[0]));
		Mesh::Point point_2 = mesh.point(mesh.vertex_handle(vh_idx_set[1]));
		Mesh::Point point_3 = mesh.point(mesh.vertex_handle(vh_idx_set[2]));

		Mesh::Point average_point = (point_1 + point_2 + point_3) / 3.0;

		std::vector<double> face_center;

		for (int i = 0; i < 3; ++i)
		{
			face_center.emplace_back(average_point.data()[i]);
		}

		return face_center;
	}
	else
	{
		std::cout << "get_Norm_Vector_of_Triangle_by_Face_Idx -> 面上多于三个点!" << std::endl;
	}

	std::vector<double> protect_data;

	return protect_data;
}

std::vector<int> Mesh_Denoising::get_Neighboor_Face_Idx_Set(Mesh& mesh, int fh_idx)
{
	std::vector<int> fh_idx_set;

	Mesh::FaceHandle fh = mesh.face_handle(fh_idx);

	Mesh::FaceFaceIter ff_it;

	bool finished = false;

	for (ff_it = mesh.ff_iter(fh); ff_it->is_valid(); ++ff_it)
	{
		if (ff_it->idx() == mesh.ff_iter(fh)->idx())
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

		fh_idx_set.emplace_back(ff_it->idx());
	}

	return fh_idx_set;
}

std::vector<double> Mesh_Denoising::update_Face_Normal_by_Face_Idx(Mesh& mesh, int fh_idx, double sigma_space, double sigma_normal)
{
	Mesh::FaceHandle fh = mesh.face_handle(fh_idx);

	std::vector<int> fh_neighboor_idx_set = get_Neighboor_Face_Idx_Set(mesh, fh_idx);

	std::vector<double> fh_neighboor_area_set;

	std::vector<double> current_fh_center = get_Center_of_Triangle_by_Face_Idx(mesh, fh_idx);
	std::vector<std::vector<double>> fh_neighboor_center_set;

	std::vector<double> current_fh_normal = get_Norm_Vector_of_Triangle_by_Face_Idx(mesh, fh_idx);
	std::vector<std::vector<double>> fh_neighboor_normal_set;

	std::vector<double> space_distance_set;

	std::vector<double> normal_difference_set;

	double total_weights = 0;

	std::vector<double> new_fh_normal;

	new_fh_normal.resize(3);

	for (int i = 0; i < fh_neighboor_idx_set.size(); ++i)
	{
		fh_neighboor_area_set.emplace_back(get_Area_of_Face_by_Face_Idx(mesh, fh_neighboor_idx_set[i]));

		fh_neighboor_center_set.emplace_back(get_Center_of_Triangle_by_Face_Idx(mesh, fh_neighboor_idx_set[i]));

		fh_neighboor_normal_set.emplace_back(get_Norm_Vector_of_Triangle_by_Face_Idx(mesh, fh_neighboor_idx_set[i]));

		space_distance_set.emplace_back(sqrt((current_fh_center[0] - fh_neighboor_center_set[i][0]) * (current_fh_center[0] - fh_neighboor_center_set[i][0]) + (current_fh_center[1] - fh_neighboor_center_set[i][1]) * (current_fh_center[1] - fh_neighboor_center_set[i][1]) + (current_fh_center[2] - fh_neighboor_center_set[i][2]) * (current_fh_center[2] - fh_neighboor_center_set[i][2])));

		normal_difference_set.emplace_back(sqrt((current_fh_normal[0] - fh_neighboor_normal_set[i][0]) * (current_fh_normal[0] - fh_neighboor_normal_set[i][0]) + (current_fh_normal[1] - fh_neighboor_normal_set[i][1]) * (current_fh_normal[1] - fh_neighboor_normal_set[i][1]) + (current_fh_normal[2] - fh_neighboor_normal_set[i][2]) * (current_fh_normal[2] - fh_neighboor_normal_set[i][2])));

		double current_weight = fh_neighboor_area_set[i] * exp(-(space_distance_set[i] * space_distance_set[i]) / 2.0 / sigma_space) * exp(-(normal_difference_set[i] * normal_difference_set[i]) / 2.0 / sigma_normal);

		total_weights += current_weight;

		for (int j = 0; j < 3; ++j)
		{
			new_fh_normal[j] += current_weight * fh_neighboor_normal_set[i][j];
		}
	}

	for (int i = 0; i < 3; ++i)
	{
		new_fh_normal[i] /= total_weights;
	}

	return new_fh_normal;
}

std::vector<std::vector<double>> Mesh_Denoising::update_Face_Normal(Mesh& mesh, int fh_normal_iteration_num, double sigma_space, double sigma_normal)
{
	std::vector<std::vector<double>> current_face_normal;

	std::vector<std::vector<double>> new_face_normal;

	std::vector<double> fh_area_set;

	std::vector<std::vector<double>> fh_center_set;

	std::vector<std::vector<int>> fh_neighboor_idx_set;

	std::vector<std::vector<double>> space_distance_set;

	current_face_normal.resize(mesh.n_faces());
	new_face_normal.resize(mesh.n_faces());
	fh_area_set.resize(mesh.n_faces());
	fh_center_set.resize(mesh.n_faces());
	fh_neighboor_idx_set.resize(mesh.n_faces());
	space_distance_set.resize(mesh.n_faces());

	for (int i = 0; i < mesh.n_faces(); ++i)
	{
		current_face_normal[i] = get_Norm_Vector_of_Triangle_by_Face_Idx(mesh, i);
		new_face_normal[i].resize(3);
		for (int j = 0; j < 3; ++j)
		{
			new_face_normal[i][j] = 0;
		}

		fh_area_set[i] = get_Area_of_Face_by_Face_Idx(mesh, i);

		fh_center_set[i] = get_Center_of_Triangle_by_Face_Idx(mesh, i);

		fh_neighboor_idx_set[i] = get_Neighboor_Face_Idx_Set(mesh, i);
	}

	for (int i = 0; i < mesh.n_faces(); ++i)
	{
		for (int j = 0; j < fh_neighboor_idx_set[i].size(); ++j)
		{
			space_distance_set[i].emplace_back(sqrt((fh_center_set[i][0] - fh_center_set[fh_neighboor_idx_set[i][j]][0]) * (fh_center_set[i][0] - fh_center_set[fh_neighboor_idx_set[i][j]][0]) + (fh_center_set[i][1] - fh_center_set[fh_neighboor_idx_set[i][j]][1]) * (fh_center_set[i][1] - fh_center_set[fh_neighboor_idx_set[i][j]][1]) + (fh_center_set[i][2] - fh_center_set[fh_neighboor_idx_set[i][j]][2]) * (fh_center_set[i][2] - fh_center_set[fh_neighboor_idx_set[i][j]][2])));
		}
	}

	for (int l = 0; l < fh_normal_iteration_num; ++l)
	{
		for (int i = 0; i < mesh.n_faces(); ++i)
		{
			std::vector<double> normal_difference_set;

			double total_weights = 0;

			for (int j = 0; j < fh_neighboor_idx_set[i].size(); ++j)
			{
				normal_difference_set.emplace_back(sqrt((current_face_normal[i][0] - current_face_normal[fh_neighboor_idx_set[i][j]][0]) * (current_face_normal[i][0] - current_face_normal[fh_neighboor_idx_set[i][j]][0]) + (current_face_normal[i][1] - current_face_normal[fh_neighboor_idx_set[i][j]][1]) * (current_face_normal[i][1] - current_face_normal[fh_neighboor_idx_set[i][j]][1]) + (current_face_normal[i][2] - current_face_normal[fh_neighboor_idx_set[i][j]][2]) * (current_face_normal[i][2] - current_face_normal[fh_neighboor_idx_set[i][j]][2])));

				double current_weight = fh_area_set[fh_neighboor_idx_set[i][j]] * exp(-(space_distance_set[i][j] * space_distance_set[i][j]) / 2.0 / sigma_space) * exp(-(normal_difference_set[j] * normal_difference_set[j]) / 2.0 / sigma_normal);

				total_weights += current_weight;

				for (int k = 0; k < 3; ++k)
				{
					new_face_normal[i][k] += current_weight * current_face_normal[fh_neighboor_idx_set[i][j]][k];
				}
			}

			for (int j = 0; j < 3; ++j)
			{
				new_face_normal[i][j] /= total_weights;
			}
		}

		for (int i = 0; i < mesh.n_faces(); ++i)
		{
			for (int j = 0; j < 3; ++j)
			{
				current_face_normal[i][j] = new_face_normal[i][j];

				new_face_normal[i][j] = 0;
			}
		}
	}

	return current_face_normal;
}

bool Mesh_Denoising::update_Vertex_Position(Mesh& mesh, int fh_normal_iteration_num, int vh_position_iteration_num, double sigma_space, double sigma_normal)
{
	if (fh_normal_iteration_num < 0 || vh_position_iteration_num < 0)
	{
		std::cout << "求解次数为负数!" << std::endl;

		return true;
	}

	std::vector<std::vector<double>> new_face_normal = update_Face_Normal(mesh, fh_normal_iteration_num, sigma_space, sigma_normal);

	std::vector<std::vector<double>> fh_center_set;

	std::vector<std::vector<int>> vh_neighboor_fh_idx_set;

	std::vector<std::vector<int>> fh_neighboor_vh_idx_set;

	std::vector<std::vector<double>> current_vertex_position;

	std::vector<std::vector<double>> new_vertex_position;

	fh_center_set.resize(mesh.n_faces());
	vh_neighboor_fh_idx_set.resize(mesh.n_vertices());
	fh_neighboor_vh_idx_set.resize(mesh.n_faces());
	current_vertex_position.resize(mesh.n_vertices());
	new_vertex_position.resize(mesh.n_vertices());

	for (int i = 0; i < mesh.n_faces(); ++i)
	{
		fh_center_set[i] = get_Center_of_Triangle_by_Face_Idx(mesh, i);

		Mesh::FaceHandle fh = mesh.face_handle(i);

		Mesh::FaceVertexIter fv_it;

		bool finished = false;

		for (fv_it = mesh.fv_iter(fh); fv_it->is_valid(); ++fv_it)
		{
			if (fv_it->idx() == mesh.fv_iter(fh)->idx())
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

			fh_neighboor_vh_idx_set[i].emplace_back(fv_it->idx());
		}
	}

	for (int i = 0; i < mesh.n_vertices(); ++i)
	{
		current_vertex_position[i].resize(3);
		new_vertex_position[i].resize(3);

		Mesh::VertexHandle vh = mesh.vertex_handle(i);

		for (int j = 0; j < 3; ++j)
		{
			current_vertex_position[i][j] = mesh.point(vh).data()[j];
			new_vertex_position[i][j] = 0;
		}

		Mesh::VertexFaceIter vf_it;

		bool finished = false;

		for (vf_it = mesh.vf_iter(vh); vf_it->is_valid(); ++vf_it)
		{
			if (vf_it->idx() == mesh.vf_iter(vh)->idx())
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

			vh_neighboor_fh_idx_set[i].emplace_back(vf_it->idx());
		}
	}

	for (int l = 0; l < vh_position_iteration_num; ++l)
	{
		for (int i = 0; i < mesh.n_vertices(); ++i)
		{
			for (int j = 0; j < vh_neighboor_fh_idx_set[i].size(); ++j)
			{
				std::vector<double> vh_to_fh_center;

				vh_to_fh_center.resize(3);

				for (int k = 0; k < 3; ++k)
				{
					vh_to_fh_center[k] = fh_center_set[vh_neighboor_fh_idx_set[i][j]][k] - current_vertex_position[i][k];
				}

				double current_weight = new_face_normal[vh_neighboor_fh_idx_set[i][j]][0] * vh_to_fh_center[0] + new_face_normal[vh_neighboor_fh_idx_set[i][j]][1] * vh_to_fh_center[1] + new_face_normal[vh_neighboor_fh_idx_set[i][j]][2] * vh_to_fh_center[2];

				for (int k = 0; k < 3; ++k)
				{
					new_vertex_position[i][k] += current_weight * new_face_normal[vh_neighboor_fh_idx_set[i][j]][k];
				}
			}

			for (int j = 0; j < 3; ++j)
			{
				new_vertex_position[i][j] /= vh_neighboor_fh_idx_set[i].size();

				new_vertex_position[i][j] += current_vertex_position[i][j];
			}
		}

		for (int i = 0; i < mesh.n_faces(); ++i)
		{
			for (int j = 0; j < 3; ++j)
			{
				fh_center_set[i][j] = (current_vertex_position[fh_neighboor_vh_idx_set[i][0]][j] + current_vertex_position[fh_neighboor_vh_idx_set[i][1]][j] + current_vertex_position[fh_neighboor_vh_idx_set[i][2]][j]) / 3.0;
			}
		}

		for (int i = 0; i < mesh.n_vertices(); ++i)
		{
			for (int j = 0; j < 3; ++j)
			{
				current_vertex_position[i][j] = new_vertex_position[i][j];

				new_vertex_position[i][j] = 0;
			}
		}
	}

	for (int i = 0; i < mesh.n_vertices(); ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			mesh.point(mesh.vertex_handle(i))[j] = current_vertex_position[i][j];
		}
	}

	return true;
}
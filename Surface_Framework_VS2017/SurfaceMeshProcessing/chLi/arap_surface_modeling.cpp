#include "arap_surface_modeling.h"

ARAP_Surface_Modeling::ARAP_Surface_Modeling()
{

}

ARAP_Surface_Modeling::~ARAP_Surface_Modeling()
{

}

double ARAP_Surface_Modeling::get_Dist_2_of_Points(Mesh::Point point_1, Mesh::Point point_2)
{
	double dist_2_of_points = 0;

	dist_2_of_points += (point_1.data()[0] - point_2.data()[0]) * (point_1.data()[0] - point_2.data()[0]);
	dist_2_of_points += (point_1.data()[1] - point_2.data()[1]) * (point_1.data()[1] - point_2.data()[1]);
	dist_2_of_points += (point_1.data()[2] - point_2.data()[2]) * (point_1.data()[2] - point_2.data()[2]);

	return dist_2_of_points;
}

double ARAP_Surface_Modeling::get_Dist_of_Points(Mesh::Point point_1, Mesh::Point point_2)
{
	return sqrt(get_Dist_2_of_Points(point_1, point_2));
}

double ARAP_Surface_Modeling::get_Norm_of_Point(Mesh::Point point)
{
	Mesh::Point zero(0, 0, 0);

	return sqrt(get_Dist_2_of_Points(point, zero));
}

double ARAP_Surface_Modeling::get_Norm_of_Vector(std::vector<double> x)
{
	double dist_to_zero = x[0] * x[0] + x[1] * x[1] + x[2] * x[2];

	return sqrt(dist_to_zero);
}

bool ARAP_Surface_Modeling::is_In_Set(std::vector<int> set, int data)
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

double ARAP_Surface_Modeling::get_Radian_of_Three_Points(Mesh::Point point_1, Mesh::Point point_mid, Mesh::Point point_2)
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

double ARAP_Surface_Modeling::get_Cot_of_Three_Points(Mesh::Point point_1, Mesh::Point point_mid, Mesh::Point point_2)
{
	double tan_of_three_points = tan(get_Radian_of_Three_Points(point_1, point_mid, point_2));

	if (tan_of_three_points != 0)
	{
		return 1.0 / tan_of_three_points;
	}

	std::cout << "tan is zero!" << std::endl;

	return 0;
}

double ARAP_Surface_Modeling::get_Area_of_Three_Points(Mesh::Point point_1, Mesh::Point point_2, Mesh::Point point_3)
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

double ARAP_Surface_Modeling::get_Area_of_Four_Points(Mesh::Point point_1, Mesh::Point point_2, Mesh::Point point_3, Mesh::Point point_4)
{
	return get_Area_of_Three_Points(point_1, point_2, point_3) + get_Area_of_Three_Points(point_1, point_3, point_4);
}

double ARAP_Surface_Modeling::get_Area_of_Face_by_Face_Idx(Mesh& mesh, int fh_idx)
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

std::vector<double> ARAP_Surface_Modeling::get_Norm_Vector_of_Triangle_by_Three_Points(Mesh::Point point_1, Mesh::Point point_2, Mesh::Point point_3)
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

std::vector<double> ARAP_Surface_Modeling::get_Norm_Vector_of_Triangle_by_Face_Idx(Mesh& mesh, int fh_idx)
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

std::vector<double> ARAP_Surface_Modeling::get_Center_of_Triangle_by_Face_Idx(Mesh& mesh, int fh_idx)
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

std::vector<int> ARAP_Surface_Modeling::get_Neighboor_Face_Idx_Set(Mesh& mesh, int fh_idx)
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

std::vector<double> ARAP_Surface_Modeling::get_Local_Average_Area_Set_of_VH(Mesh& mesh)
{
	std::vector<double> local_average_area_set;

	local_average_area_set.resize(mesh.n_vertices());

	for (int i = 0; i < mesh.n_vertices(); ++i)
	{
		local_average_area_set[i] = 0;

		std::vector<int> vh_neighboor_vh_idx_set;

		for (auto vh : mesh.vv_range(mesh.vertex_handle(i)))
		{
			vh_neighboor_vh_idx_set.emplace_back(vh.idx());
		}

		Mesh::Point current_point = mesh.point(mesh.vertex_handle(i));

		for (int j = 0; j < vh_neighboor_vh_idx_set.size(); ++j)
		{
			bool in_same_triangle = false;

			Mesh::HalfedgeHandle heh_1 = mesh.find_halfedge(mesh.vertex_handle(vh_neighboor_vh_idx_set[j]), mesh.vertex_handle(i));
			Mesh::HalfedgeHandle heh_2 = mesh.find_halfedge(mesh.vertex_handle(i), mesh.vertex_handle(vh_neighboor_vh_idx_set[j]));

			if (mesh.next_halfedge_handle(mesh.next_halfedge_handle(mesh.next_halfedge_handle(heh_1))).idx() == heh_1.idx() && mesh.to_vertex_handle(mesh.next_halfedge_handle(heh_1)).idx() == vh_neighboor_vh_idx_set[(j + 1) % vh_neighboor_vh_idx_set.size()])
			{
				in_same_triangle = true;
			}
			else if (mesh.next_halfedge_handle(mesh.next_halfedge_handle(mesh.next_halfedge_handle(heh_2))).idx() == heh_2.idx() && mesh.from_vertex_handle(mesh.prev_halfedge_handle(heh_2)).idx() == vh_neighboor_vh_idx_set[(j + 1) % vh_neighboor_vh_idx_set.size()])
			{
				in_same_triangle = true;
			}

			if (in_same_triangle)
			{
				Mesh::Point point1 = mesh.point(mesh.vertex_handle(vh_neighboor_vh_idx_set[j]));
				Mesh::Point point2 = mesh.point(mesh.vertex_handle(vh_neighboor_vh_idx_set[(j + 1) % vh_neighboor_vh_idx_set.size()]));
				Mesh::Point avg_point = (point1 + point2 + current_point) / 3.0;

				if (vh_neighboor_vh_idx_set.size() == 2)
				{
					local_average_area_set[i] += get_Area_of_Four_Points(current_point, (current_point + point1) / 2.0, avg_point, (current_point + point2) / 2.0) / 2.0;
				}
				else
				{
					local_average_area_set[i] += get_Area_of_Four_Points(current_point, (current_point + point1) / 2.0, avg_point, (current_point + point2) / 2.0);
				}
			}
		}
	}

	return local_average_area_set;
}

std::vector<std::vector<double>> ARAP_Surface_Modeling::get_Cot_Weight_Set(Mesh& mesh)
{
	std::vector<std::vector<double>> cot_weight_set;

	cot_weight_set.resize(mesh.n_vertices());

	for (int i = 0; i < mesh.n_vertices(); ++i)
	{
		std::vector<int> vh_neighboor_vh_idx_set;

		for (auto vh : mesh.vv_range(mesh.vertex_handle(i)))
		{
			vh_neighboor_vh_idx_set.emplace_back(vh.idx());
		}

		cot_weight_set[i].resize(vh_neighboor_vh_idx_set.size());

		Mesh::Point current_point = mesh.point(mesh.vertex_handle(i));

		for (int j = 0; j < vh_neighboor_vh_idx_set.size(); ++j)
		{
			if (!mesh.is_boundary(mesh.edge_handle(mesh.find_halfedge(mesh.vertex_handle(i), mesh.vertex_handle(vh_neighboor_vh_idx_set[j])))))
			{
				Mesh::Point point1 = mesh.point(mesh.vertex_handle(vh_neighboor_vh_idx_set[(j - 1 + vh_neighboor_vh_idx_set.size()) % vh_neighboor_vh_idx_set.size()]));
				Mesh::Point point2 = mesh.point(mesh.vertex_handle(vh_neighboor_vh_idx_set[j]));
				Mesh::Point point3 = mesh.point(mesh.vertex_handle(vh_neighboor_vh_idx_set[(j + 1) % vh_neighboor_vh_idx_set.size()]));

				double cot_sum = (get_Cot_of_Three_Points(current_point, point1, point2) + get_Cot_of_Three_Points(current_point, point3, point2)) / 2.0;

				double radian_1 = get_Radian_of_Three_Points(current_point, point1, point2) / 2.0;
				double radian_2 = get_Radian_of_Three_Points(current_point, point3, point2) / 2.0;

				double dist = get_Norm_of_Point(mesh.point(mesh.vertex_handle(i)) - mesh.point(mesh.vertex_handle(vh_neighboor_vh_idx_set[j])));

				double mvc = (tan(radian_1) + tan(radian_2)) / dist;

				cot_weight_set[i][j] = mvc;
			}
			else if (!mesh.is_boundary(mesh.edge_handle(mesh.find_halfedge(mesh.vertex_handle(i), mesh.vertex_handle(vh_neighboor_vh_idx_set[(j + 1) % vh_neighboor_vh_idx_set.size()])))))
			{
				Mesh::Point point2 = mesh.point(mesh.vertex_handle(vh_neighboor_vh_idx_set[j]));
				Mesh::Point point3 = mesh.point(mesh.vertex_handle(vh_neighboor_vh_idx_set[(j + 1) % vh_neighboor_vh_idx_set.size()]));

				double cot_sum = get_Cot_of_Three_Points(current_point, point3, point2);

				double radian_2 = get_Radian_of_Three_Points(current_point, point3, point2) / 2.0;

				double dist = get_Norm_of_Point(mesh.point(mesh.vertex_handle(i)) - mesh.point(mesh.vertex_handle(vh_neighboor_vh_idx_set[j])));

				double mvc = (2 * tan(radian_2)) / dist;

				cot_weight_set[i][j] = mvc;
			}
			else
			{
				Mesh::Point point1 = mesh.point(mesh.vertex_handle(vh_neighboor_vh_idx_set[(j - 1 + vh_neighboor_vh_idx_set.size()) % vh_neighboor_vh_idx_set.size()]));
				Mesh::Point point2 = mesh.point(mesh.vertex_handle(vh_neighboor_vh_idx_set[j]));

				double cot_sum = get_Cot_of_Three_Points(current_point, point1, point2);

				double radian_1 = get_Radian_of_Three_Points(current_point, point1, point2) / 2.0;

				double dist = get_Norm_of_Point(mesh.point(mesh.vertex_handle(i)) - mesh.point(mesh.vertex_handle(vh_neighboor_vh_idx_set[j])));

				double mvc = (2 * tan(radian_1)) / dist;

				cot_weight_set[i][j] = mvc;
			}
		}

		double current_sum = 0;

		for (int j = 0; j < vh_neighboor_vh_idx_set.size(); ++j)
		{
			current_sum += cot_weight_set[i][j];
		}

		for (int j = 0; j < vh_neighboor_vh_idx_set.size(); ++j)
		{
			cot_weight_set[i][j] /= current_sum;
		}
	}

	return cot_weight_set;
}

std::vector<std::vector<double>> ARAP_Surface_Modeling::Update_VH_Position(Mesh& mesh, std::vector<int> target_vh_idx, std::vector<std::vector<double>> target_pose, std::vector<int> fixed_vh_idx)
{
	std::vector<double> local_average_area_set = get_Local_Average_Area_Set_of_VH(mesh);

	std::vector<std::vector<double>> cot_weight_set = get_Cot_Weight_Set(mesh);

	std::vector<std::vector<double>> source_vh_position_set;

	std::vector<std::vector<double>> target_vh_position_set;

	source_vh_position_set.resize(mesh.n_vertices());

	for (int i = 0; i < mesh.n_vertices(); ++i)
	{
		Mesh::Point current_point = mesh.point(mesh.vertex_handle(i));

		for (int j = 0; j < 3; ++j)
		{
			source_vh_position_set[i].emplace_back(current_point.data()[j]);
		}
	}

	if (target_vh_idx.size() == 0 || fixed_vh_idx.size() == 0)
	{
		return source_vh_position_set;
	}

	target_vh_position_set.resize(mesh.n_vertices());

	Mesh::Point avg_new_target_pose;

	for (int i = 0; i < target_vh_idx.size(); ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			avg_new_target_pose[j] += target_pose[i][j];
		}
	}

	avg_new_target_pose /= target_vh_idx.size();

	Mesh::Point avg_target_pose;

	for (int i = 0; i < target_vh_idx.size(); ++i)
	{
		avg_target_pose += mesh.point(mesh.vertex_handle(target_vh_idx[i]));
	}

	avg_target_pose /= target_vh_idx.size();

	Mesh::Point avg_fixed_pose;

	for (int i = 0; i < fixed_vh_idx.size(); ++i)
	{
		avg_fixed_pose += mesh.point(mesh.vertex_handle(fixed_vh_idx[i]));
	}

	avg_fixed_pose /= fixed_vh_idx.size();

	for (int i = 0; i < mesh.n_vertices(); ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			target_vh_position_set[i].emplace_back(source_vh_position_set[i][j]);
		}

		bool need_to_update = true;

		for (int j = 0; j < target_vh_idx.size(); ++j)
		{
			if (i == target_vh_idx[j])
			{
				for (int k = 0; k < 3; ++k)
				{
					target_vh_position_set[i][k] = target_pose[j][k];

					//mesh.point(mesh.vertex_handle(i))[k] = target_pose[j][k];
				}

				need_to_update = false;

				break;
			}
		}

		if (need_to_update)
		{
			for (int j = 0; j < fixed_vh_idx.size(); ++j)
			{
				if (i == fixed_vh_idx[j])
				{
					need_to_update = false;

					break;
				}
			}
		}

		if (!need_to_update)
		{
			continue;
		}

		/*double dist_to_target_vh = get_Norm_of_Point(avg_target_pose - mesh.point(mesh.vertex_handle(i)));
		double dist_to_fixed_vh = get_Norm_of_Point(avg_fixed_pose - mesh.point(mesh.vertex_handle(i)));

		Mesh::Point move_vector = avg_new_target_pose - avg_target_pose;

		for (int k = 0; k < 3; ++k)
		{
			target_vh_position_set[i][k] += dist_to_fixed_vh / (dist_to_fixed_vh + dist_to_target_vh) * move_vector.data()[k];
		}*/
	}

	MatrixXd A(mesh.n_vertices(), mesh.n_vertices());

	MatrixXd x(mesh.n_vertices(), 3);

	MatrixXd b(mesh.n_vertices(), 3);

	std::vector<MatrixXd> R_Set;

	R_Set.resize(mesh.n_vertices());

	for (int i = 0; i < mesh.n_vertices(); ++i)
	{
		R_Set[i].resize(3, 3);
	}

	for (int i = 0; i < mesh.n_vertices(); ++i)
	{
		MatrixXd S(3, 3);

		S.setZero();

		std::vector<int> vh_neighboor_vh_idx_set;

		for (auto vh : mesh.vv_range(mesh.vertex_handle(i)))
		{
			vh_neighboor_vh_idx_set.emplace_back(vh.idx());
		}

		for (int j = 0; j < vh_neighboor_vh_idx_set.size(); ++j)
		{
			Mesh::Point source_vector = mesh.point(mesh.vertex_handle(i)) - mesh.point(mesh.vertex_handle(vh_neighboor_vh_idx_set[j]));

			MatrixXd source_matrix(3, 1);

			for (int k = 0; k < 3; ++k)
			{
				source_matrix(k, 0) = source_vector.data()[k];
			}

			MatrixXd target_matrix(1, 3);

			for (int k = 0; k < 3; ++k)
			{
				target_matrix(0, k) = target_vh_position_set[i][k] - target_vh_position_set[vh_neighboor_vh_idx_set[j]][k];
			}

			S += cot_weight_set[i][j] * source_matrix * target_matrix;
		}

		JacobiSVD<MatrixXd> svd(S, ComputeFullU | ComputeFullV);

		MatrixXd U = svd.matrixU();
		MatrixXd V = svd.matrixV();

		R_Set[i] = V * U.transpose();
	}

	A.setZero();

	b.setZero();

	for (int i = 0; i < mesh.n_vertices(); ++i)
	{
		bool fix_this_vh = false;

		for (int j = 0; j < target_vh_idx.size(); ++j)
		{
			if (i == target_vh_idx[j])
			{
				A(i, i) = 1.0;

				for (int k = 0; k < 3; ++k)
				{
					b(i, k) = target_pose[j][k];
				}

				fix_this_vh = true;

				break;
			}
		}

		if (!fix_this_vh)
		{
			for (int j = 0; j < fixed_vh_idx.size(); ++j)
			{
				if (i == fixed_vh_idx[j])
				{
					A(i, i) = 1.0;

					for (int k = 0; k < 3; ++k)
					{
						b(i, k) = mesh.point(mesh.vertex_handle(i)).data()[k];
					}

					fix_this_vh = true;

					break;
				}
			}
		}

		if (fix_this_vh)
		{
			continue;
		}

		std::vector<int> vh_neighboor_vh_idx_set;

		for (auto vh : mesh.vv_range(mesh.vertex_handle(i)))
		{
			vh_neighboor_vh_idx_set.emplace_back(vh.idx());
		}

		for (int j = 0; j < vh_neighboor_vh_idx_set.size(); ++j)
		{
			A(i, i) += local_average_area_set[i] * cot_weight_set[i][j];

			int current_vh_neighboor_idx = 0;

			for (auto vh : mesh.vv_range(mesh.vertex_handle(vh_neighboor_vh_idx_set[j])))
			{
				if (vh.idx() != i)
				{
					++current_vh_neighboor_idx;
				}
				else
				{
					break;
				}
			}

			A(i, i) += local_average_area_set[vh_neighboor_vh_idx_set[j]] * cot_weight_set[vh_neighboor_vh_idx_set[j]][current_vh_neighboor_idx];

			A(i, vh_neighboor_vh_idx_set[j]) -= local_average_area_set[i] * cot_weight_set[i][j];

			A(i, vh_neighboor_vh_idx_set[j]) -= local_average_area_set[vh_neighboor_vh_idx_set[j]] * cot_weight_set[vh_neighboor_vh_idx_set[j]][current_vh_neighboor_idx];

			Mesh::Point source_vector = mesh.point(mesh.vertex_handle(i)) - mesh.point(mesh.vertex_handle(vh_neighboor_vh_idx_set[j]));

			MatrixXd source_matrix(3, 1);

			for (int k = 0; k < 3; ++k)
			{
				source_matrix(k, 0) = source_vector.data()[k];
			}

			source_matrix = local_average_area_set[i] * cot_weight_set[i][j] * R_Set[i] * source_matrix;

			for (int k = 0; k < 3; ++k)
			{
				b(i, k) += source_matrix(k, 0);
			}

			for (int k = 0; k < 3; ++k)
			{
				source_matrix(k, 0) = source_vector.data()[k];
			}

			source_matrix = local_average_area_set[vh_neighboor_vh_idx_set[j]] * cot_weight_set[vh_neighboor_vh_idx_set[j]][current_vh_neighboor_idx] * R_Set[vh_neighboor_vh_idx_set[j]] * source_matrix;

			for (int k = 0; k < 3; ++k)
			{
				b(i, k) += source_matrix(k, 0);
			}
		}
	}

	FullPivHouseholderQR<MatrixXd>* Solver = new FullPivHouseholderQR<MatrixXd>(A);

	x = Solver->solve(b);

	for (int i = 0; i < mesh.n_vertices(); ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			target_vh_position_set[i][j] = x(i, j);
		}
	}
	
	return target_vh_position_set;
}
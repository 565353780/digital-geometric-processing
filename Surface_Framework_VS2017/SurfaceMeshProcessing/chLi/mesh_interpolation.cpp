#include "mesh_interpolation.h"

Mesh_Interpolation::Mesh_Interpolation()
{
	need_to_update_matrix = false;
	need_to_update_matrix_set = false;
}

Mesh_Interpolation::~Mesh_Interpolation()
{

}

double Mesh_Interpolation::get_Dist_2_of_Points(Mesh::Point point_1, Mesh::Point point_2)
{
	double dist_2_of_points = 0;

	dist_2_of_points += (point_1.data()[0] - point_2.data()[0]) * (point_1.data()[0] - point_2.data()[0]);
	dist_2_of_points += (point_1.data()[1] - point_2.data()[1]) * (point_1.data()[1] - point_2.data()[1]);
	dist_2_of_points += (point_1.data()[2] - point_2.data()[2]) * (point_1.data()[2] - point_2.data()[2]);

	return dist_2_of_points;
}

double Mesh_Interpolation::get_Dist_of_Points(Mesh::Point point_1, Mesh::Point point_2)
{
	return sqrt(get_Dist_2_of_Points(point_1, point_2));
}

double Mesh_Interpolation::get_Norm_of_Point(Mesh::Point point)
{
	Mesh::Point zero(0, 0, 0);

	return sqrt(get_Dist_2_of_Points(point, zero));
}

double Mesh_Interpolation::get_Norm_of_Vector(std::vector<double> x)
{
	double dist_to_zero = x[0] * x[0] + x[1] * x[1] + x[2] * x[2];

	return sqrt(dist_to_zero);
}

bool Mesh_Interpolation::is_In_Set(std::vector<int> set, int data)
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

double Mesh_Interpolation::get_Radian_of_Three_Points(Mesh::Point point_1, Mesh::Point point_mid, Mesh::Point point_2)
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

double Mesh_Interpolation::get_Cot_of_Three_Points(Mesh::Point point_1, Mesh::Point point_mid, Mesh::Point point_2)
{
	double tan_of_three_points = tan(get_Radian_of_Three_Points(point_1, point_mid, point_2));

	if (tan_of_three_points != 0)
	{
		return 1.0 / tan_of_three_points;
	}

	std::cout << "tan is zero!" << std::endl;

	return 0;
}

double Mesh_Interpolation::get_Area_of_Three_Points(Mesh::Point point_1, Mesh::Point point_2, Mesh::Point point_3)
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

double Mesh_Interpolation::get_Area_of_Four_Points(Mesh::Point point_1, Mesh::Point point_2, Mesh::Point point_3, Mesh::Point point_4)
{
	return get_Area_of_Three_Points(point_1, point_2, point_3) + get_Area_of_Three_Points(point_1, point_3, point_4);
}

double Mesh_Interpolation::get_Area_of_Face_by_Face_Idx(Mesh& mesh, int fh_idx)
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

std::vector<double> Mesh_Interpolation::get_Norm_Vector_of_Triangle_by_Three_Points(Mesh::Point point_1, Mesh::Point point_2, Mesh::Point point_3)
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

std::vector<double> Mesh_Interpolation::get_Norm_Vector_of_Triangle_by_Face_Idx(Mesh& mesh, int fh_idx)
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

std::vector<double> Mesh_Interpolation::get_Center_of_Triangle_by_Face_Idx(Mesh& mesh, int fh_idx)
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

std::vector<int> Mesh_Interpolation::get_Neighboor_Face_Idx_Set(Mesh& mesh, int fh_idx)
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

std::vector<std::vector<int>> Mesh_Interpolation::get_FVH_Idx_Set(Mesh& mesh)
{
	std::vector<std::vector<int>> fvh_idx_set;

	fvh_idx_set.resize(mesh.n_faces());

	for (int i = 0; i < mesh.n_faces(); ++i)
	{
		for (auto fvh : mesh.fv_range(mesh.face_handle(i)))
		{
			fvh_idx_set[i].emplace_back(fvh.idx());
		}
	}

	return fvh_idx_set;
}

std::vector<std::vector<int>> Mesh_Interpolation::get_VFH_Idx_Set(Mesh& mesh)
{
	std::vector<std::vector<int>> vfh_idx_set;

	vfh_idx_set.resize(mesh.n_vertices());

	for (int i = 0; i < mesh.n_vertices(); ++i)
	{
		for (auto vfh : mesh.vf_range(mesh.vertex_handle(i)))
		{
			vfh_idx_set[i].emplace_back(vfh.idx());
		}
	}

	return vfh_idx_set;
}

std::vector<double> Mesh_Interpolation::get_Face_Area_Set(Mesh& mesh)
{
	std::vector<double> face_area_set;

	face_area_set.resize(mesh.n_faces());

	for (int i = 0; i < mesh.n_faces(); ++i)
	{
		face_area_set[i] = get_Area_of_Face_by_Face_Idx(mesh, i);
	}

	return face_area_set;
}

std::vector<double> Mesh_Interpolation::get_Local_Average_Area_Set_of_VH(Mesh& mesh)
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
			if (!mesh.is_boundary(mesh.vertex_handle(vh_neighboor_vh_idx_set[j])) || !mesh.is_boundary(mesh.vertex_handle(vh_neighboor_vh_idx_set[(j + 1) % vh_neighboor_vh_idx_set.size()])))
			{
				Mesh::Point point1 = mesh.point(mesh.vertex_handle(vh_neighboor_vh_idx_set[j]));
				Mesh::Point point2 = mesh.point(mesh.vertex_handle(vh_neighboor_vh_idx_set[(j + 1) % vh_neighboor_vh_idx_set.size()]));
				Mesh::Point avg_point = (point1 + point2 + current_point) / 3.0;

				local_average_area_set[i] += get_Area_of_Four_Points(current_point, (current_point + point1) / 2.0, avg_point, (current_point + point2) / 2.0);
			}
		}
	}

	return local_average_area_set;
}

std::vector<std::vector<double>> Mesh_Interpolation::get_Cot_Weight_Set(Mesh& mesh)
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

				cot_weight_set[i].emplace_back(cot_sum);
			}
			else if (!mesh.is_boundary(mesh.edge_handle(mesh.find_halfedge(mesh.vertex_handle(i), mesh.vertex_handle(vh_neighboor_vh_idx_set[(j + 1) % vh_neighboor_vh_idx_set.size()])))))
			{
				Mesh::Point point2 = mesh.point(mesh.vertex_handle(vh_neighboor_vh_idx_set[j]));
				Mesh::Point point3 = mesh.point(mesh.vertex_handle(vh_neighboor_vh_idx_set[(j + 1) % vh_neighboor_vh_idx_set.size()]));

				double cot_sum = get_Cot_of_Three_Points(current_point, point3, point2);

				cot_weight_set[i].emplace_back(cot_sum);
			}
			else
			{
				Mesh::Point point1 = mesh.point(mesh.vertex_handle(vh_neighboor_vh_idx_set[(j - 1 + vh_neighboor_vh_idx_set.size()) % vh_neighboor_vh_idx_set.size()]));
				Mesh::Point point2 = mesh.point(mesh.vertex_handle(vh_neighboor_vh_idx_set[j]));

				double cot_sum = get_Cot_of_Three_Points(current_point, point1, point2);

				cot_weight_set[i].emplace_back(cot_sum);
			}
		}
	}

	return cot_weight_set;
}

bool Mesh_Interpolation::update_Source_Mesh(Mesh& mesh)
{
	if (vh_source_mesh_interpolation_set.size() != mesh.n_vertices())
	{
		need_to_update_matrix = true;
		need_to_update_matrix_set = true;

		vh_source_mesh_interpolation_set.resize(mesh.n_vertices());
	}

	for (int i = 0; i < mesh.n_vertices(); ++i)
	{
		vh_source_mesh_interpolation_set[i].resize(3);

		for (int j = 0; j < 3; ++j)
		{
			if (vh_source_mesh_interpolation_set[i][j] != mesh.point(mesh.vertex_handle(i)).data()[j])
			{
				need_to_update_matrix = true;
				need_to_update_matrix_set = true;

				vh_source_mesh_interpolation_set[i][j] = mesh.point(mesh.vertex_handle(i)).data()[j];
			}
		}
	}

	return true;
}

bool Mesh_Interpolation::update_Target_Mesh(Mesh& mesh)
{
	if (vh_target_mesh_interpolation_set.size() != mesh.n_vertices())
	{
		need_to_update_matrix = true;
		need_to_update_matrix_set = true;

		vh_target_mesh_interpolation_set.resize(mesh.n_vertices());
	}

	for (int i = 0; i < mesh.n_vertices(); ++i)
	{
		vh_target_mesh_interpolation_set[i].resize(3);

		for (int j = 0; j < 3; ++j)
		{
			if (vh_target_mesh_interpolation_set[i][j] != mesh.point(mesh.vertex_handle(i)).data()[j])
			{
				need_to_update_matrix = true;
				need_to_update_matrix_set = true;

				vh_target_mesh_interpolation_set[i][j] = mesh.point(mesh.vertex_handle(i)).data()[j];
			}
		}
	}

	return true;
}

bool Mesh_Interpolation::update_Matrix()
{
	if (need_to_update_matrix)
	{
		std::cout << "Start Update Matrix ..." << std::endl;

		x.resize(2, vh_source_mesh_interpolation_set.size());

		MatrixXd b(2, vh_target_mesh_interpolation_set.size());

		for (int i = 0; i < vh_source_mesh_interpolation_set.size(); ++i)
		{
			for (int j = 0; j < 2; ++j)
			{
				x(j, i) = vh_source_mesh_interpolation_set[i][j];
				b(j, i) = vh_target_mesh_interpolation_set[i][j];
			}
		}

		A.resize(vh_source_mesh_interpolation_set.size(), vh_source_mesh_interpolation_set.size());

		FullPivHouseholderQR<MatrixXd>* Solver = new FullPivHouseholderQR<MatrixXd>(x);

		A = Solver->solve(b).transpose();

		U.resize(vh_source_mesh_interpolation_set.size(), vh_source_mesh_interpolation_set.size());
		V.resize(vh_source_mesh_interpolation_set.size(), vh_source_mesh_interpolation_set.size());
		Sigma.resize(vh_source_mesh_interpolation_set.size());
		E.resize(vh_source_mesh_interpolation_set.size(), vh_source_mesh_interpolation_set.size());
		R.resize(vh_source_mesh_interpolation_set.size(), vh_source_mesh_interpolation_set.size());
		S.resize(vh_source_mesh_interpolation_set.size(), vh_source_mesh_interpolation_set.size());

		JacobiSVD<MatrixXd> svd(A, ComputeFullU | ComputeFullV);

		U = svd.matrixU();
		V = svd.matrixV();
		Sigma = svd.singularValues();

		E.setZero();

		for (int i = 0; i < vh_source_mesh_interpolation_set.size(); ++i)
		{
			E(i, i) = Sigma(i);
		}

		R = U * V.transpose();

		S = V * E * V.transpose();

		need_to_update_matrix = false;

		std::cout << "Finish Update Matrix !" << std::endl;
	}

	return true;
}

bool Mesh_Interpolation::get_Matrix_and_Radian(std::vector<std::vector<int>> fvh_idx_set)
{
	Matrix_A_set.resize(fvh_idx_set.size());
	Matrix_U_set.resize(fvh_idx_set.size());
	Matrix_V_set.resize(fvh_idx_set.size());
	Matrix_E_set.resize(fvh_idx_set.size());
	Matrix_R_set.resize(fvh_idx_set.size());
	Matrix_S_set.resize(fvh_idx_set.size());
	Rotation_Radian_set.resize(fvh_idx_set.size());
	Matrix_P_inverse_set.resize(fvh_idx_set.size());

	for (int i = 0; i < fvh_idx_set.size(); ++i)
	{
		Matrix3d P;
		MatrixXd Q(3, 2);

		for (int j = 0; j < 3; ++j)
		{
			P(j, 0) = vh_source_mesh_interpolation_set[fvh_idx_set[i][j]][0];
			P(j, 1) = vh_source_mesh_interpolation_set[fvh_idx_set[i][j]][1];
			P(j, 2) = 1.0;

			Q(j, 0) = vh_target_mesh_interpolation_set[fvh_idx_set[i][j]][0];
			Q(j, 1) = vh_target_mesh_interpolation_set[fvh_idx_set[i][j]][1];
		}

		Matrix_P_inverse_set[i] = P.inverse();

		MatrixXd A_current = Matrix_P_inverse_set[i] * Q;

		for (int j = 0; j < 2; ++j)
		{
			for (int k = 0; k < 2; ++k)
			{
				Matrix_A_set[i](j, k) = A_current(k, j);
			}
		}

		JacobiSVD<Matrix2d> svd(Matrix_A_set[i], ComputeFullU | ComputeFullV);

		Matrix_U_set[i] = svd.matrixU();
		Matrix_V_set[i] = svd.matrixV();
		Sigma = svd.singularValues();
		Matrix_E_set[i] = Matrix2d::Identity();
		Matrix_E_set[i](0, 0) = Sigma(0);
		Matrix_E_set[i](1, 1) = Sigma(1);

		Matrix_R_set[i] = Matrix_U_set[i] * Matrix_V_set[i].transpose();
		Matrix_S_set[i] = Matrix_V_set[i] * Matrix_E_set[i] * Matrix_V_set[i].transpose();

		double current_cos = Matrix_R_set[i](0, 0);
		double current_sin = Matrix_R_set[i](0, 1);

		if (current_cos >= 0)
		{
			Rotation_Radian_set[i] = asin(current_sin);
		}
		else
		{
			if (current_sin >= 0)
			{
				Rotation_Radian_set[i] = acos(current_cos);
			}
			else
			{
				Rotation_Radian_set[i] = -acos(current_cos);
			}
		}
	}

	return true;
}

bool Mesh_Interpolation::get_A_t(double t)
{
	Matrix_A_t_set.resize(Matrix_A_set.size());

	for (int i = 0; i < Matrix_A_t_set.size(); ++i)
	{
		Matrix2d R_t;
		R_t(0, 0) = cos(t * Rotation_Radian_set[i]);
		R_t(0, 1) = sin(t * Rotation_Radian_set[i]);
		R_t(1, 0) = -sin(t * Rotation_Radian_set[i]);
		R_t(1, 1) = cos(t * Rotation_Radian_set[i]);

		Matrix2d S_t = t * Matrix_S_set[i];
		S_t(0, 0) += 1.0 - t;
		S_t(1, 1) += 1.0 - t;

		Matrix_A_t_set[i] = R_t * S_t;
	}

	return true;
}

std::vector<std::vector<double>> Mesh_Interpolation::get_Mesh_Interpolation_Result(Mesh& mesh, int method, double t)
{
	std::vector<std::vector<double>> vh_mesh_interpolation_result;

	vh_mesh_interpolation_result.resize(vh_source_mesh_interpolation_set.size());

	MatrixXd b(vh_source_mesh_interpolation_set.size(), 2);

	std::cout << "Start Solve ..." << std::endl;
	if (method == 1)
	{
		update_Matrix();

		MatrixXd A_t = t * A;
		for (int i = 0; i < vh_source_mesh_interpolation_set.size(); ++i)
		{
			A_t(i, i) += 1.0 - t;
		}

		b = A_t * x.transpose();

		for (int i = 0; i < vh_mesh_interpolation_result.size(); ++i)
		{
			vh_mesh_interpolation_result[i].resize(3);

			for (int j = 0; j < 2; ++j)
			{
				vh_mesh_interpolation_result[i][j] = b(i, j);
			}
			vh_mesh_interpolation_result[i][2] = 0;
		}
	}
	else if (method == 2)
	{
		update_Matrix();

		MatrixXd U_t = t * U;
		MatrixXd VT_t = t * V.transpose();
		MatrixXd E_t = t * E;
		for (int i = 0; i < vh_source_mesh_interpolation_set.size(); ++i)
		{
			U_t(i, i) += 1.0 - t;
			VT_t(i, i) += 1.0 - t;
			E_t(i, i) += 1.0 - t;
		}

		b = U_t * E_t * VT_t * x.transpose();
	}
	else if (method == 3)
	{
		std::vector<std::vector<int>> fvh_idx_set = get_FVH_Idx_Set(mesh);

		std::vector<std::vector<int>> vfh_idx_set = get_VFH_Idx_Set(mesh);

		if (need_to_update_matrix_set)
		{
			std::cout << "Start update Matrix set ..." << std::endl;

			get_Matrix_and_Radian(fvh_idx_set);

			std::cout << "Finish update Matrix set !" << std::endl;
		}

		get_A_t(t);

		std::cout << "Start update A_t and b_t ..." << std::endl;

		MatrixXd A_t(mesh.n_vertices(), mesh.n_vertices());
		MatrixXd x_t(mesh.n_vertices(), 2);
		MatrixXd b_t(mesh.n_vertices(), 2);

		A_t.setZero();
		b_t.setZero();

		for (int i = 0; i < mesh.n_vertices(); ++i)
		{
			for (int j = 0; j < vfh_idx_set[i].size(); ++j)
			{
				int current_fh_idx = vfh_idx_set[i][j];

				int vh_i_idx_position = -1;

				for (int k = 0; k < 3; ++k)
				{
					if (fvh_idx_set[current_fh_idx][k] == i)
					{
						vh_i_idx_position = k;

						break;
					}
				}

				for (int k = 0; k < 3; ++k)
				{
					A_t(i, fvh_idx_set[current_fh_idx][k]) += Matrix_P_inverse_set[current_fh_idx](0, vh_i_idx_position) * Matrix_P_inverse_set[current_fh_idx](0, k);
					A_t(i, fvh_idx_set[current_fh_idx][k]) += Matrix_P_inverse_set[current_fh_idx](1, vh_i_idx_position) * Matrix_P_inverse_set[current_fh_idx](1, k);
				}

				b_t(i, 0) += Matrix_P_inverse_set[current_fh_idx](0, vh_i_idx_position) * Matrix_A_t_set[current_fh_idx](0, 0);
				b_t(i, 0) += Matrix_P_inverse_set[current_fh_idx](1, vh_i_idx_position) * Matrix_A_t_set[current_fh_idx](0, 1);
				b_t(i, 1) += Matrix_P_inverse_set[current_fh_idx](0, vh_i_idx_position) * Matrix_A_t_set[current_fh_idx](1, 0);
				b_t(i, 1) += Matrix_P_inverse_set[current_fh_idx](1, vh_i_idx_position) * Matrix_A_t_set[current_fh_idx](1, 1);
			}
		}

		A_t(0, 0) += 1.0;
		b_t(0, 0) += (1.0 - t) * vh_source_mesh_interpolation_set[0][0] + t * vh_target_mesh_interpolation_set[0][0];
		b_t(0, 1) += (1.0 - t) * vh_source_mesh_interpolation_set[0][1] + t * vh_target_mesh_interpolation_set[0][1];

		std::cout << "Finish update A_t and b_t !" << std::endl;
		std::cout << "Start solve A_t and b_t ..." << std::endl;

		if (need_to_update_matrix_set)
		{
			//A_t_inverse = (A_t.transpose() * A_t).inverse();
			A_t_inverse = A_t.inverse();

			need_to_update_matrix_set = false;
		}

		//FullPivHouseholderQR<MatrixXd>* Solver = new FullPivHouseholderQR<MatrixXd>(A_t);

		//x_t = Solver->solve(b_t);

		//x_t = A_t_inverse * A_t.transpose() * b_t;
		x_t = A_t_inverse * b_t;

		std::cout << "Finish solve A_t and b_t !" << std::endl;

		for (int i = 0; i < vh_mesh_interpolation_result.size(); ++i)
		{
			vh_mesh_interpolation_result[i].resize(3);

			for (int j = 0; j < 2; ++j)
			{
				vh_mesh_interpolation_result[i][j] = x_t(i, j);
			}
			vh_mesh_interpolation_result[i][2] = 0;
		}
	}
	std::cout << "Finish Get Result !" << std::endl;

	return vh_mesh_interpolation_result;
}
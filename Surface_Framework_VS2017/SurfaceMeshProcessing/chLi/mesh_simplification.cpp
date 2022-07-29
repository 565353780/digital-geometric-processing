#include "mesh_simplification.h"

Mesh_Simplification::Mesh_Simplification()
{
	
}

Mesh_Simplification::~Mesh_Simplification()
{

}

double Mesh_Simplification::get_Dist_2_of_Points(Mesh::Point point_1, Mesh::Point point_2)
{
	double dist_2_of_points = 0;

	dist_2_of_points += (point_1.data()[0] - point_2.data()[0]) * (point_1.data()[0] - point_2.data()[0]);
	dist_2_of_points += (point_1.data()[1] - point_2.data()[1]) * (point_1.data()[1] - point_2.data()[1]);
	dist_2_of_points += (point_1.data()[2] - point_2.data()[2]) * (point_1.data()[2] - point_2.data()[2]);

	return dist_2_of_points;
}

double Mesh_Simplification::get_Dist_of_Points(Mesh::Point point_1, Mesh::Point point_2)
{
	return sqrt(get_Dist_2_of_Points(point_1, point_2));
}

double Mesh_Simplification::get_Norm_of_Point(Mesh::Point point)
{
	Mesh::Point zero(0, 0, 0);

	return sqrt(get_Dist_2_of_Points(point, zero));
}

double Mesh_Simplification::get_Norm_of_Vector(std::vector<double> x)
{
	double dist_to_zero = x[0] * x[0] + x[1] * x[1] + x[2] * x[2];

	return sqrt(dist_to_zero);
}

bool Mesh_Simplification::is_In_Set(std::vector<int> set, int data)
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

double Mesh_Simplification::get_Radian_of_Three_Points(Mesh::Point point_1, Mesh::Point point_mid, Mesh::Point point_2)
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

double Mesh_Simplification::get_Cot_of_Three_Points(Mesh::Point point_1, Mesh::Point point_mid, Mesh::Point point_2)
{
	double tan_of_three_points = tan(get_Radian_of_Three_Points(point_1, point_mid, point_2));

	if (tan_of_three_points != 0)
	{
		return 1.0 / tan_of_three_points;
	}

	std::cout << "tan is zero!" << std::endl;

	return 0;
}

double Mesh_Simplification::get_Area_of_Three_Points(Mesh::Point point_1, Mesh::Point point_2, Mesh::Point point_3)
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

double Mesh_Simplification::get_Area_of_Four_Points(Mesh::Point point_1, Mesh::Point point_2, Mesh::Point point_3, Mesh::Point point_4)
{
	return get_Area_of_Three_Points(point_1, point_2, point_3) + get_Area_of_Three_Points(point_1, point_3, point_4);
}

double Mesh_Simplification::get_Area_of_Face_by_Face_Idx(Mesh& mesh, int fh_idx)
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

std::vector<double> Mesh_Simplification::get_Norm_Vector_of_Triangle_by_Three_Points(Mesh::Point point_1, Mesh::Point point_2, Mesh::Point point_3)
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

std::vector<double> Mesh_Simplification::get_Norm_Vector_of_Triangle_by_Face_Idx(Mesh& mesh, int fh_idx)
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

std::vector<double> Mesh_Simplification::get_Center_of_Triangle_by_Face_Idx(Mesh& mesh, int fh_idx)
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

std::vector<int> Mesh_Simplification::get_Neighboor_Face_Idx_Set(Mesh& mesh, int fh_idx)
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

std::vector<std::vector<int>> Mesh_Simplification::get_FVH_Idx_Set(Mesh& mesh)
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

std::vector<std::vector<int>> Mesh_Simplification::get_VFH_Idx_Set(Mesh& mesh)
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

std::vector<double> Mesh_Simplification::get_Face_Area_Set(Mesh& mesh)
{
	std::vector<double> face_area_set;

	face_area_set.resize(mesh.n_faces());

	for (int i = 0; i < mesh.n_faces(); ++i)
	{
		face_area_set[i] = get_Area_of_Face_by_Face_Idx(mesh, i);
	}

	return face_area_set;
}

std::vector<double> Mesh_Simplification::get_Local_Average_Area_Set_of_VH(Mesh& mesh)
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

std::vector<Vector4d> Mesh_Simplification::get_Norm_Tuta_Set(Mesh& mesh)
{
	std::vector<Vector4d> norm_tuta_set;

	norm_tuta_set.resize(mesh.n_faces());

	for (int i = 0; i < mesh.n_faces(); ++i)
	{
		std::vector<double> current_norm = get_Norm_Vector_of_Triangle_by_Face_Idx(mesh, i);

		Vector3d current_norm_vector;

		for (int j = 0; j < 3; ++j)
		{
			current_norm_vector(j) = current_norm[j];
		}

		int current_fvh_idx = mesh.fv_iter(mesh.face_handle(i))->idx();
		Vector3d current_vh_vector;

		for (int j = 0; j < 3; ++j)
		{
			current_vh_vector(j) = mesh.point(mesh.vertex_handle(current_fvh_idx)).data()[j];
		}

		double current_d = current_norm_vector.transpose() * current_vh_vector;

		for (int j = 0; j < 3; ++j)
		{
			norm_tuta_set[i](j) = current_norm_vector(j);
		}

		norm_tuta_set[i](3) = -current_d;
	}

	return norm_tuta_set;
}

std::vector<Matrix4d> Mesh_Simplification::get_Face_Quadratic_error_Matrix_Set(Mesh& mesh)
{
	std::vector<Vector4d> norm_tuta_set = get_Norm_Tuta_Set(mesh);

	std::vector<Matrix4d> face_quadratic_error_matrix_set;

	face_quadratic_error_matrix_set.resize(mesh.n_faces());

	for (int i = 0; i < mesh.n_faces(); ++i)
	{
		face_quadratic_error_matrix_set[i] = norm_tuta_set[i] * norm_tuta_set[i].transpose();
	}

	return face_quadratic_error_matrix_set;
}

std::vector<Matrix4d> Mesh_Simplification::get_Vertex_Quadratic_error_Matrix_Set(Mesh& mesh)
{
	std::vector<Matrix4d> face_quadratic_error_matrix_set = get_Face_Quadratic_error_Matrix_Set(mesh);

	std::vector<std::vector<int>> vfh_idx_set = get_VFH_Idx_Set(mesh);

	std::vector<Matrix4d> vertex_quadratic_error_matrix_set;

	vertex_quadratic_error_matrix_set.resize(mesh.n_vertices());

	for (int i = 0; i < mesh.n_vertices(); ++i)
	{
		vertex_quadratic_error_matrix_set[i].setZero();

		for (int j = 0; j < vfh_idx_set[i].size(); ++j)
		{
			vertex_quadratic_error_matrix_set[i] += face_quadratic_error_matrix_set[vfh_idx_set[i][j]];
		}
	}

	return vertex_quadratic_error_matrix_set;
}

std::vector<Matrix4d> Mesh_Simplification::get_Edge_Quadratic_error_Matrix_Set(Mesh& mesh)
{
	std::vector<Matrix4d> vertex_quadratic_error_matrix_set = get_Vertex_Quadratic_error_Matrix_Set(mesh);

	std::vector<Matrix4d> edge_quadratic_error_matrix_set;

	edge_quadratic_error_matrix_set.resize(mesh.n_edges());

	for (int i = 0; i < mesh.n_edges(); ++i)
	{
		edge_quadratic_error_matrix_set[i].setZero();

		Mesh::HalfedgeHandle current_heh = mesh.halfedge_handle(mesh.edge_handle(i), 0);

		int ev_idx_1 = mesh.from_vertex_handle(current_heh).idx();
		int ev_idx_2 = mesh.to_vertex_handle(current_heh).idx();

		edge_quadratic_error_matrix_set[i] += vertex_quadratic_error_matrix_set[ev_idx_1];
		edge_quadratic_error_matrix_set[i] += vertex_quadratic_error_matrix_set[ev_idx_2];
	}

	return edge_quadratic_error_matrix_set;
}

std::vector<double> Mesh_Simplification::get_Edge_Quadratic_error_Set(Mesh& mesh)
{
	std::vector<Matrix4d> edge_quadratic_error_matrix_set = get_Edge_Quadratic_error_Matrix_Set(mesh);

	std::vector<double> edge_quadratic_error_set;

	edge_quadratic_error_set.resize(mesh.n_edges());

	for (int i = 0; i < mesh.n_edges(); ++i)
	{
		Mesh::HalfedgeHandle current_heh = mesh.halfedge_handle(mesh.edge_handle(i), 0);

		int ev_idx_1 = mesh.from_vertex_handle(current_heh).idx();
		int ev_idx_2 = mesh.to_vertex_handle(current_heh).idx();

		Mesh::Point ev_1_point = mesh.point(mesh.vertex_handle(ev_idx_1));
		Mesh::Point ev_1_to_2_point = mesh.point(mesh.vertex_handle(ev_idx_2)) - ev_1_point;

		Vector4d ev_1_vector;
		Vector4d ev_1_to_2_vector;

		for (int j = 0; j < 3; ++j)
		{
			ev_1_vector(j) = ev_1_point.data()[j];
			ev_1_to_2_vector(j) = ev_1_to_2_point.data()[j];
		}

		ev_1_vector(3) = 1;
		ev_1_to_2_vector(3) = 1;

		double lambda_b1 = ev_1_to_2_vector.transpose() * edge_quadratic_error_matrix_set[i] * ev_1_vector;
		double lambda_b2 = ev_1_vector.transpose() * edge_quadratic_error_matrix_set[i] * ev_1_to_2_vector;

		double lambda_a = ev_1_to_2_vector.transpose() * edge_quadratic_error_matrix_set[i] * ev_1_to_2_vector;

		double lambda = -(lambda_b1 + lambda_b2) / lambda_a / 2.0;

		if (lambda < 0)
		{
			//std::cout << "edge : " << i << " , lambda : " << lambda << std::endl;
			lambda = 0;
		}
		else if (lambda > 1)
		{
			//std::cout << "edge : " << i << " , lambda : " << lambda << std::endl;
			lambda = 1;
		}

		lambda = 0.5;

		Vector4d ev_tuta_vector = ev_1_vector + lambda * ev_1_to_2_vector;

		ev_tuta_vector(3) = 1;

		edge_quadratic_error_set[i] = ev_tuta_vector.transpose() * edge_quadratic_error_matrix_set[i] * ev_tuta_vector;
	}

	return edge_quadratic_error_set;
}

std::vector<double> Mesh_Simplification::get_Edge_Quadratic_error_Set(Mesh& mesh, std::vector<Matrix4d> edge_quadratic_error_matrix_set)
{
	std::vector<double> edge_quadratic_error_set;

	edge_quadratic_error_set.resize(mesh.n_edges());

	for (int i = 0; i < mesh.n_edges(); ++i)
	{
		Mesh::HalfedgeHandle current_heh = mesh.halfedge_handle(mesh.edge_handle(i), 0);

		int ev_idx_1 = mesh.from_vertex_handle(current_heh).idx();
		int ev_idx_2 = mesh.to_vertex_handle(current_heh).idx();

		Mesh::Point ev_1_point = mesh.point(mesh.vertex_handle(ev_idx_1));
		Mesh::Point ev_1_to_2_point = mesh.point(mesh.vertex_handle(ev_idx_2)) - ev_1_point;

		Vector4d ev_1_vector;
		Vector4d ev_1_to_2_vector;

		for (int j = 0; j < 3; ++j)
		{
			ev_1_vector(j) = ev_1_point.data()[j];
			ev_1_to_2_vector(j) = ev_1_to_2_point.data()[j];
		}

		ev_1_vector(3) = 1;
		ev_1_to_2_vector(3) = 1;

		double lambda_b1 = ev_1_to_2_vector.transpose() * edge_quadratic_error_matrix_set[i] * ev_1_vector;
		double lambda_b2 = ev_1_vector.transpose() * edge_quadratic_error_matrix_set[i] * ev_1_to_2_vector;

		double lambda_a = ev_1_to_2_vector.transpose() * edge_quadratic_error_matrix_set[i] * ev_1_to_2_vector;

		double lambda = -(lambda_b1 + lambda_b2) / lambda_a / 2.0;

		if (lambda < 0)
		{
			//std::cout << "edge : " << i << " , lambda : " << lambda << std::endl;
			lambda = 0;
		}
		else if (lambda > 1)
		{
			//std::cout << "edge : " << i << " , lambda : " << lambda << std::endl;
			lambda = 1;
		}

		lambda = 0.5;

		Vector4d ev_tuta_vector = ev_1_vector + lambda * ev_1_to_2_vector;

		ev_tuta_vector(3) = 1;

		edge_quadratic_error_set[i] = ev_tuta_vector.transpose() * edge_quadratic_error_matrix_set[i] * ev_tuta_vector;
	}

	return edge_quadratic_error_set;
}

bool Mesh_Simplification::get_Mesh_Simplification_Result(Mesh& mesh, int target_vertex_num)
{
	/*std::cout << "before simplification : " << std::endl;

	std::cout << "vertex : " << std::endl;
	for (int i = 0; i < mesh.n_vertices(); ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			std::cout << mesh.point(mesh.vertex_handle(i)).data()[j] << ",";
		}
		std::cout << std::endl;
	}

	std::cout << "edge : " << std::endl;
	for (int i = 0; i < mesh.n_edges(); ++i)
	{
		Mesh::HalfedgeHandle heh = mesh.halfedge_handle(mesh.edge_handle(i), 0);
		std::cout << mesh.from_vertex_handle(heh).idx() << ",";
		std::cout << mesh.to_vertex_handle(heh).idx() << ",";
		std::cout << std::endl;
	}

	std::cout << "face : " << std::endl;
	for (int i = 0; i < mesh.n_faces(); ++i)
	{
		Mesh::FaceVertexIter fv_it = mesh.fv_iter(mesh.face_handle(i));
		for (int j = 0; j < 3; ++j)
		{
			std::cout << fv_it->idx() << ",";
			++fv_it;
		}
		std::cout << std::endl;
	}*/

	int unused_vh_num = 0;

	while(mesh.n_vertices() > target_vertex_num && target_vertex_num > 2)
	{
		if (unused_vh_num >= mesh.n_edges())
		{
			break;
		}

		//std::cout << "in process with vertex num : " << mesh.n_vertices() << std::endl;

		std::vector<Matrix4d> edge_quadratic_error_matrix_set = get_Edge_Quadratic_error_Matrix_Set(mesh);

		std::vector<double> edge_quadratic_error_set = get_Edge_Quadratic_error_Set(mesh, edge_quadratic_error_matrix_set);

		double min_error;
		std::vector<int> min_error_idx_set;

		min_error_idx_set.resize(unused_vh_num + 1);

		for (int i = 0; i < unused_vh_num + 1; ++i)
		{
			min_error_idx_set[i] = -1;
		}

		for (int i = 0; i < unused_vh_num + 1; ++i)
		{
			for (int j = 0; j < mesh.n_edges(); ++j)
			{
				Mesh::HalfedgeHandle heh = mesh.halfedge_handle(mesh.edge_handle(j), 0);

				if (mesh.is_boundary(mesh.from_vertex_handle(heh)) || mesh.is_boundary(mesh.to_vertex_handle(heh)))
				{
					continue;
				}

				bool have_saved = false;

				for (int k = 0; k < i; ++k)
				{
					if (j == min_error_idx_set[k])
					{
						have_saved = true;
						break;
					}
				}

				if (have_saved)
				{
					continue;
				}

				if (!mesh.is_boundary(mesh.edge_handle(j)))
				{
					if (min_error_idx_set[i] == -1)
					{
						min_error = edge_quadratic_error_set[j];
						min_error_idx_set[i] = j;
					}
					else if (edge_quadratic_error_set[j] < min_error)
					{
						min_error = edge_quadratic_error_set[j];
						min_error_idx_set[i] = j;
					}
				}
			}
		}

		/*std::cout << "unused_vh_num : " << unused_vh_num << std::endl;

		for (int i = 0; i < min_error_idx_set.size(); ++i)
		{
			std::cout << min_error_idx_set[i] << std::endl;
		}*/

		int min_error_idx = min_error_idx_set[unused_vh_num];

		//std::cout << "current min error : " << min_error << std::endl;

		Mesh::HalfedgeHandle current_heh = mesh.halfedge_handle(mesh.edge_handle(min_error_idx), 0);

		if (!mesh.is_collapse_ok(current_heh))
		{
			++unused_vh_num;
			continue;
		}

		unused_vh_num = 0;

		int ev_idx_1 = mesh.from_vertex_handle(current_heh).idx();
		int ev_idx_2 = mesh.to_vertex_handle(current_heh).idx();

		Mesh::Point ev_1_point = mesh.point(mesh.vertex_handle(ev_idx_1));
		Mesh::Point ev_1_to_2_point = mesh.point(mesh.vertex_handle(ev_idx_2)) - ev_1_point;

		Vector4d ev_1_vector;
		Vector4d ev_1_to_2_vector;

		for (int j = 0; j < 3; ++j)
		{
			ev_1_vector(j) = ev_1_point.data()[j];
			ev_1_to_2_vector(j) = ev_1_to_2_point.data()[j];
		}

		ev_1_vector(3) = 1;
		ev_1_to_2_vector(3) = 1;

		double lambda_b1 = ev_1_to_2_vector.transpose() * edge_quadratic_error_matrix_set[min_error_idx] * ev_1_vector;
		double lambda_b2 = ev_1_vector.transpose() * edge_quadratic_error_matrix_set[min_error_idx] * ev_1_to_2_vector;

		double lambda_a = ev_1_to_2_vector.transpose() * edge_quadratic_error_matrix_set[min_error_idx] * ev_1_to_2_vector;

		double lambda = -(lambda_b1 + lambda_b2) / lambda_a / 2.0;

		if (lambda < 0)
		{
			//std::cout << "edge : " << i << " , lambda : " << lambda << std::endl;
			lambda = 0;
		}
		else if (lambda > 1)
		{
			//std::cout << "edge : " << i << " , lambda : " << lambda << std::endl;
			lambda = 1;
		}

		lambda = 0.5;

		Vector4d ev_tuta_vector = ev_1_vector + lambda * ev_1_to_2_vector;

		/*for (int j = 0; j < 3; ++j)
		{
			std::cout << mesh.point(mesh.vertex_handle(ev_idx_1)).data()[j] << std::endl;
		}
		for (int j = 0; j < 3; ++j)
		{
			std::cout << mesh.point(mesh.vertex_handle(ev_idx_2)).data()[j] - mesh.point(mesh.vertex_handle(ev_idx_1)).data()[j] << std::endl;
		}*/

		mesh.request_vertex_status();
		mesh.request_edge_status();
		mesh.request_face_status();

		mesh.collapse(current_heh);

		for (int j = 0; j < 3; ++j)
		{
			mesh.point(mesh.vertex_handle(ev_idx_2))[j] = ev_tuta_vector(j);
		}

		mesh.garbage_collection();
	}

	std::cout << "Finish Get Result !" << std::endl;

	return true;
}
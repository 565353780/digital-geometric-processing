#include "optimal_delaunay_triangulation.h"

Optimal_Delaunay_Triangulation::Optimal_Delaunay_Triangulation()
{

}

Optimal_Delaunay_Triangulation::~Optimal_Delaunay_Triangulation()
{

}

double Optimal_Delaunay_Triangulation::get_Dist_2_of_Points(Mesh::Point point_1, Mesh::Point point_2)
{
	double dist_2_of_points = 0;

	dist_2_of_points += (point_1.data()[0] - point_2.data()[0]) * (point_1.data()[0] - point_2.data()[0]);
	dist_2_of_points += (point_1.data()[1] - point_2.data()[1]) * (point_1.data()[1] - point_2.data()[1]);
	dist_2_of_points += (point_1.data()[2] - point_2.data()[2]) * (point_1.data()[2] - point_2.data()[2]);

	return dist_2_of_points;
}

double Optimal_Delaunay_Triangulation::get_Dist_of_Points(Mesh::Point point_1, Mesh::Point point_2)
{
	return sqrt(get_Dist_2_of_Points(point_1, point_2));
}

double Optimal_Delaunay_Triangulation::get_Dist_2_of_Vector(VectorXd& vector_1, VectorXd& vector_2)
{
	double dist_2 = 0;

	for (int i = 0; i < vector_1.size(); ++i)
	{
		dist_2 += (vector_1(i) - vector_2(i)) * (vector_1(i) - vector_2(i));
	}

	return dist_2;
}

double Optimal_Delaunay_Triangulation::get_Dist_2_of_Vector(RowVectorXd& vector_1, RowVectorXd& vector_2)
{
	double dist_2 = 0;

	for (int i = 0; i < vector_1.size(); ++i)
	{
		dist_2 += (vector_1(i) - vector_2(i)) * (vector_1(i) - vector_2(i));
	}

	return dist_2;
}

double Optimal_Delaunay_Triangulation::get_Dist_of_Vector(VectorXd& vector_1, VectorXd& vector_2)
{
	return sqrt(get_Dist_2_of_Vector(vector_1, vector_2));
}

double Optimal_Delaunay_Triangulation::get_Dist_of_Vector(RowVectorXd& vector_1, RowVectorXd& vector_2)
{
	return sqrt(get_Dist_2_of_Vector(vector_1, vector_2));
}

double Optimal_Delaunay_Triangulation::get_Norm_of_Point(Mesh::Point point)
{
	Mesh::Point zero(0, 0, 0);

	return sqrt(get_Dist_2_of_Points(point, zero));
}

double Optimal_Delaunay_Triangulation::get_Norm_of_Vector(std::vector<double> x)
{
	double dist_to_zero = x[0] * x[0] + x[1] * x[1] + x[2] * x[2];

	return sqrt(dist_to_zero);
}

bool Optimal_Delaunay_Triangulation::is_In_Set(std::vector<int> set, int data)
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

double Optimal_Delaunay_Triangulation::get_Radian_of_Three_Points(Mesh::Point point_1, Mesh::Point point_mid, Mesh::Point point_2)
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

double Optimal_Delaunay_Triangulation::get_Cot_of_Three_Points(Mesh::Point point_1, Mesh::Point point_mid, Mesh::Point point_2)
{
	double tan_of_three_points = tan(get_Radian_of_Three_Points(point_1, point_mid, point_2));

	if (tan_of_three_points != 0)
	{
		return 1.0 / tan_of_three_points;
	}

	std::cout << "tan is zero!" << std::endl;

	return 0;
}

double Optimal_Delaunay_Triangulation::get_Area_of_Three_Points(Mesh::Point point_1, Mesh::Point point_2, Mesh::Point point_3)
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

double Optimal_Delaunay_Triangulation::get_Area_of_Four_Points(Mesh::Point point_1, Mesh::Point point_2, Mesh::Point point_3, Mesh::Point point_4)
{
	return get_Area_of_Three_Points(point_1, point_2, point_3) + get_Area_of_Three_Points(point_1, point_3, point_4);
}

double Optimal_Delaunay_Triangulation::get_Area_of_Face_by_Face_Idx(Mesh& mesh, int fh_idx)
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

std::vector<double> Optimal_Delaunay_Triangulation::get_Norm_Vector_of_Triangle_by_Three_Points(Mesh::Point point_1, Mesh::Point point_2, Mesh::Point point_3)
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

std::vector<double> Optimal_Delaunay_Triangulation::get_Norm_Vector_of_Triangle_by_Face_Idx(Mesh& mesh, int fh_idx)
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

std::vector<double> Optimal_Delaunay_Triangulation::get_Center_of_Triangle_by_Face_Idx(Mesh& mesh, int fh_idx)
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

std::vector<int> Optimal_Delaunay_Triangulation::get_Neighboor_Face_Idx_Set(Mesh& mesh, int fh_idx)
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

std::vector<std::vector<int>> Optimal_Delaunay_Triangulation::get_FVH_Idx_Set(Mesh& mesh)
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

std::vector<std::vector<int>> Optimal_Delaunay_Triangulation::get_VFH_Idx_Set(Mesh& mesh)
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

std::vector<double> Optimal_Delaunay_Triangulation::get_Face_Area_Set(Mesh& mesh)
{
	std::vector<double> face_area_set;

	face_area_set.resize(mesh.n_faces());

	for (int i = 0; i < mesh.n_faces(); ++i)
	{
		face_area_set[i] = get_Area_of_Face_by_Face_Idx(mesh, i);
	}

	return face_area_set;
}

std::vector<double> Optimal_Delaunay_Triangulation::get_Local_Average_Area_Set_of_VH(Mesh& mesh)
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

Matrix<double, 1, 3> Optimal_Delaunay_Triangulation::get_Vector3d_from_VH(Mesh& mesh, int vh_idx)
{
	Matrix<double, 1, 3> v;

	Mesh::Point point = mesh.point(mesh.vertex_handle(vh_idx));

	for (int i = 0; i < 3; ++i)
	{
		v(i) = point.data()[i];
	}

	return v;
}

MatrixXd Optimal_Delaunay_Triangulation::get_Circum_Circle_Center(Mesh& mesh)
{
	MatrixXd circum_circle_center(mesh.n_faces(), 4);

	circum_circle_center.setZero();

	for (int i = 0; i < mesh.n_faces(); ++i)
	{
		Mesh::FaceVertexIter fv_it = mesh.fv_iter(mesh.face_handle(i));

		RowVectorXd p1 = get_Vector3d_from_VH(mesh, fv_it->idx()).block(0, 0, 1, 2);
		++fv_it;
		RowVectorXd p2 = get_Vector3d_from_VH(mesh, fv_it->idx()).block(0, 0, 1, 2);
		++fv_it;
		RowVectorXd p3 = get_Vector3d_from_VH(mesh, fv_it->idx()).block(0, 0, 1, 2);

		double A1 = 2.0 * (p2(0) - p1(0));
		double B1 = 2.0 * (p2(1) - p1(1));
		double C1 = p2(0) * p2(0) + p2(1) * p2(1) - p1(0) * p1(0) - p1(1) * p1(1);
		double A2 = 2.0 * (p3(0) - p2(0));
		double B2 = 2.0 * (p3(1) - p2(1));
		double C2 = p3(0) * p3(0) + p3(1) * p3(1) - p2(0) * p2(0) - p2(1) * p2(1);

		circum_circle_center(i, 0) = (C1 * B2 - C2 * B1) / (A1 * B2 - A2 * B1);
		circum_circle_center(i, 1) = (A1 * C2 - A2 * C1) / (A1 * B2 - A2 * B1);

		RowVectorXd center = circum_circle_center.block(i, 0, 1, 2);

		circum_circle_center(i, 3) = get_Dist_of_Vector(p1, center);
	}

	return circum_circle_center;
}

bool Optimal_Delaunay_Triangulation::update_Delaunay_Triangulation(Mesh& mesh)
{
	bool all_finished = false;

	while (!all_finished)
	{
		all_finished = true;

		for (int i = 0; i < mesh.n_edges(); ++i)
		{
			MatrixXd circum_circle_center = get_Circum_Circle_Center(mesh);

			Mesh::EdgeHandle eh = mesh.edge_handle(i);

			if (mesh.is_boundary(eh) || !mesh.is_flip_ok(eh))
			{
				continue;
			}

			Mesh::HalfedgeHandle heh = mesh.halfedge_handle(eh, 0);

			Mesh::FaceHandle fh_1 = mesh.face_handle(heh);
			//Mesh::FaceHandle fh_2 = mesh.face_handle(mesh.opposite_halfedge_handle(heh));

			RowVectorXd current_circle_center_1 = circum_circle_center.block(fh_1.idx(), 0, 1, 2);
			//VectorXd current_circle_center_2 = circum_circle_center.block(fh_2.idx(), 0, 1, 2);

			//int far_away_vh_idx_1 = mesh.to_vertex_handle(mesh.next_halfedge_handle(heh)).idx();
			int far_away_vh_idx_2 = mesh.to_vertex_handle(mesh.next_halfedge_handle(mesh.opposite_halfedge_handle(heh))).idx();

			//VectorXd vec_1 = get_Vector3d_from_VH(mesh, far_away_vh_idx_1).block(0, 0, 1, 2);
			RowVectorXd vec_2 = get_Vector3d_from_VH(mesh, far_away_vh_idx_2).block(0, 0, 1, 2);

			if (get_Dist_of_Vector(current_circle_center_1, vec_2) < circum_circle_center(fh_1.idx(), 3))
			{
				mesh.flip(eh);

				all_finished = false;
			}
		}
	}

	return true;
}

bool Optimal_Delaunay_Triangulation::update_Vertex_Position_By_Circum_Center(Mesh& mesh)
{
	vector<vector<int>> vfh_idx_set = get_VFH_Idx_Set(mesh);

	for (int i = 0; i < mesh.n_vertices(); ++i)
	{
		MatrixXd circum_circle_center = get_Circum_Circle_Center(mesh);

		vector<double> f_area_set = get_Face_Area_Set(mesh);

		Mesh::VertexHandle vh = mesh.vertex_handle(i);

		if (mesh.is_boundary(vh))
		{
			continue;
		}

		double total_area = 0;

		RowVector2d target_vh_position;

		target_vh_position.setZero();

		for (int j = 0; j < vfh_idx_set[i].size(); ++j)
		{
			target_vh_position += f_area_set[vfh_idx_set[i][j]] * circum_circle_center.block(vfh_idx_set[i][j], 0, 1, 2);

			total_area += f_area_set[vfh_idx_set[i][j]];
		}

		if (total_area != 0)
		{
			target_vh_position /= total_area;

			for (int j = 0; j < 2; ++j)
			{
				mesh.point(vh)[j] = target_vh_position(i, j);
			}
		}
	}

	return true;
}

bool Optimal_Delaunay_Triangulation::get_Optimal_Delaunay_Triangulation_Result(Mesh& mesh, int solve_num)
{
	if (solve_num < 0)
	{
		return true;
	}

	for (int i = 0; i < solve_num; ++i)
	{
		update_Delaunay_Triangulation(mesh);

		update_Vertex_Position_By_Circum_Center(mesh);

		cout << "finish episode : " << i + 1 << endl;
	}

	std::cout << "Finish Get Result !" << std::endl;

	return true;
}

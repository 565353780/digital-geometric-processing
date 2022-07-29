#include "remeshing.h"

Remeshing::Remeshing()
{

}

Remeshing::~Remeshing()
{

}

double Remeshing::get_Dist_2_of_Points(Mesh::Point point_1, Mesh::Point point_2)
{
	double dist_2_of_points = 0;

	dist_2_of_points += (point_1.data()[0] - point_2.data()[0]) * (point_1.data()[0] - point_2.data()[0]);
	dist_2_of_points += (point_1.data()[1] - point_2.data()[1]) * (point_1.data()[1] - point_2.data()[1]);
	dist_2_of_points += (point_1.data()[2] - point_2.data()[2]) * (point_1.data()[2] - point_2.data()[2]);

	return dist_2_of_points;
}

double Remeshing::get_Dist_of_Points(Mesh::Point point_1, Mesh::Point point_2)
{
	return sqrt(get_Dist_2_of_Points(point_1, point_2));
}

double Remeshing::get_Norm_of_Point(Mesh::Point point)
{
	Mesh::Point zero(0, 0, 0);

	return sqrt(get_Dist_2_of_Points(point, zero));
}

double Remeshing::get_Norm_of_Vector(std::vector<double> x)
{
	double dist_to_zero = x[0] * x[0] + x[1] * x[1] + x[2] * x[2];

	return sqrt(dist_to_zero);
}

bool Remeshing::is_In_Set(std::vector<int> set, int data)
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

double Remeshing::get_Radian_of_Three_Points(Mesh::Point point_1, Mesh::Point point_mid, Mesh::Point point_2)
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

double Remeshing::get_Cot_of_Three_Points(Mesh::Point point_1, Mesh::Point point_mid, Mesh::Point point_2)
{
	double tan_of_three_points = tan(get_Radian_of_Three_Points(point_1, point_mid, point_2));

	if (tan_of_three_points != 0)
	{
		return 1.0 / tan_of_three_points;
	}

	std::cout << "tan is zero!" << std::endl;

	return 0;
}

double Remeshing::get_Area_of_Three_Points(Mesh::Point point_1, Mesh::Point point_2, Mesh::Point point_3)
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

double Remeshing::get_Area_of_Four_Points(Mesh::Point point_1, Mesh::Point point_2, Mesh::Point point_3, Mesh::Point point_4)
{
	return get_Area_of_Three_Points(point_1, point_2, point_3) + get_Area_of_Three_Points(point_1, point_3, point_4);
}

double Remeshing::get_Area_of_Face_by_Face_Idx(Mesh& mesh, int fh_idx)
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

std::vector<double> Remeshing::get_Norm_Vector_of_Triangle_by_Three_Points(Mesh::Point point_1, Mesh::Point point_2, Mesh::Point point_3)
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

std::vector<double> Remeshing::get_Norm_Vector_of_Triangle_by_Face_Idx(Mesh& mesh, int fh_idx)
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

std::vector<double> Remeshing::get_Center_of_Triangle_by_Face_Idx(Mesh& mesh, int fh_idx)
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

std::vector<int> Remeshing::get_Neighboor_Face_Idx_Set(Mesh& mesh, int fh_idx)
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

std::vector<std::vector<int>> Remeshing::get_FVH_Idx_Set(Mesh& mesh)
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

std::vector<std::vector<int>> Remeshing::get_VFH_Idx_Set(Mesh& mesh)
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

std::vector<double> Remeshing::get_Face_Area_Set(Mesh& mesh)
{
	std::vector<double> face_area_set;

	face_area_set.resize(mesh.n_faces());

	for (int i = 0; i < mesh.n_faces(); ++i)
	{
		face_area_set[i] = get_Area_of_Face_by_Face_Idx(mesh, i);
	}

	return face_area_set;
}

std::vector<double> Remeshing::get_Local_Average_Area_Set_of_VH(Mesh& mesh)
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

void Remeshing::get_Face_Local_Basis_Set(Mesh& mesh)
{
	face_local_basis_set.resize(mesh.n_faces());

	for (int i = 0; i < mesh.n_faces(); ++i)
	{
		face_local_basis_set[i].resize(3);

		Mesh::FaceVertexIter fv_it = mesh.fv_iter(mesh.face_handle(i));

		std::vector<Vector3d> fv_vector_set;

		fv_vector_set.resize(3);

		for (int j = 0; j < 3; ++j)
		{
			Mesh::Point current_point = mesh.point(mesh.vertex_handle(fv_it->idx()));

			for (int k = 0; k < 3; ++k)
			{
				fv_vector_set[j](k) = current_point.data()[k];
			}

			++fv_it;
		}

		face_local_basis_set[i][0] = (fv_vector_set[1] - fv_vector_set[0]).normalized();

		Vector3d v02 = fv_vector_set[2] - fv_vector_set[0];

		face_local_basis_set[i][2] = face_local_basis_set[i][0].cross(v02).normalized();

		face_local_basis_set[i][1] = -face_local_basis_set[i][0].cross(face_local_basis_set[i][2]).normalized();
	}
}

void Remeshing::get_Face_Bary_Center_Set(Mesh& mesh)
{
	face_bary_center_set.resize(mesh.n_faces());

	for (int i = 0; i < mesh.n_faces(); ++i)
	{
		std::vector<double> current_center = get_Center_of_Triangle_by_Face_Idx(mesh, i);

		for (int j = 0; j < 3; ++j)
		{
			face_bary_center_set[i](j) = current_center[j];
		}
	}
}

void Remeshing::get_Global_Edge_Scale(Mesh& mesh)
{
	global_edge_scale = 0;

	for (int i = 0; i < mesh.n_edges(); ++i)
	{
		global_edge_scale += mesh.calc_edge_length(mesh.edge_handle(i));
	}

	global_edge_scale /= mesh.n_edges();
}

Matrix<double, 1, 3> Remeshing::get_Vector3d_from_VH(Mesh& mesh, int vh_idx)
{
	Matrix<double, 1, 3> v;

	Mesh::Point point = mesh.point(mesh.vertex_handle(vh_idx));

	for (int i = 0; i < 3; ++i)
	{
		v(i) = point.data()[i];
	}

	return v;
}

MatrixXd Remeshing::random_constraints(const Matrix<double, 1, 3>& b1, const Matrix<double, 1, 3>& b2, int n)
{
	MatrixXd r(1, n * 3);
	for (unsigned i = 0; i < n; ++i)
	{
		double a = (double(rand()) / RAND_MAX) * 2 * M_PI;
		double s = 1 + ((double(rand()) / RAND_MAX)) * 5;
		Matrix<double, 1, 3> t = s * (cos(a) * b1 + sin(a) * b2);
		r.block(0, i * 3, 1, 3) = t;
	}
	return r;
}

bool Remeshing::split_long_edges(Mesh& mesh, double high)
{
	bool have_high_edge = true;

	while (have_high_edge)
	{
		have_high_edge = false;

		for (int i = 0; i < mesh.n_edges(); ++i)
		{
			Mesh::EdgeHandle eh = mesh.edge_handle(i);

			Mesh::HalfedgeHandle heh = mesh.halfedge_handle(eh, 0);

			double current_dist = get_Dist_of_Points(mesh.point(mesh.from_vertex_handle(heh)), mesh.point(mesh.to_vertex_handle(heh)));

			if (current_dist > high)
			{
				mesh.split(eh, (mesh.point(mesh.from_vertex_handle(heh)) + mesh.point(mesh.to_vertex_handle(heh))) / 2.0);

				have_high_edge = true;

				break;
			}
		}
	}

	mesh.request_face_normals();
	mesh.request_vertex_normals();
	mesh.request_halfedge_normals();

	mesh.update_normals();

	mesh.release_face_normals();
	mesh.release_vertex_normals();
	mesh.release_halfedge_normals();

	return true;
}

bool Remeshing::collapse_short_edges(Mesh& mesh, double low, double high)
{
	bool have_low_edge = true;

	while (have_low_edge)
	{
		have_low_edge = false;

		for (int i = 0; i < mesh.n_edges(); ++i)
		{
			Mesh::EdgeHandle eh = mesh.edge_handle(i);

			Mesh::HalfedgeHandle heh = mesh.halfedge_handle(eh, 0);

			double current_dist = get_Dist_of_Points(mesh.point(mesh.from_vertex_handle(heh)), mesh.point(mesh.to_vertex_handle(heh)));

			if (current_dist < low)
			{
				have_low_edge = true;

				Mesh::VertexHandle current_vh = mesh.from_vertex_handle(heh);

				int to_vh_idx = mesh.to_vertex_handle(heh).idx();

				Mesh::VertexVertexIter vv_it = mesh.vv_iter(current_vh);

				++vv_it;

				bool collapse_ok = true;

				while (vv_it->idx() != to_vh_idx)
				{
					if (get_Dist_of_Points(mesh.point(current_vh), mesh.point(vv_it)) > high)
					{
						collapse_ok = false;

						break;
					}

					++vv_it;
				}

				if (mesh.is_boundary(current_vh))
				{
					collapse_ok = false;
				}

				if (collapse_ok && mesh.is_collapse_ok(heh))
				{
					mesh.collapse(heh);

					mesh.garbage_collection();

					break;
				}
				else
				{
					have_low_edge = false;
				}
			}
		}
	}

	mesh.request_face_normals();
	mesh.request_vertex_normals();
	mesh.request_halfedge_normals();

	mesh.update_normals();

	mesh.release_face_normals();
	mesh.release_vertex_normals();
	mesh.release_halfedge_normals();

	return true;
}

bool Remeshing::equalize_valences(Mesh& mesh)
{
	for (int i = 0; i < mesh.n_edges(); ++i)
	{
		Mesh::EdgeHandle eh = mesh.edge_handle(i);

		if (!mesh.is_flip_ok(eh) || mesh.is_boundary(eh))
		{
			continue;
		}

		Mesh::HalfedgeHandle heh = mesh.halfedge_handle(eh, 0);

		Mesh::VertexHandle a, b, c, d;

		int target_a, target_b, target_c, target_d;

		Mesh::HalfedgeHandle search_heh = heh;

		a = mesh.from_vertex_handle(heh);

		b = mesh.to_vertex_handle(heh);

		c = mesh.to_vertex_handle(mesh.next_halfedge_handle(heh));

		d = mesh.to_vertex_handle(mesh.opposite_halfedge_handle(heh));

		if (mesh.is_boundary(a))
		{
			target_a = 4;
		}
		else
		{
			target_a = 6;
		}

		if (mesh.is_boundary(b))
		{
			target_b = 4;
		}
		else
		{
			target_b = 6;
		}

		if (mesh.is_boundary(c))
		{
			target_c = 4;
		}
		else
		{
			target_c = 6;
		}

		if (mesh.is_boundary(d))
		{
			target_d = 4;
		}
		else
		{
			target_d = 6;
		}

		int deviation_pre = abs(int(mesh.valence(a)) - target_a) + abs(int(mesh.valence(b)) - target_b) + abs(int(mesh.valence(c)) - target_c) + abs(int(mesh.valence(d)) - target_d);

		int deviation_post = abs(int(mesh.valence(a) - 1) - target_a) + abs(int(mesh.valence(b) - 1) - target_b) + abs(int(mesh.valence(c) + 1) - target_c) + abs(int(mesh.valence(d) + 1) - target_d);

		if (deviation_pre > deviation_post)
		{
			mesh.flip(eh);
		}
	}

	mesh.request_face_normals();
	mesh.request_vertex_normals();
	mesh.request_halfedge_normals();

	mesh.update_normals();

	mesh.release_face_normals();
	mesh.release_vertex_normals();
	mesh.release_halfedge_normals();

	return true;
}

bool Remeshing::tangential_relaxation(Mesh& mesh)
{
	MatrixXd q(mesh.n_vertices(), 3);

	q.setZero();

	MatrixXd p(mesh.n_vertices(), 3);

	p.setZero();

	MatrixXd n(mesh.n_vertices(), 3);

	n.setZero();

	for (int i = 0; i < mesh.n_vertices(); ++i)
	{
		Mesh::VertexHandle vh = mesh.vertex_handle(i);

		int current_valence = mesh.valence(vh);

		Mesh::VertexVertexIter vv_it = mesh.vv_iter(vh);

		for (int j = 0; j < current_valence; ++j)
		{
			q.row(i) += get_Vector3d_from_VH(mesh, vv_it->idx());

			++vv_it;
		}

		q.row(i) /= 1.0 * current_valence;

		p.row(i) = get_Vector3d_from_VH(mesh, i);

		Mesh::Normal normal = mesh.normal(vh);

		for (int j = 0; j < 3; ++j)
		{
			n(i, j) = normal.data()[j];
		}
	}

	for (int i = 0; i < mesh.n_vertices(); ++i)
	{
		Mesh::VertexHandle vh = mesh.vertex_handle(i);

		if (mesh.is_boundary(vh))
		{
			continue;
		}

		p.row(i) = q.row(i) + (n.row(i) * (p.row(i) - q.row(i)).transpose()) * n.row(i);

		for (int j = 0; j < 3; ++j)
		{
			mesh.point(vh)[j] = p(i, j);
		}
	}

	mesh.request_face_normals();
	mesh.request_vertex_normals();
	mesh.request_halfedge_normals();

	mesh.update_normals();

	mesh.release_face_normals();
	mesh.release_vertex_normals();
	mesh.release_halfedge_normals();

	return true;
}

bool Remeshing::projecct_to_surface(Mesh& mesh, Tree& tree)
{
	for (int i = 0; i < mesh.n_vertices(); ++i)
	{
		Mesh::VertexHandle vh = mesh.vertex_handle(i);

		RowVector3d current_v = get_Vector3d_from_VH(mesh, i);

		Point current_p(current_v(0), current_v(1), current_v(2));

		Point closest_point = tree.closest_point(current_p);

		for (int j = 0; j < 3; ++j)
		{
			mesh.point(vh)[j] = closest_point[j];
		}
	}

	return true;
}

bool Remeshing::get_Remeshing_Result(Mesh& mesh, double target_edge_length)
{
	if (target_edge_length < 0)
	{
		return true;
	}

	double low = 4.0 / 5.0 * target_edge_length;
	double high = 4.0 / 3.0 * target_edge_length;

	vector<Point> point_list;

	list<Triangle> triangles;

	for (int i = 0; i < mesh.n_vertices(); ++i)
	{
		RowVector3d current_v = get_Vector3d_from_VH(mesh, i);

		point_list.emplace_back(Point(current_v(0), current_v(1), current_v(2)));
	}

	for (int i = 0; i < mesh.n_faces(); ++i)
	{
		Mesh::FaceVertexIter fv_it = mesh.fv_iter(mesh.face_handle(i));

		int v_idx_1 = fv_it->idx();

		++fv_it;

		int v_idx_2 = fv_it->idx();

		++fv_it;

		int v_idx_3 = fv_it->idx();

		triangles.emplace_back(Triangle(point_list[v_idx_1], point_list[v_idx_2], point_list[v_idx_3]));
	}

	Tree tree(triangles.begin(), triangles.end());

	for (int i = 0; i < 10; ++i)
	{
		cout << "========start episode : " << i << "========" << endl;

		split_long_edges(mesh, high);

		cout << "finish split_long_edges" << endl;

		collapse_short_edges(mesh, low, high);

		cout << "finish collapse_short_edges" << endl;

		equalize_valences(mesh);

		cout << "finish equalize_valences" << endl;

		tangential_relaxation(mesh);

		cout << "finish tangential_relaxation" << endl;

		projecct_to_surface(mesh, tree);

		cout << "finish projecct_to_surface" << endl;
	}

	std::cout << "Finish Get Result !" << std::endl;

	return true;
}

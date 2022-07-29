#include "cross_fields.h"

Cross_Fields::Cross_Fields()
{

}

Cross_Fields::~Cross_Fields()
{

}

double Cross_Fields::get_Dist_2_of_Points(Mesh::Point point_1, Mesh::Point point_2)
{
	double dist_2_of_points = 0;

	dist_2_of_points += (point_1.data()[0] - point_2.data()[0]) * (point_1.data()[0] - point_2.data()[0]);
	dist_2_of_points += (point_1.data()[1] - point_2.data()[1]) * (point_1.data()[1] - point_2.data()[1]);
	dist_2_of_points += (point_1.data()[2] - point_2.data()[2]) * (point_1.data()[2] - point_2.data()[2]);

	return dist_2_of_points;
}

double Cross_Fields::get_Dist_of_Points(Mesh::Point point_1, Mesh::Point point_2)
{
	return sqrt(get_Dist_2_of_Points(point_1, point_2));
}

double Cross_Fields::get_Norm_of_Point(Mesh::Point point)
{
	Mesh::Point zero(0, 0, 0);

	return sqrt(get_Dist_2_of_Points(point, zero));
}

double Cross_Fields::get_Norm_of_Vector(std::vector<double> x)
{
	double dist_to_zero = x[0] * x[0] + x[1] * x[1] + x[2] * x[2];

	return sqrt(dist_to_zero);
}

bool Cross_Fields::is_In_Set(std::vector<int> set, int data)
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

double Cross_Fields::get_Radian_of_Three_Points(Mesh::Point point_1, Mesh::Point point_mid, Mesh::Point point_2)
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

double Cross_Fields::get_Cot_of_Three_Points(Mesh::Point point_1, Mesh::Point point_mid, Mesh::Point point_2)
{
	double tan_of_three_points = tan(get_Radian_of_Three_Points(point_1, point_mid, point_2));

	if (tan_of_three_points != 0)
	{
		return 1.0 / tan_of_three_points;
	}

	std::cout << "tan is zero!" << std::endl;

	return 0;
}

double Cross_Fields::get_Area_of_Three_Points(Mesh::Point point_1, Mesh::Point point_2, Mesh::Point point_3)
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

double Cross_Fields::get_Area_of_Four_Points(Mesh::Point point_1, Mesh::Point point_2, Mesh::Point point_3, Mesh::Point point_4)
{
	return get_Area_of_Three_Points(point_1, point_2, point_3) + get_Area_of_Three_Points(point_1, point_3, point_4);
}

double Cross_Fields::get_Area_of_Face_by_Face_Idx(Mesh& mesh, int fh_idx)
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

std::vector<double> Cross_Fields::get_Norm_Vector_of_Triangle_by_Three_Points(Mesh::Point point_1, Mesh::Point point_2, Mesh::Point point_3)
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

std::vector<double> Cross_Fields::get_Norm_Vector_of_Triangle_by_Face_Idx(Mesh& mesh, int fh_idx)
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

std::vector<double> Cross_Fields::get_Center_of_Triangle_by_Face_Idx(Mesh& mesh, int fh_idx)
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

std::vector<int> Cross_Fields::get_Neighboor_Face_Idx_Set(Mesh& mesh, int fh_idx)
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

std::vector<std::vector<int>> Cross_Fields::get_FVH_Idx_Set(Mesh& mesh)
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

std::vector<std::vector<int>> Cross_Fields::get_VFH_Idx_Set(Mesh& mesh)
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

std::vector<double> Cross_Fields::get_Face_Area_Set(Mesh& mesh)
{
	std::vector<double> face_area_set;

	face_area_set.resize(mesh.n_faces());

	for (int i = 0; i < mesh.n_faces(); ++i)
	{
		face_area_set[i] = get_Area_of_Face_by_Face_Idx(mesh, i);
	}

	return face_area_set;
}

std::vector<double> Cross_Fields::get_Local_Average_Area_Set_of_VH(Mesh& mesh)
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

void Cross_Fields::get_Face_Local_Basis_Set(Mesh& mesh)
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

void Cross_Fields::get_Face_Bary_Center_Set(Mesh& mesh)
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

void Cross_Fields::get_Global_Edge_Scale(Mesh& mesh)
{
	global_edge_scale = 0;

	for (int i = 0; i < mesh.n_edges(); ++i)
	{
		global_edge_scale += mesh.calc_edge_length(mesh.edge_handle(i));
	}

	global_edge_scale /= mesh.n_edges();
}

Matrix<double, 1, 3> Cross_Fields::get_Vector3d_from_VH(Mesh& mesh, int vh_idx)
{
	Matrix<double, 1, 3> v;

	Mesh::Point point = mesh.point(mesh.vertex_handle(vh_idx));

	for (int i = 0; i < 3; ++i)
	{
		v(i) = point.data()[i];
	}

	return v;
}

MatrixXd Cross_Fields::random_constraints(const Matrix<double, 1, 3>& b1, const Matrix<double, 1, 3>& b2, int n)
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

VectorXd Cross_Fields::get_K(Mesh& mesh)
{
	VectorXd K;

	K.setZero(mesh.n_edges());

	for (unsigned eid = 0; eid < mesh.n_edges(); ++eid)
	{
		Mesh::EdgeHandle eh = mesh.edge_handle(eid);

		if (!mesh.is_boundary(eh))
		{
			Mesh::HalfedgeHandle heh = mesh.halfedge_handle(eh, 0);

			int fid0 = mesh.face_handle(heh).idx();
			int fid1 = mesh.face_handle(mesh.opposite_halfedge_handle(heh)).idx();

			int common_vh_idx0 = mesh.from_vertex_handle(heh).idx();
			int common_vh_idx1 = mesh.to_vertex_handle(heh).idx();

			Matrix<double, 1, 3> N0 = face_local_basis_set[fid0][2];
			Matrix<double, 1, 3> N1 = face_local_basis_set[fid1][2];

			Matrix<double, 1, 3> common_v0 = get_Vector3d_from_VH(mesh, common_vh_idx0);
			Matrix<double, 1, 3> common_v1 = get_Vector3d_from_VH(mesh, common_vh_idx1);

			Matrix<double, 1, 3> common_edge = (common_v1 - common_v0);

			common_edge.normalize();

			Matrix<double, 1, 3> o = common_v0;

			// Map the two triangles in a new space where the common edge is the x axis and the N0 the z axis
			Matrix3d P;
			Matrix<double, 1, 3> tmp = -N0.cross(common_edge);
			P << common_edge, tmp, N0;

			Matrix3d V0;
			Mesh::FaceVertexIter fv_it = mesh.fv_iter(mesh.face_handle(fid0));
			for (int j = 0; j < 3; ++j)
			{
				V0.row(j) = get_Vector3d_from_VH(mesh, fv_it->idx()) - o;

				++fv_it;
			}
			V0 = (P * V0.transpose()).transpose();

			int far_away_common_edge_vh_row = -1;

			Matrix3d V1;
			fv_it = mesh.fv_iter(mesh.face_handle(fid1));
			for (int j = 0; j < 3; ++j)
			{
				V0.row(j) = get_Vector3d_from_VH(mesh, fv_it->idx()) - o;

				if (fv_it->idx() != common_vh_idx0 && fv_it->idx() != common_vh_idx1)
				{
					far_away_common_edge_vh_row = j;
				}

				++fv_it;
			}
			V1 = (P * V1.transpose()).transpose();

			// compute rotation R such that R * N1 = N0
			// i.e. map both triangles to the same plane
			double alpha = -atan2(V1(far_away_common_edge_vh_row, 2), V1(far_away_common_edge_vh_row, 1));

			Matrix3d R;
			R << 1, 0, 0,
				0, cos(alpha), -sin(alpha),
				0, sin(alpha), cos(alpha);
			V1 = (R * V1.transpose()).transpose();

			// measure the angle between the reference frames
			// k_ij is the angle between the triangle on the left and the one on the right
			Matrix<double, 1, 3> ref0 = V0.row(1) - V0.row(0);
			Matrix<double, 1, 3> ref1 = V1.row(1) - V1.row(0);

			ref0.normalize();
			ref1.normalize();

			double ktemp = atan2(ref1(1), ref1(0)) - atan2(ref0(1), ref0(0));

			// just to be sure, rotate ref0 using angle ktemp...
			Matrix2d R2;
			R2 << cos(ktemp), -sin(ktemp), sin(ktemp), cos(ktemp);

			Matrix<double, 1, 2> tmp1 = R2 * (ref0.head(2)).transpose();

			//      assert(tmp1(0) - ref1(0) < 1e-10);
			//      assert(tmp1(1) - ref1(1) < 1e-10);

			K[eid] = ktemp;
		}
	}

	return K;
}

void Cross_Fields::doCombs(int offset, int k, int N, std::vector<std::vector<int>>& allCombs)
{
	if (k == 0) {
		allCombs.emplace_back(combinations);
		return;
	}
	for (int i = offset; i <= N - k; ++i) {
		combinations.push_back(i);
		doCombs(i + 1, k - 1, N, allCombs);
		combinations.pop_back();
	}
}

void Cross_Fields::get_General_Coeff_Constraints(Mesh& mesh, const VectorXi& isConstrained, const Matrix<double, Dynamic, Dynamic>& cfW, int k, Matrix<std::complex<double>, Dynamic, 1>& Ck)
{
	int numConstrained = isConstrained.sum();
	Ck.resize(numConstrained, 1);
	int n = cfW.cols() / 3;

	std::vector<std::vector<int>> allCombs;

	combinations.clear();

	allCombs.clear();

	doCombs(0, k + 1, n, allCombs);

	int ind = 0;
	for (int fi = 0; fi < mesh.n_faces(); ++fi)
	{
		const Matrix<double, 1, 3>& b1 = face_local_basis_set[fi][0];
		const Matrix<double, 1, 3>& b2 = face_local_basis_set[fi][1];
		if (isConstrained[fi])
		{
			std::complex<double> ck(0);

			for (int j = 0; j < allCombs.size(); ++j)
			{
				std::complex<double> tk(1.);
				//collect products
				for (int i = 0; i < allCombs[j].size(); ++i)
				{
					int index = allCombs[j][i];

					const Matrix<double, 1, 3>& w = cfW.block(fi, 3 * index, 1, 3);
					double w0 = w.dot(b1);
					double w1 = w.dot(b2);
					std::complex<double> u(w0, w1);
					tk *= u * u;
				}
				//collect sum
				ck += tk;
			}
			Ck(ind) = ck;
			++ind;
		}
	}
}

void Cross_Fields::get_General_Coeff_Constraints(Mesh& mesh, const VectorXi& isConstrained, const Matrix<double, Dynamic, Dynamic>& cfW, int k, const VectorXi& rootsIndex, Matrix<std::complex<double>, Dynamic, 1>& Ck)
{
	int numConstrained = isConstrained.sum();
	Ck.resize(numConstrained, 1);
	int n = rootsIndex.rows();

	std::vector<std::vector<int>> allCombs;

	combinations.clear();

	allCombs.clear();

	doCombs(0, k + 1, n, allCombs);

	int ind = 0;
	for (int fi = 0; fi < mesh.n_faces(); ++fi)
	{
		const Matrix<double, 1, 3>& b1 = face_local_basis_set[fi][0];
		const Matrix<double, 1, 3>& b2 = face_local_basis_set[fi][1];
		if (isConstrained[fi])
		{
			std::complex<double> ck(0);

			for (int j = 0; j < allCombs.size(); ++j)
			{
				std::complex<double> tk(1.);
				//collect products
				for (int i = 0; i < allCombs[j].size(); ++i)
				{
					int index = allCombs[j][i];

					int ri = rootsIndex[index];
					Matrix<double, 1, 3> w;
					if (ri > 0)
						w = cfW.block(fi, 3 * (ri - 1), 1, 3);
					else
						w = -cfW.block(fi, 3 * (-ri - 1), 1, 3);
					double w0 = w.dot(b1);
					double w1 = w.dot(b2);
					std::complex<double> u(w0, w1);
					tk *= u;
				}
				//collect sum
				ck += tk;
			}
			Ck(ind) = ck;
			++ind;
		}
	}
}

void Cross_Fields::get_Coefficient_Laplacian(Mesh& mesh, int n, VectorXd K, SparseMatrix<std::complex<double>>& D)
{
	std::vector<Triplet<std::complex<double>>> tripletList;

	for (int eid = 0; eid < mesh.n_edges(); ++eid)
	{
		Mesh::EdgeHandle eh = mesh.edge_handle(eid);

		if (!mesh.is_boundary(eh))
		{
			Mesh::HalfedgeHandle heh = mesh.halfedge_handle(eh, 0);

			int fid0 = mesh.face_handle(heh).idx();
			int fid1 = mesh.face_handle(mesh.opposite_halfedge_handle(heh)).idx();

			tripletList.emplace_back(Triplet<std::complex<double>>(fid0, fid0, std::complex<double>(1.)));
			tripletList.emplace_back(Triplet<std::complex<double>>(fid1, fid1, std::complex<double>(1.)));
			tripletList.emplace_back(Triplet<std::complex<double>>(fid0, fid1, std::complex<double>(-1. * std::polar(1., -1. * n * K[eid]))));
			tripletList.emplace_back(Triplet<std::complex<double>>(fid1, fid0, std::complex<double>(-1. * std::polar(1., 1. * n * K[eid]))));
		}
	}

	D.resize(mesh.n_faces(), mesh.n_faces());
	D.setFromTriplets(tripletList.begin(), tripletList.end());
}

void Cross_Fields::slice(const SparseMatrix<std::complex<double>>& X, const Matrix<int, Dynamic, 1>& R, const Matrix<int, Dynamic, 1>& C, SparseMatrix<std::complex<double>>& Y)
{
	int xm = X.rows();
	int xn = X.cols();
	int ym = R.size();
	int yn = C.size();

	// special case when R or C is empty
	if (ym == 0 || yn == 0)
	{
		Y.resize(ym, yn);
		return;
	}

	assert(R.minCoeff() >= 0);
	assert(R.maxCoeff() < xm);
	assert(C.minCoeff() >= 0);
	assert(C.maxCoeff() < xn);

	// Build reindexing maps for columns and rows, -1 means not in map
	std::vector<std::vector<int> > RI;
	RI.resize(xm);
	for (int i = 0; i < ym; i++)
	{
		RI[R(i)].push_back(i);
	}
	std::vector<std::vector<int> > CI;
	CI.resize(xn);
	// initialize to -1
	for (int i = 0; i < yn; i++)
	{
		CI[C(i)].push_back(i);
	}
	// Resize output
	DynamicSparseMatrix<std::complex<double>, RowMajor> dyn_Y(ym, yn);
	// Take a guess at the number of nonzeros (this assumes uniform distribution
	// not banded or heavily diagonal)
	dyn_Y.reserve((X.nonZeros() / (X.rows() * X.cols())) * (ym * yn));
	// Iterate over outside
	for (int k = 0; k < X.outerSize(); ++k)
	{
		// Iterate over inside
		for (SparseMatrix<std::complex<double>>::InnerIterator it(X, k); it; ++it)
		{
			std::vector<int>::iterator rit, cit;
			for (rit = RI[it.row()].begin(); rit != RI[it.row()].end(); rit++)
			{
				for (cit = CI[it.col()].begin(); cit != CI[it.col()].end(); cit++)
				{
					dyn_Y.coeffRef(*rit, *cit) = it.value();
				}
			}
		}
	}
	Y = SparseMatrix<std::complex<double>>(dyn_Y);
}

void Cross_Fields::min_Quad_With_Known_Mini(const SparseMatrix<std::complex<double>>& Q, const SparseMatrix<std::complex<double>>& f, const VectorXi isConstrained, const Matrix<std::complex<double>, Dynamic, 1>& xknown, Matrix<std::complex<double>, Dynamic, 1>& x)
{
	int N = Q.rows();

	int nc = xknown.rows();
	VectorXi known; known.setZero(nc, 1);
	VectorXi unknown; unknown.setZero(N - nc, 1);

	int indk = 0, indu = 0;
	for (int i = 0; i < N; ++i)
		if (isConstrained[i])
		{
			known[indk] = i;
			indk++;
		}
		else
		{
			unknown[indu] = i;
			indu++;
		}

	SparseMatrix<std::complex<double>> Quu, Quk;

	slice(Q, unknown, unknown, Quu);
	slice(Q, unknown, known, Quk);


	std::vector<Triplet<std::complex<double>>> tripletList;

	SparseMatrix<std::complex<double>> fu(N - nc, 1);

	slice(f, unknown, VectorXi::Zero(1, 1), fu);

	SparseMatrix<std::complex<double>> rhs = (Quk * xknown).sparseView() + .5 * fu;

	SparseLU<SparseMatrix<std::complex<double>>> solver;
	solver.compute(-Quu);
	if (solver.info() != Success)
	{
		std::cerr << "Decomposition failed!" << std::endl;
		return;
	}
	Eigen::SparseMatrix<std::complex<double>>  b = solver.solve(rhs);
	if (solver.info() != Success)
	{
		std::cerr << "Solving failed!" << std::endl;
		return;
	}

	indk = 0, indu = 0;
	x.setZero(N, 1);
	for (int i = 0; i < N; ++i)
		if (isConstrained[i])
			x[i] = xknown[indk++];
		else
			x[i] = b.coeff(indu++, 0);
}

void Cross_Fields::set_Field_From_General_Coefficients(Mesh& mesh, int num, const std::vector<Matrix<std::complex<double>, Dynamic, 1>>& coeffs, std::vector<Matrix<double, Eigen::Dynamic, 2>>& pv)
{
	pv.assign(num, Matrix<double, Dynamic, 2>::Zero(mesh.n_faces(), 2));
	
	for (int i = 0; i < mesh.n_faces(); ++i)
	{
		//    poly coefficients: 1, 0, -Acoeff, 0, Bcoeff
		//    matlab code from roots (given there are no trailing zeros in the polynomial coefficients)
		Eigen::Matrix<std::complex<double>, Eigen::Dynamic, 1> polyCoeff;
		polyCoeff.setZero(2 * num + 1, 1);
		polyCoeff[0] = 1.;
		int sign = 1;
		for (int k = 0; k < num; ++k)
		{
			sign = -sign;
			int degree = 2 * (k + 1);
			polyCoeff[degree] = (1. * sign) * coeffs[k](i);
		}

		Matrix<std::complex<double>, Dynamic, 1> roots;

		int n = polyCoeff.rows() - 1;

		Matrix<std::complex<double>, Dynamic, 1> d(n, 1);
		d = polyCoeff.tail(n) / polyCoeff(0);

		Matrix<std::complex<double>, Dynamic, Dynamic> I;
		I.setIdentity(n - 1, n - 1);

		Matrix<std::complex<double>, Dynamic, 1> z;
		z.setZero(n - 1, 1);

		Matrix<std::complex<double>, Dynamic, Dynamic> a(n, n);
		a << -d.transpose(), I, z;
		roots = a.eigenvalues();

		VectorXi done;
		done.setZero(2 * num, 1);

		Matrix<std::complex<double>, Dynamic, 1> u(num, 1);
		int ind = 0;
		for (int k = 0; k < 2 * num; ++k)
		{
			if (done[k])
				continue;
			u[ind] = roots[k];
			done[k] = 1;

			int mini = -1;
			double mind = 1e10;
			for (int l = k + 1; l < 2 * num; ++l)
			{
				double dist = abs(roots[l] + u[ind]);
				if (dist < mind)
				{
					mind = dist;
					mini = l;
				}
			}
			done[mini] = 1;
			ind++;
		}
		for (int k = 0; k < num; ++k)
		{
			pv[k](i, 0) = real(u[k]);
			pv[k](i, 1) = imag(u[k]);
		}
	}
}

bool Cross_Fields::get_Cross_Fields_Result(Mesh& mesh, MatrixXd& output, int num)
{
	get_Face_Local_Basis_Set(mesh);

	get_Face_Bary_Center_Set(mesh);

	get_Global_Edge_Scale(mesh);

	global_edge_scale /= 16.0;

	MatrixXd rotate_angle_set(1, mesh.n_faces());

	for (int i = 0; i < mesh.n_faces(); ++i)
	{
		double z1 = face_local_basis_set[i][0](2);
		double z2 = face_local_basis_set[i][1](2);

		if (z2 != 0)
		{
			rotate_angle_set(i) = atan(-z1 / z2);
		}
		else if (z1 != 0)
		{
			rotate_angle_set(i) = M_PI_2;
		}
		else
		{
			rotate_angle_set(i) = 0;
		}
	}

	output.setZero(mesh.n_faces(), 3 * num);

	for (int i = 0; i < mesh.n_faces(); ++i)
	{
		if (num > 0)
		{
			output.block(i, 0, 1, 3) = (face_local_basis_set[i][0] * cos(rotate_angle_set(i)) + face_local_basis_set[i][1] * sin(rotate_angle_set(i))).normalized() * global_edge_scale;
		}
		if (num > 1)
		{
			output.block(i, 3, 1, 3) = (face_local_basis_set[i][0] * cos(rotate_angle_set(i) + M_PI_2) + face_local_basis_set[i][1] * sin(rotate_angle_set(i) + M_PI_2)).normalized() * global_edge_scale;
		}
	}

	return true;

	VectorXi b(4);
	b << 4550, 2321, 5413, 5350;

	for (int i = 0; i < b.size(); ++i)
	{
		b(i) = b(i) % mesh.n_faces();
	}

	MatrixXd bc(b.size(), num * 3);
	for (unsigned i = 0; i < b.size(); ++i)
	{
		MatrixXd t = random_constraints(face_local_basis_set[b(i)][0], face_local_basis_set[b(i)][1], num);
		bc.row(i) = t;
	}

	VectorXi isConstrained = VectorXi::Constant(mesh.n_faces(), 0);

	MatrixXd cfW = MatrixXd::Constant(mesh.n_faces(), bc.cols(), 0);

	for (unsigned i = 0; i < b.size(); ++i)
	{
		isConstrained(b(i)) = 1;
		cfW.row(b(i)) << bc.row(i);
	}

	VectorXd K = get_K(mesh);

	std::vector<Matrix<std::complex<double>, Dynamic, 1>> coeffs(num, Matrix<std::complex<double>, Dynamic, 1>::Zero(mesh.n_faces(), 1));

	for (int i = 0; i < num; ++i)
	{
		int degree = 2 * (i + 1);

		Matrix<std::complex<double>, Dynamic, 1> Ck;

		get_General_Coeff_Constraints(mesh, isConstrained, cfW, i, Ck);

		SparseMatrix<std::complex<double>> DD;

		get_Coefficient_Laplacian(mesh, degree, K, DD);

		SparseMatrix<std::complex<double>> f;

		f.resize(mesh.n_faces(), 1);

		min_Quad_With_Known_Mini(DD, f, isConstrained, Ck, coeffs[i]);
	}

	std::vector<Matrix<double, Eigen::Dynamic, 2>> pv;

	set_Field_From_General_Coefficients(mesh, num, coeffs, pv);

	output.setZero(mesh.n_faces(), 3 * num);
	for (int fi = 0; fi < mesh.n_faces(); ++fi)
	{
		const Matrix<double, 1, 3>& b1 = face_local_basis_set[fi][0];
		const Matrix<double, 1, 3>& b2 = face_local_basis_set[fi][1];
		for (int i = 0; i < num; ++i)
		{
			output.block(fi, 3 * i, 1, 3) = (pv[i](fi, 0) * b1 + pv[i](fi, 1) * b2).normalized() * global_edge_scale;
		}
	}

	for (int i = 0; i < b.size(); ++i)
	{
		for (int j = 0; j < num; ++j)
		{
			output.block(b[i], num * 3, 1, 3) = bc.block(i, num * 3, 1, 3).normalized();
		}
	}

	std::cout << "Finish Get Result !" << std::endl;

	return true;
}

bool Cross_Fields::get_General_Cross_Fields_Result(Mesh& mesh, MatrixXd& output, int num)
{
	get_Face_Local_Basis_Set(mesh);

	get_Face_Bary_Center_Set(mesh);

	get_Global_Edge_Scale(mesh);

	global_edge_scale /= 4.0;

	MatrixXd rotate_angle_set(1, mesh.n_faces());

	for (int i = 0; i < mesh.n_faces(); ++i)
	{
		double z1 = face_local_basis_set[i][0](2);
		double z2 = face_local_basis_set[i][1](2);

		if (z2 != 0)
		{
			rotate_angle_set(i) = atan(-z1 / z2);
		}
		else if(z1 != 0)
		{
			rotate_angle_set(i) = M_PI_2;
		}
		else
		{
			rotate_angle_set(i) = 0;
		}
	}

	output.setZero(mesh.n_faces(), 3 * num);

	for (int i = 0; i < mesh.n_faces(); ++i)
	{
		for (int j = 0; j < num; ++j)
		{
			for (int k = 0; k < 3; ++k)
			{
				output(i, 3 * j + k) = face_local_basis_set[i][j](k) * cos(rotate_angle_set(i)) + face_local_basis_set[i][j](k) * sin(rotate_angle_set(i));
			}
		}
	}

	return true;

	VectorXi b(3);
	b << 1511, 603, 506;

	for (int i = 0; i < b.size(); ++i)
	{
		b(i) = b(i) % mesh.n_faces();
	}

	int numConstraintsToGenerate;
	if (num >= 5)
		numConstraintsToGenerate = num - 2;
	else
		if (num >= 3)
			numConstraintsToGenerate = num - 1;
		else
			numConstraintsToGenerate = num;


	MatrixXd bc(b.size(), numConstraintsToGenerate * 3);
	for (unsigned i = 0; i < b.size(); ++i)
	{
		MatrixXd t = random_constraints(face_local_basis_set[b(i)][0], face_local_basis_set[b(i)][1], num);
		bc.row(i) = t;
	}

	VectorXi rootsIndex(num);
	for (int i = 0; i < numConstraintsToGenerate; ++i)
		rootsIndex[i] = i + 1;
	if (num >= 5)
		rootsIndex[num - 2] = -2;
	if (num >= 3)
		rootsIndex[num - 1] = -1;

	VectorXi isConstrained = VectorXi::Constant(mesh.n_faces(), 0);

	MatrixXd cfW = MatrixXd::Constant(mesh.n_faces(), bc.cols(), 0);

	for (unsigned i = 0; i < b.size(); ++i)
	{
		isConstrained(b(i)) = 1;
		cfW.row(b(i)) << bc.row(i);
	}

	VectorXd K = get_K(mesh);

	std::vector<Matrix<std::complex<double>, Dynamic, 1>> coeffs(num, Matrix<std::complex<double>, Dynamic, 1>::Zero(mesh.n_faces(), 1));

	for (int i = 0; i < num; ++i)
	{
		int degree = i + 1;

		Matrix<std::complex<double>, Dynamic, 1> Ck;

		get_General_Coeff_Constraints(mesh, isConstrained, cfW, i, rootsIndex, Ck);

		SparseMatrix<std::complex<double>> DD;

		get_Coefficient_Laplacian(mesh, degree, K, DD);

		SparseMatrix<std::complex<double>> f;

		f.resize(mesh.n_faces(), 1);

		min_Quad_With_Known_Mini(DD, f, isConstrained, Ck, coeffs[i]);
	}

	std::vector<Matrix<double, Eigen::Dynamic, 2>> pv;

	// need to change degree to k+1
	set_Field_From_General_Coefficients(mesh, num, coeffs, pv);

	output.setZero(mesh.n_faces(), 3 * num);
	for (int fi = 0; fi < mesh.n_faces(); ++fi)
	{
		const Matrix<double, 1, 3>& b1 = face_local_basis_set[fi][0];
		const Matrix<double, 1, 3>& b2 = face_local_basis_set[fi][1];
		for (int i = 0; i < num; ++i)
		{
			output.block(fi, 3 * i, 1, 3) = (pv[i](fi, 0) * b1 + pv[i](fi, 1) * b2).normalized() * global_edge_scale;
		}
	}

	std::cout << "Finish Get Result !" << std::endl;

	return true;
}
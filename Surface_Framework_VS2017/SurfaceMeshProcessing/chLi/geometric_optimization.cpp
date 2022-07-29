#include "geometric_optimization.h"

Geometric_Optimization::Geometric_Optimization()
{

}

Geometric_Optimization::~Geometric_Optimization()
{

}

double Geometric_Optimization::get_Dist_2_of_Points(Mesh::Point point_1, Mesh::Point point_2)
{
	double dist_2_of_points = 0;

	dist_2_of_points += (point_1.data()[0] - point_2.data()[0]) * (point_1.data()[0] - point_2.data()[0]);
	dist_2_of_points += (point_1.data()[1] - point_2.data()[1]) * (point_1.data()[1] - point_2.data()[1]);
	dist_2_of_points += (point_1.data()[2] - point_2.data()[2]) * (point_1.data()[2] - point_2.data()[2]);

	return dist_2_of_points;
}

double Geometric_Optimization::get_Dist_of_Points(Mesh::Point point_1, Mesh::Point point_2)
{
	return sqrt(get_Dist_2_of_Points(point_1, point_2));
}

double Geometric_Optimization::get_Dist_2_of_Vector(VectorXd& vector_1, VectorXd& vector_2)
{
	double dist_2 = 0;

	for (int i = 0; i < vector_1.size(); ++i)
	{
		dist_2 += (vector_1(i) - vector_2(i)) * (vector_1(i) - vector_2(i));
	}

	return dist_2;
}

double Geometric_Optimization::get_Dist_2_of_Vector(RowVectorXd& vector_1, RowVectorXd& vector_2)
{
	double dist_2 = 0;

	for (int i = 0; i < vector_1.size(); ++i)
	{
		dist_2 += (vector_1(i) - vector_2(i)) * (vector_1(i) - vector_2(i));
	}

	return dist_2;
}

double Geometric_Optimization::get_Dist_of_Vector(VectorXd& vector_1, VectorXd& vector_2)
{
	return sqrt(get_Dist_2_of_Vector(vector_1, vector_2));
}

double Geometric_Optimization::get_Dist_of_Vector(RowVectorXd& vector_1, RowVectorXd& vector_2)
{
	return sqrt(get_Dist_2_of_Vector(vector_1, vector_2));
}

double Geometric_Optimization::get_Norm_of_Point(Mesh::Point point)
{
	Mesh::Point zero(0, 0, 0);

	return sqrt(get_Dist_2_of_Points(point, zero));
}

double Geometric_Optimization::get_Norm_of_Vector(std::vector<double> x)
{
	double dist_to_zero = x[0] * x[0] + x[1] * x[1] + x[2] * x[2];

	return sqrt(dist_to_zero);
}

bool Geometric_Optimization::is_In_Set(std::vector<int> set, int data)
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

double Geometric_Optimization::get_Radian_of_Three_Points(Mesh::Point point_1, Mesh::Point point_mid, Mesh::Point point_2)
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

double Geometric_Optimization::get_Cot_of_Three_Points(Mesh::Point point_1, Mesh::Point point_mid, Mesh::Point point_2)
{
	double tan_of_three_points = tan(get_Radian_of_Three_Points(point_1, point_mid, point_2));

	if (tan_of_three_points != 0)
	{
		return 1.0 / tan_of_three_points;
	}

	std::cout << "tan is zero!" << std::endl;

	return 0;
}

double Geometric_Optimization::get_Area_of_Three_Points(Mesh::Point point_1, Mesh::Point point_2, Mesh::Point point_3)
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

double Geometric_Optimization::get_Area_of_Four_Points(Mesh::Point point_1, Mesh::Point point_2, Mesh::Point point_3, Mesh::Point point_4)
{
	return get_Area_of_Three_Points(point_1, point_2, point_3) + get_Area_of_Three_Points(point_1, point_3, point_4);
}

double Geometric_Optimization::get_Area_of_Face_by_Face_Idx(Mesh& mesh, int fh_idx)
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

std::vector<double> Geometric_Optimization::get_Norm_Vector_of_Triangle_by_Three_Points(Mesh::Point point_1, Mesh::Point point_2, Mesh::Point point_3)
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

std::vector<double> Geometric_Optimization::get_Norm_Vector_of_Triangle_by_Face_Idx(Mesh& mesh, int fh_idx)
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

std::vector<double> Geometric_Optimization::get_Center_of_Triangle_by_Face_Idx(Mesh& mesh, int fh_idx)
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

std::vector<int> Geometric_Optimization::get_Neighboor_Face_Idx_Set(Mesh& mesh, int fh_idx)
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

std::vector<std::vector<double>> Geometric_Optimization::get_Parameterization_2D(Mesh& mesh)
{
	std::vector<int> boundary_vh_idx_set;

	for (int i = 0; i < mesh.n_vertices(); ++i)
	{
		if (mesh.is_boundary(mesh.vertex_handle(i)))
		{
			boundary_vh_idx_set.emplace_back(i);

			break;
		}
	}

	Mesh::VertexVertexIter vv_it;

	while (true)
	{
		int current_vh_idx = boundary_vh_idx_set[boundary_vh_idx_set.size() - 1];

		bool finished = false;

		std::vector<int> current_neighboor_boundary_vh_idx_set;

		int first_neighboor_vh_idx = mesh.vv_iter(mesh.vertex_handle(current_vh_idx))->idx();

		for (vv_it = mesh.vv_iter(mesh.vertex_handle(current_vh_idx)); vv_it->is_valid(); ++vv_it)
		{
			if (vv_it->idx() == first_neighboor_vh_idx)
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

			if (mesh.is_boundary(mesh.vertex_handle(vv_it->idx())))
			{
				current_neighboor_boundary_vh_idx_set.emplace_back(vv_it->idx());
			}

			if (current_neighboor_boundary_vh_idx_set.size() == 2)
			{
				break;
			}
		}

		if (boundary_vh_idx_set.size() == 1)
		{
			boundary_vh_idx_set.emplace_back(current_neighboor_boundary_vh_idx_set[0]);
		}
		else
		{
			if (current_neighboor_boundary_vh_idx_set[0] == boundary_vh_idx_set[boundary_vh_idx_set.size() - 2])
			{
				if (current_neighboor_boundary_vh_idx_set[1] == boundary_vh_idx_set[0])
				{
					break;
				}

				boundary_vh_idx_set.emplace_back(current_neighboor_boundary_vh_idx_set[1]);
			}
			else
			{
				if (current_neighboor_boundary_vh_idx_set[0] == boundary_vh_idx_set[0])
				{
					break;
				}

				boundary_vh_idx_set.emplace_back(current_neighboor_boundary_vh_idx_set[0]);
			}
		}
	}

	std::vector<std::vector<double>> param_result;

	param_result.resize(mesh.n_vertices());

	for (int i = 0; i < param_result.size(); ++i)
	{
		param_result[i].resize(3);
	}

	//SparseMatrix<double> A(mesh.n_vertices(), mesh.n_vertices());
	MatrixXd A(mesh.n_vertices(), mesh.n_vertices());

	A.setZero();

	MatrixXd b(mesh.n_vertices(), 3);
	MatrixXd x(mesh.n_vertices(), 3);

	b.setZero();

	//std::vector<Triplet<double>> triplet_list;

	for (int i = 0; i < mesh.n_vertices(); ++i)
	{
		Mesh::VertexHandle vh = mesh.vertex_handle(i);

		Mesh::Point vh_point = mesh.point(vh);

		if (mesh.is_boundary(vh))
		{
			//triplet_list.emplace_back(Triplet<double>(i, i, 1));
			A(i, i) = 1.0;

			/*for (int j = 0; j < 3; ++j)
			{
				b(i, j) = vh_point.data()[j];
			}*/

			for (int k = 0; k < boundary_vh_idx_set.size(); ++k)
			{
				if (i == boundary_vh_idx_set[k])
				{
					b(i, 0) = cos(2.0 * M_PI * k / boundary_vh_idx_set.size());
					b(i, 1) = sin(2.0 * M_PI * k / boundary_vh_idx_set.size());
					b(i, 2) = 0.0;
				}
			}
		}
		else
		{
			//triplet_list.emplace_back(Triplet<double>(i, i, -1));
			A(i, i) = -1.0;

			std::vector<int> vh_neighboor_vh_idx_set;

			bool finished = false;

			for (Mesh::VertexVertexIter vv_it = mesh.vv_iter(vh); vv_it->is_valid(); ++vv_it)
			{
				if (vv_it->idx() == mesh.vv_iter(vh)->idx())
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

				vh_neighboor_vh_idx_set.emplace_back(vv_it->idx());
			}

			std::vector<double> weights_list;
			double weights_sum = 0;

			for (int j = 0; j < vh_neighboor_vh_idx_set.size(); ++j)
			{
				Mesh::Point last_point = mesh.point(mesh.vertex_handle(vh_neighboor_vh_idx_set[(j - 1 + vh_neighboor_vh_idx_set.size()) % vh_neighboor_vh_idx_set.size()]));
				Mesh::Point this_point = mesh.point(mesh.vertex_handle(vh_neighboor_vh_idx_set[j]));
				Mesh::Point next_point = mesh.point(mesh.vertex_handle(vh_neighboor_vh_idx_set[(j + 1) % vh_neighboor_vh_idx_set.size()]));

				double radian_1 = get_Radian_of_Three_Points(last_point, vh_point, this_point) / 2.0;
				double radian_2 = get_Radian_of_Three_Points(this_point, vh_point, next_point) / 2.0;

				weights_list.emplace_back((tan(radian_1) + tan(radian_2)) / get_Norm_of_Point(this_point - vh_point));

				weights_sum += weights_list[j];
			}

			for (int j = 0; j < vh_neighboor_vh_idx_set.size(); ++j)
			{
				//triplet_list.emplace_back(Triplet<double>(i, vh_neighboor_vh_idx_set[j], weights_list[j] / weights_sum));
				A(i, vh_neighboor_vh_idx_set[j]) = weights_list[j] / weights_sum;
			}
		}
	}

	//A.setFromTriplets(triplet_list.begin(), triplet_list.end());
	//A.makeCompressed();

	//SimplicialCholesky<SparseMatrix<double>>* Solver = new SimplicialCholesky<SparseMatrix<double>>(A);
	FullPivHouseholderQR<MatrixXd>* Solver = new FullPivHouseholderQR<MatrixXd>(A);

	//MatrixXd V = Solver->matrixV(), U = Solver->matrixU();
	//MatrixXd S = U.inverse() * A * V.transpose().inverse();

	x = Solver->solve(b);

	for (int i = 0; i < mesh.n_vertices(); ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			param_result[i][j] = x(i, j);
		}
	}

	return param_result;
}

std::vector<std::vector<std::vector<double>>> Geometric_Optimization::get_Source_Triangle_Position_2D(Mesh& mesh)
{
	std::vector<std::vector<std::vector<double>>> source_triangle_position;

	source_triangle_position.resize(mesh.n_faces());

	for (int i = 0; i < mesh.n_faces(); ++i)
	{
		Mesh::FaceHandle fh = mesh.face_handle(i);

		std::vector<int> fvh_idx_set;

		for (auto fvh : mesh.fv_range(fh))
		{
			fvh_idx_set.emplace_back(fvh.idx());
		}

		Mesh::Point point1 = mesh.point(mesh.vertex_handle(fvh_idx_set[0]));
		Mesh::Point point2 = mesh.point(mesh.vertex_handle(fvh_idx_set[1]));
		Mesh::Point point3 = mesh.point(mesh.vertex_handle(fvh_idx_set[2]));

		source_triangle_position[i].resize(3);

		source_triangle_position[i][0].emplace_back(0);
		source_triangle_position[i][0].emplace_back(0);

		source_triangle_position[i][1].emplace_back(get_Norm_of_Point(point2 - point1));
		source_triangle_position[i][1].emplace_back(0);

		double angle = get_Radian_of_Three_Points(point2, point1, point3);

		double norm = get_Norm_of_Point(point3 - point1);

		source_triangle_position[i][2].emplace_back(norm * cos(angle));
		source_triangle_position[i][2].emplace_back(norm * sin(angle));
	}

	return source_triangle_position;
}

std::vector<std::vector<std::vector<double>>> Geometric_Optimization::get_Face_Transformation_Matrix(Mesh& mesh, std::vector<std::vector<double>> param_2d, std::vector<std::vector<std::vector<double>>> source_triangle_position_2d)
{
	std::vector<std::vector<std::vector<double>>> face_transformation_matrix;

	face_transformation_matrix.resize(mesh.n_faces());

	for (int i = 0; i < mesh.n_faces(); ++i)
	{
		face_transformation_matrix[i].resize(2);

		face_transformation_matrix[i][0].resize(2);
		face_transformation_matrix[i][1].resize(2);

		MatrixXd A(3, 3);
		MatrixXd x(3, 3), b(3, 3);

		Mesh::FaceHandle fh = mesh.face_handle(i);

		std::vector<int> fvh_idx_set;

		for (auto fvh : mesh.fv_range(fh))
		{
			fvh_idx_set.emplace_back(fvh.idx());
		}

		for (int j = 0; j < 3; ++j)
		{
			A(0, j) = source_triangle_position_2d[i][j][0];
			A(1, j) = source_triangle_position_2d[i][j][1];
			A(2, j) = 1.0;

			b(0, j) = param_2d[fvh_idx_set[j]][0];
			b(1, j) = param_2d[fvh_idx_set[j]][1];
			b(2, j) = 1.0;
		}

		x = b * A.inverse();

		for (int j = 0; j < 2; ++j)
		{
			for (int k = 0; k < 2; ++k)
			{
				face_transformation_matrix[i][j][k] = x(j, k);
			}
		}
	}

	return face_transformation_matrix;
}

std::vector<std::vector<std::vector<double>>> Geometric_Optimization::get_Face_Target_Transformation_Matrix(std::vector<std::vector<std::vector<double>>> face_transformation_matrix)
{
	std::vector<std::vector<std::vector<double>>> face_target_transformation_matrix;

	face_target_transformation_matrix.resize(face_transformation_matrix.size());

	for (int i = 0; i < face_transformation_matrix.size(); ++i)
	{
		face_target_transformation_matrix[i].resize(2);

		face_target_transformation_matrix[i][0].resize(2);
		face_target_transformation_matrix[i][1].resize(2);

		MatrixXd A(2, 2);

		for (int j = 0; j < 2; ++j)
		{
			for (int k = 0; k < 2; ++k)
			{
				A(j, k) = face_transformation_matrix[i][j][k];
			}
		}

		JacobiSVD<MatrixXd> svd(A, ComputeThinU | ComputeThinV);

		MatrixXd U = svd.matrixU();
		MatrixXd V = svd.matrixV();
		VectorXd Sigma = svd.singularValues();

		MatrixXd similar_matrix = U * V.transpose();

		for (int j = 0; j < 2; ++j)
		{
			for (int k = 0; k < 2; ++k)
			{
				face_target_transformation_matrix[i][j][k] = similar_matrix(j, k);
			}
		}
	}

	return face_target_transformation_matrix;
}

vector<MatrixXd> Geometric_Optimization::get_Face_Target_USwUT_Matrix(std::vector<std::vector<std::vector<double>>> face_transformation_matrix)
{
	vector<MatrixXd> face_target_sw_matrix;

	face_target_sw_matrix.resize(face_transformation_matrix.size());

	for (int i = 0; i < face_transformation_matrix.size(); ++i)
	{
		MatrixXd A(2, 2);

		for (int j = 0; j < 2; ++j)
		{
			for (int k = 0; k < 2; ++k)
			{
				A(j, k) = face_transformation_matrix[i][j][k];
			}
		}

		JacobiSVD<MatrixXd> svd(A, ComputeFullU | ComputeFullV);

		MatrixXd U = svd.matrixU();
		MatrixXd V = svd.matrixV();
		VectorXd Sigma = svd.singularValues();
		MatrixXd Sigma_Matrix(2, 2);
		Sigma_Matrix.setZero();

		for (int j = 0; j < 2; ++j)
		{
			if (Sigma(j) == 0)
			{
				Sigma_Matrix(j, j) = 0;
			}
			else if (Sigma(j) != 1.0)
			{
				Sigma_Matrix(j, j) = sqrt((Sigma(j) - 1.0 / Sigma(j) / Sigma(j) / Sigma(j)) / (Sigma(j) - 1.0));
			}
			else
			{
				Sigma_Matrix(j, j) = 2.0;
			}
		}

		face_target_sw_matrix[i] = U * Sigma_Matrix * U.transpose();
	}

	return face_target_sw_matrix;
}

vector<MatrixXd> Geometric_Optimization::get_Source_Triangle_Position_Matrix_Inverse(std::vector<std::vector<std::vector<double>>> source_triangle_position_2d)
{
	vector<MatrixXd> source_triangle_position_matrix_inverse;

	source_triangle_position_matrix_inverse.resize(source_triangle_position_2d.size());

	for (int i = 0; i < source_triangle_position_2d.size(); ++i)
	{
		MatrixXd A(3, 3);

		for (int j = 0; j < 3; ++j)
		{
			A(j, 0) = source_triangle_position_2d[i][j][0];
			A(j, 1) = source_triangle_position_2d[i][j][1];
			A(j, 2) = 1.0;
		}

		source_triangle_position_matrix_inverse[i] = A.inverse();
	}

	return source_triangle_position_matrix_inverse;
}

std::vector<std::vector<int>> Geometric_Optimization::get_FVH_Idx_Set(Mesh& mesh)
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

std::vector<std::vector<int>> Geometric_Optimization::get_VFH_Idx_Set(Mesh& mesh)
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

std::vector<double> Geometric_Optimization::get_Face_Area_Set(Mesh& mesh)
{
	std::vector<double> face_area_set;

	face_area_set.resize(mesh.n_faces());

	for (int i = 0; i < mesh.n_faces(); ++i)
	{
		face_area_set[i] = get_Area_of_Face_by_Face_Idx(mesh, i);
	}

	return face_area_set;
}

std::vector<double> Geometric_Optimization::get_Local_Average_Area_Set_of_VH(Mesh& mesh)
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

Matrix<double, 1, 3> Geometric_Optimization::get_Vector3d_from_VH(Mesh& mesh, int vh_idx)
{
	Matrix<double, 1, 3> v;

	Mesh::Point point = mesh.point(mesh.vertex_handle(vh_idx));

	for (int i = 0; i < 3; ++i)
	{
		v(i) = point.data()[i];
	}

	return v;
}

MatrixXd Geometric_Optimization::get_Circum_Circle_Center(Mesh& mesh)
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

bool Geometric_Optimization::update_Delaunay_Triangulation(Mesh& mesh)
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

bool Geometric_Optimization::update_Vertex_Position_By_Circum_Center(Mesh& mesh)
{
	vector<vector<int>> vfh_idx_set = get_VFH_Idx_Set(mesh);

	MatrixXd circum_circle_center = get_Circum_Circle_Center(mesh);

	for (int i = 0; i < mesh.n_vertices(); ++i)
	{
		Mesh::VertexHandle vh = mesh.vertex_handle(i);

		if (mesh.is_boundary(vh))
		{
			continue;
		}

		RowVector2d target_vh_position;

		target_vh_position.setZero();

		for (int j = 0; j < vfh_idx_set[i].size(); ++j)
		{
			target_vh_position += circum_circle_center.block(vfh_idx_set[i][j], 0, 1, 2);
		}

		target_vh_position /= vfh_idx_set[i].size();

		for (int j = 0; j < 2; ++j)
		{
			mesh.point(vh)[j] = target_vh_position(j);
		}
	}

	return true;
}

std::vector<std::vector<double>> Geometric_Optimization::Update_VH_Position(Mesh& mesh, int iteration_num)
{
	//每个三角形平移旋转到原点处
	std::vector<std::vector<std::vector<double>>> source_triangle_position_2d = get_Source_Triangle_Position_2D(mesh);

	//该矩阵 * (ui, vi, 1)[3x3] 可得到 (J)T
	vector<MatrixXd> source_triangle_position_matrix_inverse = get_Source_Triangle_Position_Matrix_Inverse(source_triangle_position_2d);

	std::vector<std::vector<int>> fvh_idx_set = get_FVH_Idx_Set(mesh);

	std::vector<std::vector<int>> vfh_idx_set = get_VFH_Idx_Set(mesh);

	//J
	std::vector<std::vector<std::vector<double>>> face_transformation_matrix;

	//R
	std::vector<std::vector<std::vector<double>>> face_target_transformation_matrix;

	//USwUT
	vector<MatrixXd> face_target_uswut_matrix;

	//(ui, vi)
	std::vector<std::vector<double>> param_2d = get_Parameterization_2D(mesh);

	for (int kk = 0; kk < iteration_num; ++kk)
	{
		face_transformation_matrix = get_Face_Transformation_Matrix(mesh, param_2d, source_triangle_position_2d);

		face_target_transformation_matrix = get_Face_Target_Transformation_Matrix(face_transformation_matrix);

		face_target_uswut_matrix = get_Face_Target_USwUT_Matrix(face_transformation_matrix);

		MatrixXd A(2 * mesh.n_vertices(), 2 * mesh.n_vertices());

		MatrixXd x(2 * mesh.n_vertices(), 1), b(2 * mesh.n_vertices(), 1);

		A.setZero();

		b.setZero();

		for (int i = 0; i < mesh.n_vertices(); ++i)
		{
			for (int j = 0; j < vfh_idx_set[i].size(); ++j)
			{
				int fh_idx = vfh_idx_set[i][j];

				int row_idx_in_face = -1;

				if (fvh_idx_set[fh_idx][0] == i)
				{
					row_idx_in_face = 0;
				}
				else if (fvh_idx_set[fh_idx][1] == i)
				{
					row_idx_in_face = 1;
				}
				else if (fvh_idx_set[fh_idx][2] == i)
				{
					row_idx_in_face = 2;
				}

				MatrixXd current_coeff_x(2, 2);
				MatrixXd current_coeff_y(2, 2);
				current_coeff_x.setZero();
				current_coeff_y.setZero();

				for (int k = 0; k < 2; ++k)
				{
					for (int l = 0; l < 2; ++l)
					{
						current_coeff_x(k, l) += face_target_uswut_matrix[fh_idx](k, 0) * source_triangle_position_matrix_inverse[fh_idx](l, row_idx_in_face);

						current_coeff_y(k, l) += face_target_uswut_matrix[fh_idx](k, 1) * source_triangle_position_matrix_inverse[fh_idx](l, row_idx_in_face);
					}
				}

				for (int k = 0; k < 2; ++k)
				{
					for (int l = 0; l < 2; ++l)
					{
						for (int m = 0; m < 3; ++m)
						{
							A(i, fvh_idx_set[fh_idx][m]) += current_coeff_x(k, l) * face_target_uswut_matrix[fh_idx](k, 0) * source_triangle_position_matrix_inverse[fh_idx](l, m);
							A(i, fvh_idx_set[fh_idx][m] + mesh.n_vertices()) += current_coeff_x(k, l) * face_target_uswut_matrix[fh_idx](k, 1) * source_triangle_position_matrix_inverse[fh_idx](l, m);

							A(i + mesh.n_vertices(), fvh_idx_set[fh_idx][m]) += current_coeff_y(k, l) * face_target_uswut_matrix[fh_idx](k, 0) * source_triangle_position_matrix_inverse[fh_idx](l, m);
							A(i + mesh.n_vertices(), fvh_idx_set[fh_idx][m] + mesh.n_vertices()) += current_coeff_y(k, l) * face_target_uswut_matrix[fh_idx](k, 1) * source_triangle_position_matrix_inverse[fh_idx](l, m);
						}

						b(i, 0) += current_coeff_x(k, l) * face_target_uswut_matrix[fh_idx](k, 0) * face_target_transformation_matrix[fh_idx][0][l];
						b(i, 0) += current_coeff_x(k, l) * face_target_uswut_matrix[fh_idx](k, 1) * face_target_transformation_matrix[fh_idx][1][l];

						b(i + mesh.n_vertices(), 0) += current_coeff_y(k, l) * face_target_uswut_matrix[fh_idx](k, 0) * face_target_transformation_matrix[fh_idx][0][l];
						b(i + mesh.n_vertices(), 0) += current_coeff_y(k, l) * face_target_uswut_matrix[fh_idx](k, 1) * face_target_transformation_matrix[fh_idx][1][l];
					}
				}
			}
		}

		FullPivHouseholderQR<MatrixXd>* Solver = new FullPivHouseholderQR<MatrixXd>(A);

		x = Solver->solve(b);

		for (int i = 0; i < mesh.n_vertices(); ++i)
		{
			param_2d[i][0] = x(i, 0);
			param_2d[i][1] = x(i + mesh.n_vertices(), 0);
		}

		cout << kk + 1 << " / " << iteration_num << endl;
	}

	return param_2d;
}

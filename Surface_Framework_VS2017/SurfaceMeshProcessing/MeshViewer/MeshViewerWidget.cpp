#include <QtCore>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include "MeshViewerWidget.h"

MeshViewerWidget::MeshViewerWidget(QWidget* parent)
	: QGLViewerWidget(parent),
	ptMin(0.0),
	ptMax(0.0),
	isEnableLighting(true),
	isTwoSideLighting(false),
	isDrawBoundingBox(false),
	isDrawBoundary(false)
{
	shortest_path_solver = new Shortest_Path_And_Minimal_Spanning_Tree();
	vh_idx_set.resize(2);
	vh_idx_set[0] = 0;
	vh_idx_set[1] = 0;
	search_num = -1;
	vh_idx_set_backup.resize(vh_idx_set.size());
	for (int i = 0; i < vh_idx_set.size(); ++i)
	{
		vh_idx_set_backup[i] = vh_idx_set[i];
	}
	search_num_backup = search_num;

	discrete_curvature_solver = new Discrete_Curvature();
	solve_mode = MeanCurvature;
	solve_mode_backup = Free;

	mesh_denoising_solver = new Mesh_Denoising();
	fh_normal_iteration_num = 0;
	vh_position_iteration_num = 0;
	sigma_space = 1.0;
	sigma_normal = 1.0;
	fh_normal_iteration_num_backup = fh_normal_iteration_num;
	vh_position_iteration_num_backup = vh_position_iteration_num;
	sigma_space_backup = sigma_space;
	sigma_normal_backup = sigma_normal;

	mesh_parameterization_1_solver = new Mesh_Parameterization_1();
	Show_Parameterization_1_Result = true;
	Show_Parameterization_1_Result_backup = Show_Parameterization_1_Result;
	Param1_Solver_Used = false;

	mesh_parameterization_2_solver = new Mesh_Parameterization_2();
	Show_Parameterization_2_Result = true;
	Show_Parameterization_2_Result_backup = Show_Parameterization_2_Result;
	param2_iteration_num = 0;
	param2_iteration_num_backup = param2_iteration_num;
	Param2_Solver_Used = false;

	arap_surface_modeling_solver = new ARAP_Surface_Modeling();
	Show_ARAP_Surface_Modeling_Result = true;
	Show_ARAP_Surface_Modeling_Result_backup = Show_ARAP_Surface_Modeling_Result;

	barycentric_coordinates_solver = new Barycentric_Coordinates();
	Show_Barycentric_Coordinates_Result = true;
	Show_Barycentric_Coordinates_Result_backup = Show_Barycentric_Coordinates_Result;

	mesh_interpolation_solver = new Mesh_Interpolation();
	Show_Mesh_Interpolation = false;
	Show_Mesh_Interpolation_backup = Show_Mesh_Interpolation;
	interpolation_t = 0;
	interpolation_t_backup = interpolation_t;
	interpolation_t_max = 1;
	interpolation_t_max_backup = interpolation_t_max;
	interpolation_method = 1;
	interpolation_method_backup = interpolation_method;
	need_to_update_source_mesh = false;
	need_to_update_target_mesh = false;
	all_solved = false;

	mesh_simplification_solver = new Mesh_Simplification();
	target_simplification_vh_num = -1;
	target_simplification_vh_num_backup = target_simplification_vh_num;

	cross_fields_solver = new Cross_Fields();
	cross_fields_num = 2;

	remeshing_solver = new Remeshing();
	target_remeshing_edge_length = 1.0;
	target_remeshing_edge_length_backup = target_remeshing_edge_length;

	optimal_delaunay_triangulation_solver = new Optimal_Delaunay_Triangulation();
	need_to_update_optimal_delaunay_triangulation = false;
	target_optimal_delaunay_triangulation_solve_num = 1;
	target_optimal_delaunay_triangulation_solve_num_backup = target_optimal_delaunay_triangulation_solve_num;

	lloyd_iteration_algorithm_solver = new Lloyd_Iteration_Algorithm();
	need_to_update_lloyd_iteration_algorithm = false;
	target_lloyd_iteration_algorithm_solve_num = 1;
	target_lloyd_iteration_algorithm_solve_num_backup = target_lloyd_iteration_algorithm_solve_num;

	geometric_optimization_solver = new Geometric_Optimization();
	Show_Geometric_Optimization_Result = true;
	Show_Geometric_Optimization_Result_backup = Show_Geometric_Optimization_Result;
	geometric_optimization_iteration_num = 0;
	geometric_optimization_iteration_num_backup = geometric_optimization_iteration_num;
	Geometric_Optimization_Solver_Used = false;
}

MeshViewerWidget::~MeshViewerWidget(void)
{
}

bool MeshViewerWidget::LoadMesh(const std::string & filename)
{
	Clear();
	bool read_OK = MeshTools::ReadMesh(mesh, filename);
	std::cout << "Load mesh from file " << filename << std::endl;
	if (read_OK)
	{
		strMeshFileName = QString::fromStdString(filename);
		QFileInfo fi(strMeshFileName);
		strMeshPath = fi.path();
		strMeshBaseName = fi.baseName();
		UpdateMesh();
		update();
		return true;
	}
	return false;
}

void MeshViewerWidget::Clear(void)
{
	mesh.clear();
}

void MeshViewerWidget::UpdateMesh(void)
{
	mesh.update_normals();
	if (mesh.vertices_empty())
	{
		std::cerr << "ERROR: UpdateMesh() No vertices!" << std::endl;
		return;
	}
	ptMin[0] = ptMin[1] = ptMin[2] = DBL_MAX;
	ptMax[0] = ptMax[1] = ptMax[2] = -DBL_MAX;
	for (const auto& vh : mesh.vertices())
	{
		ptMin.minimize(mesh.point(vh));
		ptMax.maximize(mesh.point(vh));
	}

	double avelen = 0.0;
	double maxlen = 0.0;
	double minlen = DBL_MAX;
	for (const auto& eh : mesh.edges())
	{
		double len = mesh.calc_edge_length(eh);
		maxlen = len > maxlen ? len : maxlen;
		minlen = len < minlen ? len : minlen;
		avelen += len;
	}

	SetScenePosition((ptMin + ptMax)*0.5, (ptMin - ptMax).norm()*0.5);
	std::cout << "Information of the input mesh:" << std::endl;
	std::cout << "  [V, E, F] = [" << mesh.n_vertices() << ", " << mesh.n_edges() << ", " << mesh.n_faces() << "]\n";
	std::cout << "  BoundingBox:\n";
	std::cout << "  X: [" << ptMin[0] << ", " << ptMax[0] << "]\n";
	std::cout << "  Y: [" << ptMin[1] << ", " << ptMax[1] << "]\n";
	std::cout << "  Z: [" << ptMin[2] << ", " << ptMax[2] << "]\n";
	std::cout << "  Diag length of BBox: " << (ptMax - ptMin).norm() << std::endl;
	std::cout << "  Edge Length: [" << minlen << ", " << maxlen << "]; AVG: " << avelen / mesh.n_edges() << std::endl;
}

bool MeshViewerWidget::SaveMesh(const std::string & filename)
{
	return MeshTools::WriteMesh(mesh, filename, DBL_DECIMAL_DIG);
}

bool MeshViewerWidget::ScreenShot()
{
	update();
	QString filename = strMeshPath + "/" + QDateTime::currentDateTime().toString("yyyyMMddHHmmsszzz") + QString(".png");
	QImage image = grabFramebuffer();
	image.save(filename);
	std::cout << "Save screen shot to " << filename.toStdString() << std::endl;
	return true;
}

void MeshViewerWidget::SetDrawBoundingBox(bool b)
{
	isDrawBoundingBox = b;
	update();
}
void MeshViewerWidget::SetDrawBoundary(bool b)
{
	isDrawBoundary = b;
	update();
}
void MeshViewerWidget::EnableLighting(bool b)
{
	isEnableLighting = b;
	update();
}
void MeshViewerWidget::EnableDoubleSide(bool b)
{
	isTwoSideLighting = b;
	update();
}

void MeshViewerWidget::ResetView(void)
{
	ResetModelviewMatrix();
	ViewCenter();
	update();
}

void MeshViewerWidget::ViewCenter(void)
{
	if (!mesh.vertices_empty())
	{
		UpdateMesh();
	}
	update();
}

void MeshViewerWidget::CopyRotation(void)
{
	CopyModelViewMatrix();
}

void MeshViewerWidget::LoadRotation(void)
{
	LoadCopyModelViewMatrix();
	update();
}

void MeshViewerWidget::PrintMeshInfo(void)
{
	std::cout << "Mesh Info:\n";
	std::cout << "  [V, E, F] = [" << mesh.n_vertices() << ", " << mesh.n_edges() << ", " << mesh.n_faces() << "]\n";
	std::cout << "  BoundingBox:\n";
	std::cout << "  X: [" << ptMin[0] << ", " << ptMax[0] << "]\n";
	std::cout << "  Y: [" << ptMin[1] << ", " << ptMax[1] << "]\n";
	std::cout << "  Z: [" << ptMin[2] << ", " << ptMax[2] << "]\n";
	std::cout << "  Diag length of BBox: " << (ptMax - ptMin).norm() << std::endl;
}

void MeshViewerWidget::DrawScene(void)
{
	glMatrixMode(GL_PROJECTION);
	glLoadMatrixd(&projectionmatrix[0]);
	glMatrixMode(GL_MODELVIEW);
	glLoadMatrixd(&modelviewmatrix[0]);
	//DrawAxis();
	if (isDrawBoundingBox) DrawBoundingBox();
	if (isDrawBoundary) DrawBoundary();
	if (isEnableLighting) glEnable(GL_LIGHTING);
	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, isTwoSideLighting);
	DrawSceneMesh();
	if (isEnableLighting) glDisable(GL_LIGHTING);
}

void MeshViewerWidget::DrawSceneMesh(void)
{
	if (mesh.n_vertices() == 0) { return; }
	SetMaterial();

	switch (drawmode)
	{
	case POINTS:
		DrawPoints();
		break;
	case WIREFRAME:
		DrawWireframe();
		break;
	case HIDDENLINES:
		DrawHiddenLines();
		break;
	case FLATLINES:
		DrawFlatLines();
		break;
	case FLAT:
		glColor3d(0.8, 0.8, 0.8);
		DrawFlat();
		break;
	case SMOOTH:
		DrawSmooth();
		break;
	case SHORTESET_PATH:
		DrawShortestPath();
		break;
	case DISCRETE_CURVATURE:
		DrawDiscreteCurvature();
		break;
	case MESH_DENOISING:
		DrawMeshDenoising();
		break;
	case MESH_PARAMETERIZATION_1:
		DrawMeshParameterization1();
		break;
	case MESH_PARAMETERIZATION_2:
		DrawMeshParameterization2();
		break;
	case ARAP_SURFACE_MODELING:
		DrawARAPSurfaceModeling();
		break;
	case BARYCENTRIC_COORDINATES:
		DrawBarycentricCoordinates();
		break;
	case MESH_INTERPOLATION:
		DrawMeshInterpolation();
		break;
	case MESH_SIMPLIFICATION:
		DrawMeshSimplification();
		break;
	case CROSS_FIELDS:
		DrawCrossFields();
		break;
	case REMESHING:
		DrawRemeshing();
		break;
	case OPTIMAL_DELAUNAY_TRIANGULATION:
		DrawOptimalDelaunayTriangulation();
		break;
	case LLOYD_ITERATION_ALGORITHM:
		DrawLloydIterationAlgorithm();
		break;
	case GEOMETRIC_OPTIMIZATION:
		DrawGeometricOptimization();
	default:
		break;
	}
}

void MeshViewerWidget::DrawPoints(void) const
{
	glColor3d(1.0, 0.5, 0.5);
	glPointSize(5);
	glBegin(GL_POINTS);
	for (const auto& vh : mesh.vertices())
	{
		glNormal3dv(mesh.normal(vh).data());
		glVertex3dv(mesh.point(vh).data());
	}
	glEnd();
}

void MeshViewerWidget::DrawWireframe(void) const
{
	glColor3d(0.2, 0.2, 0.2);
	glBegin(GL_LINES);
	for (const auto& eh : mesh.edges())
	{
		auto heh = mesh.halfedge_handle(eh, 0);
		auto vh0 = mesh.from_vertex_handle(heh);
		auto vh1 = mesh.to_vertex_handle(heh);
		glNormal3dv(mesh.normal(vh0).data());
		glVertex3dv(mesh.point(vh0).data());
		glNormal3dv(mesh.normal(vh1).data());
		glVertex3dv(mesh.point(vh1).data());
	}
	glEnd();
}

void MeshViewerWidget::DrawHiddenLines() const
{
	glLineWidth(1.0);
	float backcolor[4];
	glGetFloatv(GL_COLOR_CLEAR_VALUE, backcolor);
	glColor4fv(backcolor);
	glDepthRange(0.01, 1.0);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	if (glIsEnabled(GL_LIGHTING))
	{
		glDisable(GL_LIGHTING);
		DrawFlat();
		glEnable(GL_LIGHTING);
	}
	else
	{
		DrawFlat();
	}
	glDepthRange(0.0, 1.0);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glColor3d(.3, .3, .3);
	DrawFlat();
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}

void MeshViewerWidget::DrawFlatLines(void) const
{
	glEnable(GL_POLYGON_OFFSET_FILL);
	glPolygonOffset(1.5f, 2.0f);
	glShadeModel(GL_FLAT);
	//glColor3d(0.8, 0.8, 0.8);
	glColor3d(1.0, 1.0, 1.0);
	DrawFlat();
	glDisable(GL_POLYGON_OFFSET_FILL);
	if (glIsEnabled(GL_LIGHTING))
	{
		glDisable(GL_LIGHTING);
		DrawWireframe();
		glEnable(GL_LIGHTING);
	}
	else
	{
		DrawWireframe();
	}
}

void MeshViewerWidget::DrawFlat(void) const
{
	glBegin(GL_TRIANGLES);
	for (const auto& fh : mesh.faces())
	{
		glNormal3dv(mesh.normal(fh).data());
		for (const auto& fvh : mesh.fv_range(fh))
		{
			glVertex3dv(mesh.point(fvh).data());
		}
	}
	glEnd();
}

void MeshViewerWidget::DrawSmooth(void) const
{
	glColor3d(0.8, 0.8, 0.8);
	glShadeModel(GL_SMOOTH);
	glLoadName(static_cast<GLuint>(mesh.n_vertices()));
	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3, GL_DOUBLE, 0, mesh.points());
	glEnableClientState(GL_NORMAL_ARRAY);
	glNormalPointer(GL_DOUBLE, 0, mesh.vertex_normals());
	for (const auto& fh : mesh.faces())
	{
		glBegin(GL_POLYGON);
		for (const auto& fvh : mesh.fv_range(fh))
		{
			glArrayElement(fvh.idx());
		}
		glEnd();
	}
	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_NORMAL_ARRAY);
}

void MeshViewerWidget::DrawBoundingBox(void) const
{
	float linewidth;
	glGetFloatv(GL_LINE_WIDTH, &linewidth);
	glLineWidth(2.0f);
	glColor3d(.3, .7, .3);
	glBegin(GL_LINES);
	for (const auto& i : { 0, 1 })
	{
		for (const auto& j : { 0, 1 })
		{
			for (const auto& k : { 0, 1 })
			{
				glVertex3d(i ? ptMin[0] : ptMax[0], j ? ptMin[1] : ptMax[1], k ? ptMin[2] : ptMax[2]);
				glVertex3d(~i ? ptMin[0] : ptMax[0], j ? ptMin[1] : ptMax[1], k ? ptMin[2] : ptMax[2]);
				glVertex3d(i ? ptMin[0] : ptMax[0], j ? ptMin[1] : ptMax[1], k ? ptMin[2] : ptMax[2]);
				glVertex3d(i ? ptMin[0] : ptMax[0], ~j ? ptMin[1] : ptMax[1], k ? ptMin[2] : ptMax[2]);
				glVertex3d(i ? ptMin[0] : ptMax[0], j ? ptMin[1] : ptMax[1], k ? ptMin[2] : ptMax[2]);
				glVertex3d(i ? ptMin[0] : ptMax[0], j ? ptMin[1] : ptMax[1], ~k ? ptMin[2] : ptMax[2]);
			}
		}
	}
	glEnd();
	glLineWidth(linewidth);
}

void MeshViewerWidget::DrawBoundary(void) const
{
	float linewidth;
	glGetFloatv(GL_LINE_WIDTH, &linewidth);
	glLineWidth(2.0f);
	glColor3d(0.1, 0.1, 0.1);
	glBegin(GL_LINES);
	for (const auto& eh : mesh.edges())
	{
		if (mesh.is_boundary(eh))
		{
			auto heh = mesh.halfedge_handle(eh, 0);
			auto vh0 = mesh.from_vertex_handle(heh);
			auto vh1 = mesh.to_vertex_handle(heh);
			glNormal3dv(mesh.normal(vh0).data());
			glVertex3dv(mesh.point(vh0).data());
			glNormal3dv(mesh.normal(vh1).data());
			glVertex3dv(mesh.point(vh1).data());
		}
	}
	glEnd();
	glLineWidth(linewidth);
}

void MeshViewerWidget::DrawShortestPath(void)
{
	glEnable(GL_POLYGON_OFFSET_FILL);
	glPolygonOffset(1.5f, 2.0f);
	glShadeModel(GL_FLAT);
	//glColor3d(0.8, 0.8, 0.8);
	glColor3d(1.0, 1.0, 1.0);
	DrawFlat();
	glDisable(GL_POLYGON_OFFSET_FILL);

	glColor3d(0.0, 1.0, 0.0);
	glPointSize(5);
	if (glIsEnabled(GL_LIGHTING))
	{
		glDisable(GL_LIGHTING);

		glBegin(GL_POINTS);
		for (int i = 0; i < vh_idx_set.size(); ++i)
		{
			if (vh_idx_set[i] >= 0 && vh_idx_set[i] < mesh.n_vertices())
			{
				Mesh::VertexHandle vh = mesh.vertex_handle(vh_idx_set[i]);
				glNormal3dv(mesh.normal(vh).data());
				glVertex3dv(mesh.point(vh).data());
			}
		}
		glEnd();

		glEnable(GL_LIGHTING);
	}
	else
	{
		glBegin(GL_POINTS);
		for (int i = 0; i < vh_idx_set.size(); ++i)
		{
			if (vh_idx_set[i] >= 0 && vh_idx_set[i] < mesh.n_vertices())
			{
				Mesh::VertexHandle vh = mesh.vertex_handle(vh_idx_set[i]);
				glNormal3dv(mesh.normal(vh).data());
				glVertex3dv(mesh.point(vh).data());
			}
		}
		glEnd();
	}

	if (!get_Shortest_Path_Set())
	{
		return;
	}

	glColor3d(1.0, 0.0, 0.0);
	if (glIsEnabled(GL_LIGHTING))
	{
		glDisable(GL_LIGHTING);

		glBegin(GL_LINES);
		for (int i = 0; i < shortest_path_set.size(); ++i)
		{
			for (int j = 0; j < shortest_path_set[i].heh_idx_set.size(); ++j)
			{
				auto heh = mesh.halfedge_handle(shortest_path_set[i].heh_idx_set[j]);
				auto vh0 = mesh.from_vertex_handle(heh);
				auto vh1 = mesh.to_vertex_handle(heh);
				glNormal3dv(mesh.normal(vh0).data());
				glVertex3dv(mesh.point(vh0).data());
				glNormal3dv(mesh.normal(vh1).data());
				glVertex3dv(mesh.point(vh1).data());
			}
		}
		glEnd();

		glEnable(GL_LIGHTING);
	}
	else
	{
		glBegin(GL_LINES);
		for (int i = 0; i < shortest_path_set.size(); ++i)
		{
			for (int j = 0; j < shortest_path_set[i].heh_idx_set.size(); ++j)
			{
				auto heh = mesh.halfedge_handle(shortest_path_set[i].heh_idx_set[j]);
				auto vh0 = mesh.from_vertex_handle(heh);
				auto vh1 = mesh.to_vertex_handle(heh);
				glNormal3dv(mesh.normal(vh0).data());
				glVertex3dv(mesh.point(vh0).data());
				glNormal3dv(mesh.normal(vh1).data());
				glVertex3dv(mesh.point(vh1).data());
			}
		}
		glEnd();
	}
}

bool MeshViewerWidget::get_Shortest_Path_Set()
{
	bool is_VH_Idx_Set_Changed = false;

	if (vh_idx_set.size() != vh_idx_set_backup.size())
	{
		is_VH_Idx_Set_Changed = true;
	}
	else
	{
		for (int i = 0; i < vh_idx_set.size(); ++i)
		{
			if (vh_idx_set[i] != vh_idx_set_backup[i])
			{
				is_VH_Idx_Set_Changed = true;

				break;
			}
		}
	}

	if (!is_VH_Idx_Set_Changed && search_num == search_num_backup && strMeshFileName.toStdString() == strMeshFileName_backup)
	{
		return true;
	}
	else
	{
		shortest_path_set = shortest_path_solver->get_Shortest_Path_Set(mesh, vh_idx_set, search_num);

		int error_message = shortest_path_set[0].heh_idx_set[0];

		if (error_message < 0)
		{
			if (error_message == Same_Point)
			{
				std::cout << "您输入的两个顶点坐标重复了，请重新输入~" << std::endl;
			}
			else if (error_message == VH_1_Out_Of_Range)
			{
				std::cout << "您输入的第一个顶点坐标有误，请重新输入~" << std::endl;
			}
			else if (error_message == VH_2_Out_Of_Range)
			{
				std::cout << "您输入的第二个顶点坐标有误，请重新输入~" << std::endl;
			}
			else if (error_message == Both_Out_Of_Range)
			{
				std::cout << "您输入的两个顶点坐标都有误，请重新输入~" << std::endl;
			}
			else if (error_message == Have_Wrong_VH_Idx)
			{
				std::cout << "您输入的顶点坐标中出现错误，请重新输入~" << std::endl;
			}

			std::cout << "顶点坐标范围为 : 0 - " << mesh.n_vertices() - 1 << std::endl;

			vh_idx_set_backup.resize(vh_idx_set.size());
			for (int i = 0; i < vh_idx_set.size(); ++i)
			{
				vh_idx_set_backup[i] = vh_idx_set[i];
			}
			search_num_backup = search_num;
			strMeshFileName_backup = strMeshFileName.toStdString();

			for (int i = 0; i < shortest_path_set.size(); ++i)
			{
				shortest_path_set[i].heh_idx_set.clear();
			}

			return false;
		}
		else
		{
			double total_dist = 0;

			for (int i = 0; i < shortest_path_set.size(); ++i)
			{
				total_dist += shortest_path_set[i].dist;
			}

			std::cout << "dist : " << total_dist << std::endl;

			vh_idx_set_backup.resize(vh_idx_set.size());
			for (int i = 0; i < vh_idx_set.size(); ++i)
			{
				vh_idx_set_backup[i] = vh_idx_set[i];
			}
			search_num_backup = search_num;
			strMeshFileName_backup = strMeshFileName.toStdString();

			return true;
		}
	}
}

void MeshViewerWidget::DrawDiscreteCurvature(void)
{
	glEnable(GL_POLYGON_OFFSET_FILL);
	glPolygonOffset(1.5f, 2.0f);
	glShadeModel(GL_FLAT);
	//glColor3d(0.8, 0.8, 0.8);
	glColor3d(1.0, 1.0, 1.0);
	DrawFlat();
	glDisable(GL_POLYGON_OFFSET_FILL);

	if (!get_Color_On_VH())
	{
		return;
	}

	if (glIsEnabled(GL_LIGHTING))
	{
		glDisable(GL_LIGHTING);

		glShadeModel(GL_SMOOTH);
		glLoadName(static_cast<GLuint>(mesh.n_vertices()));
		glEnableClientState(GL_VERTEX_ARRAY);
		glVertexPointer(3, GL_DOUBLE, 0, mesh.points());
		glEnableClientState(GL_NORMAL_ARRAY);
		glNormalPointer(GL_DOUBLE, 0, mesh.vertex_normals());
		for (const auto& fh : mesh.faces())
		{
			glBegin(GL_POLYGON);
			std::vector<int> face_idx_set;
			for (const auto& fvh : mesh.fv_range(fh))
			{
				face_idx_set.emplace_back(fvh.idx());
			}

			for (int i = 0; i < face_idx_set.size(); ++i)
			{
				if (mesh.is_boundary(mesh.vertex_handle(face_idx_set[i])))
				{
					std::vector<double> average_rgb;
					average_rgb.resize(3);

					int vertex_selected_num = 0;

					for (int j = 0; j < face_idx_set.size(); ++j)
					{
						if (!mesh.is_boundary(mesh.vertex_handle(face_idx_set[j])))
						{
							for (int k = 0; k < 3; ++k)
							{
								average_rgb[k] += vh_color_set[face_idx_set[j]][k];
							}

							++vertex_selected_num;
						}
					}
					for (int j = 0; j < 3; ++j)
					{
						average_rgb[j] /= vertex_selected_num;
					}

					glColor3d(average_rgb[0], average_rgb[1], average_rgb[2]);
				}
				else
				{
					glColor3d(vh_color_set[face_idx_set[i]][0], vh_color_set[face_idx_set[i]][1], vh_color_set[face_idx_set[i]][2]);
				}
				glArrayElement(face_idx_set[i]);
			}
			glEnd();
		}
		glDisableClientState(GL_VERTEX_ARRAY);
		glDisableClientState(GL_NORMAL_ARRAY);

		glEnable(GL_LIGHTING);
	}
	else
	{
		glShadeModel(GL_SMOOTH);
		glLoadName(static_cast<GLuint>(mesh.n_vertices()));
		glEnableClientState(GL_VERTEX_ARRAY);
		glVertexPointer(3, GL_DOUBLE, 0, mesh.points());
		glEnableClientState(GL_NORMAL_ARRAY);
		glNormalPointer(GL_DOUBLE, 0, mesh.vertex_normals());
		for (const auto& fh : mesh.faces())
		{
			glBegin(GL_POLYGON);
			std::vector<int> face_idx_set;
			for (const auto& fvh : mesh.fv_range(fh))
			{
				face_idx_set.emplace_back(fvh.idx());
			}

			for (int i = 0; i < face_idx_set.size(); ++i)
			{
				if (mesh.is_boundary(mesh.vertex_handle(face_idx_set[i])))
				{
					std::vector<double> average_rgb;
					average_rgb.resize(3);

					int vertex_selected_num = 0;

					for (int j = 0; j < face_idx_set.size(); ++j)
					{
						if (!mesh.is_boundary(mesh.vertex_handle(face_idx_set[j])))
						{
							for (int k = 0; k < 3; ++k)
							{
								average_rgb[k] += vh_color_set[face_idx_set[j]][k];
							}

							++vertex_selected_num;
						}
					}
					for (int j = 0; j < 3; ++j)
					{
						average_rgb[j] /= vertex_selected_num;
					}

					glColor3d(average_rgb[0], average_rgb[1], average_rgb[2]);
				}
				else
				{
					glColor3d(vh_color_set[face_idx_set[i]][0], vh_color_set[face_idx_set[i]][1], vh_color_set[face_idx_set[i]][2]);
				}
				glArrayElement(face_idx_set[i]);
			}
			glEnd();
		}
		glDisableClientState(GL_VERTEX_ARRAY);
		glDisableClientState(GL_NORMAL_ARRAY);
	}
}

bool MeshViewerWidget::get_Color_On_VH()
{
	if (mesh.n_vertices() < 3)
	{
		return false;
	}

	if (solve_mode == solve_mode_backup && strMeshFileName.toStdString() == strMeshFileName_backup)
	{
		return true;
	}
	else
	{
		vh_color_set.resize(mesh.n_vertices());

		std::vector<double> vh_curvature_set;

		double vh_curvature_min;
		double vh_curvature_max;

		vh_curvature_set.resize(mesh.n_vertices());

		for (int i = 0; i < vh_color_set.size(); ++i)
		{
			vh_color_set[i].resize(3);

			if (solve_mode == MeanCurvature)
			{
				vh_curvature_set[i] = discrete_curvature_solver->get_Mean_Curvature_of_Vertex(mesh, i);
			}
			else if (solve_mode == AbsoluteMeanCurvature)
			{
				vh_curvature_set[i] = discrete_curvature_solver->get_Absolute_Mean_Curvature_of_Vertex(mesh, i);
			}
			else if (solve_mode == GaussianCurvature)
			{
				vh_curvature_set[i] = discrete_curvature_solver->get_Gaussian_Curvature_of_Vertex(mesh, i);
			}

			if (i == 0)
			{
				vh_curvature_max = vh_curvature_set[i];
				vh_curvature_min = vh_curvature_set[i];
				vh_lowest_color_idx = i;
			}
			else
			{
				if (vh_curvature_set[i] > vh_curvature_max)
				{
					vh_curvature_max = vh_curvature_set[i];
				}
				else if (vh_curvature_set[i] < vh_curvature_min)
				{
					vh_curvature_min = vh_curvature_set[i];
					vh_lowest_color_idx = i;
				}
			}
		}

		/*double time = 1.0;
		double multi = 1;

		for (int i = 0; i < vh_curvature_set.size(); ++i)
		{
			double sign = 1.0;
			if (vh_curvature_set[i] < 0)
			{
				sign = -1.0;
			}
			vh_curvature_set[i] = sign * pow(sign * vh_curvature_set[i] * multi, time);
		}

		double sign = 1.0;

		if (vh_curvature_min < 0)
		{
			sign = -1.0;
		}

		vh_curvature_min = sign * pow(sign * vh_curvature_min * multi, time);

		sign = 1.0;

		if (vh_curvature_max < 0)
		{
			sign = -1.0;
		}

		vh_curvature_max = sign * pow(sign * vh_curvature_max * multi, time);*/

		for (int i = 0; i < vh_color_set.size(); ++i)
		{
			std::vector<double> current_rgb = discrete_curvature_solver->get_RGB_of_Color_Bar_Jet(vh_curvature_set[i], vh_curvature_min, vh_curvature_max);

			for (int j = 0; j < vh_color_set[i].size(); ++j)
			{
				vh_color_set[i][j] = current_rgb[j];
			}
		}

		strMeshFileName_backup = strMeshFileName.toStdString();
		solve_mode_backup = solve_mode;

		return true;
	}
}

void MeshViewerWidget::DrawMeshDenoising(void)
{
	if (!get_Denoising_Mesh())
	{
		std::cout << "获得结果失败!" << std::endl;
	}

	glEnable(GL_POLYGON_OFFSET_FILL);
	glPolygonOffset(1.5f, 2.0f);
	glShadeModel(GL_FLAT);
	//glColor3d(0.8, 0.8, 0.8);
	glColor3d(1.0, 1.0, 1.0);
	glBegin(GL_TRIANGLES);
	for (const auto& fh : mesh.faces())
	{
		glNormal3dv(mesh.normal(fh).data());
		for (const auto& fvh : mesh.fv_range(fh))
		{
			glVertex3dv(mesh.point(fvh).data());
		}
	}
	glEnd();
	glDisable(GL_POLYGON_OFFSET_FILL);
}

bool MeshViewerWidget::get_Denoising_Mesh()
{
	if (mesh.n_vertices() < 3)
	{
		return false;
	}

	if (fh_normal_iteration_num == fh_normal_iteration_num_backup && vh_position_iteration_num == vh_position_iteration_num_backup && sigma_space == sigma_space_backup && sigma_normal == sigma_normal_backup && strMeshFileName.toStdString() == strMeshFileName_backup)
	{
		return true;
	}
	else
	{
		LoadMesh(strMeshFileName.toStdString());

		mesh_denoising_solver->update_Vertex_Position(mesh, fh_normal_iteration_num, vh_position_iteration_num, sigma_space, sigma_normal);

		UpdateMesh();

		strMeshFileName_backup = strMeshFileName.toStdString();

		fh_normal_iteration_num_backup = fh_normal_iteration_num;
		
		vh_position_iteration_num_backup = vh_position_iteration_num;

		sigma_space_backup = sigma_space;

		sigma_normal_backup = sigma_normal;

		return true;
	}
}

void MeshViewerWidget::DrawMeshParameterization1(void)
{
	if (!get_Mesh_Parameterization_1())
	{
		std::cout << "获得结果失败!" << std::endl;
	}

	DrawFlatLines();
}

bool MeshViewerWidget::get_Mesh_Parameterization_1()
{
	if (Show_Parameterization_1_Result == Show_Parameterization_1_Result_backup && Param1_Solver_Used && strMeshFileName.toStdString() == strMeshFileName_backup)
	{
		if (Show_Parameterization_2_Result)
		{
			Show_Parameterization_2_Result_backup = false;
		}
		else
		{
			Show_Parameterization_2_Result_backup = true;
		}

		if (Show_Geometric_Optimization_Result)
		{
			Show_Geometric_Optimization_Result_backup = false;
		}
		else
		{
			Show_Geometric_Optimization_Result_backup = true;
		}

		return true;
	}
	else
	{
		if (strMeshFileName.toStdString() != strMeshFileName_backup || !Param1_Solver_Used)
		{
			LoadMesh(strMeshFileName.toStdString());

			vh_param_1_position_set = mesh_parameterization_1_solver->get_Parameterization(mesh);

			Param1_Solver_Used = true;

			if (strMeshFileName.toStdString() != strMeshFileName_backup)
			{
				Param2_Solver_Used = false;

				Geometric_Optimization_Solver_Used = false;
			}

			need_to_update_optimal_delaunay_triangulation = true;
		}

		if (Show_Parameterization_1_Result)
		{
			for (int i = 0; i < mesh.n_vertices(); ++i)
			{
				for (int j = 0; j < 3; ++j)
				{
					mesh.point(mesh.vertex_handle(i))[j] = vh_param_1_position_set[i][j];
				}
			}

			UpdateMesh();
		}
		else
		{
			LoadMesh(strMeshFileName.toStdString());

			UpdateMesh();
		}

		Show_Parameterization_1_Result_backup = Show_Parameterization_1_Result;
		strMeshFileName_backup = strMeshFileName.toStdString();

		return true;
	}
}

void MeshViewerWidget::DrawMeshParameterization2(void)
{
	if (!get_Mesh_Parameterization_2())
	{
		std::cout << "获得结果失败!" << std::endl;
	}

	DrawFlatLines();
}

bool MeshViewerWidget::get_Mesh_Parameterization_2()
{
	if (Show_Parameterization_2_Result == Show_Parameterization_2_Result_backup && param2_iteration_num == param2_iteration_num_backup && Param2_Solver_Used && strMeshFileName.toStdString() == strMeshFileName_backup)
	{
		if (Show_Parameterization_1_Result)
		{
			Show_Parameterization_1_Result_backup = false;
		}
		else
		{
			Show_Parameterization_1_Result_backup = true;
		}

		if (Show_Geometric_Optimization_Result)
		{
			Show_Geometric_Optimization_Result_backup = false;
		}
		else
		{
			Show_Geometric_Optimization_Result_backup = true;
		}

		return true;
	}
	else
	{
		if (strMeshFileName.toStdString() != strMeshFileName_backup || !Param2_Solver_Used || param2_iteration_num != param2_iteration_num_backup)
		{
			LoadMesh(strMeshFileName.toStdString());

			vh_param_2_position_set = mesh_parameterization_2_solver->Update_VH_Position(mesh, param2_iteration_num);

			Param2_Solver_Used = true;

			if (strMeshFileName.toStdString() != strMeshFileName_backup)
			{
				Param1_Solver_Used = false;

				Geometric_Optimization_Solver_Used = false;
			}

			need_to_update_optimal_delaunay_triangulation = true;
		}

		if (Show_Parameterization_2_Result)
		{
			for (int i = 0; i < mesh.n_vertices(); ++i)
			{
				for (int j = 0; j < 3; ++j)
				{
					mesh.point(mesh.vertex_handle(i))[j] = vh_param_2_position_set[i][j];
				}
			}

			UpdateMesh();
		}
		else
		{
			LoadMesh(strMeshFileName.toStdString());

			UpdateMesh();
		}

		Show_Parameterization_2_Result_backup = Show_Parameterization_2_Result;
		strMeshFileName_backup = strMeshFileName.toStdString();
		param2_iteration_num_backup = param2_iteration_num;

		return true;
	}
}

void MeshViewerWidget::DrawARAPSurfaceModeling(void)
{
	if (!get_ARAP_Surface_Modeling())
	{
		std::cout << "获得结果失败!" << std::endl;
	}

	DrawFlatLines();

	glPointSize(5);
	if (glIsEnabled(GL_LIGHTING))
	{
		glDisable(GL_LIGHTING);

		glBegin(GL_POINTS);
		glColor3d(0.0, 1.0, 0.0);
		for (int i = 0; i < target_vh_idx.size(); ++i)
		{
			if (target_vh_idx[i] >= 0 && target_vh_idx[i] < mesh.n_vertices())
			{
				Mesh::VertexHandle vh = mesh.vertex_handle(target_vh_idx[i]);
				glNormal3dv(mesh.normal(vh).data());
				glVertex3dv(mesh.point(vh).data());
			}
		}
		glColor3d(1.0, 0.0, 0.0);
		for (int i = 0; i < fixed_vh_idx.size(); ++i)
		{
			if (fixed_vh_idx[i] >= 0 && fixed_vh_idx[i] < mesh.n_vertices())
			{
				Mesh::VertexHandle vh = mesh.vertex_handle(fixed_vh_idx[i]);
				glNormal3dv(mesh.normal(vh).data());
				glVertex3dv(mesh.point(vh).data());
			}
		}
		glEnd();

		glEnable(GL_LIGHTING);
	}
	else
	{
		glBegin(GL_POINTS);
		glColor3d(0.0, 1.0, 0.0);
		for (int i = 0; i < target_vh_idx.size(); ++i)
		{
			if (target_vh_idx[i] >= 0 && target_vh_idx[i] < mesh.n_vertices())
			{
				Mesh::VertexHandle vh = mesh.vertex_handle(target_vh_idx[i]);
				glNormal3dv(mesh.normal(vh).data());
				glVertex3dv(mesh.point(vh).data());
			}
		}
		glColor3d(1.0, 0.0, 0.0);
		for (int i = 0; i < fixed_vh_idx.size(); ++i)
		{
			if (fixed_vh_idx[i] >= 0 && fixed_vh_idx[i] < mesh.n_vertices())
			{
				Mesh::VertexHandle vh = mesh.vertex_handle(fixed_vh_idx[i]);
				glNormal3dv(mesh.normal(vh).data());
				glVertex3dv(mesh.point(vh).data());
			}
		}
		glEnd();
	}
}

bool MeshViewerWidget::get_ARAP_Surface_Modeling()
{
	bool is_Input_Set_Changed = false;

	if (target_vh_idx.size() != target_vh_idx_backup.size() || fixed_vh_idx.size() != fixed_vh_idx_backup.size())
	{
		is_Input_Set_Changed = true;
	}
	else
	{
		for (int i = 0; i < target_vh_idx.size(); ++i)
		{
			if (target_vh_idx[i] != target_vh_idx_backup[i])
			{
				is_Input_Set_Changed = true;

				break;
			}
			for (int j = 0; j < 3; ++j)
			{
				if (target_pose[i][j] != target_pose_backup[i][j])
				{
					is_Input_Set_Changed = true;

					break;
				}
			}

			if (is_Input_Set_Changed)
			{
				break;
			}
		}

		for (int i = 0; i < fixed_vh_idx.size(); ++i)
		{
			if (fixed_vh_idx[i] != fixed_vh_idx_backup[i])
			{
				is_Input_Set_Changed = true;

				break;
			}
		}
	}

	if (!is_Input_Set_Changed && Show_ARAP_Surface_Modeling_Result == Show_ARAP_Surface_Modeling_Result_backup && strMeshFileName.toStdString() == strMeshFileName_backup)
	{
		return true;
	}
	else
	{
		LoadMesh(strMeshFileName.toStdString());

		fixed_vh_idx.resize(0);

		for (int i = 0; i < mesh.n_vertices(); ++i)
		{
			if (mesh.is_boundary(mesh.vertex_handle(i)))
			{
				fixed_vh_idx.emplace_back(i);
			}
		}

		vh_arap_modeling_position_set = arap_surface_modeling_solver->Update_VH_Position(mesh, target_vh_idx, target_pose, fixed_vh_idx);

		for (int i = 0; i < mesh.n_vertices(); ++i)
		{
			for (int j = 0; j < 3; ++j)
			{
				mesh.point(mesh.vertex_handle(i))[j] = vh_arap_modeling_position_set[i][j];
			}
		}

		UpdateMesh();

		Show_ARAP_Surface_Modeling_Result_backup = Show_ARAP_Surface_Modeling_Result;
		strMeshFileName_backup = strMeshFileName.toStdString();
		target_vh_idx_backup.resize(target_vh_idx.size());
		target_pose_backup.resize(target_pose.size());
		fixed_vh_idx_backup.resize(fixed_vh_idx.size());
		for (int i = 0; i < target_vh_idx.size(); ++i)
		{
			target_vh_idx_backup[i] = target_vh_idx[i];
			target_pose_backup[i].resize(3);
			for (int j = 0; j < 3; ++j)
			{
				target_pose_backup[i][j] = target_pose[i][j];
			}
		}
		for (int i = 0; i < fixed_vh_idx.size(); ++i)
		{
			fixed_vh_idx_backup[i] = fixed_vh_idx[i];
		}

		return true;
	}
}

void MeshViewerWidget::DrawBarycentricCoordinates(void)
{
	if (!get_Barycentric_Coordinates())
	{
		std::cout << "获得结果失败!" << std::endl;
	}

	DrawFlatLines();

	/*glPointSize(5);
	if (glIsEnabled(GL_LIGHTING))
	{
		glDisable(GL_LIGHTING);

		glBegin(GL_POINTS);
		glColor3d(0.0, 1.0, 0.0);
		for (int i = 0; i < target_vh_idx.size(); ++i)
		{
			if (target_vh_idx[i] >= 0 && target_vh_idx[i] < mesh.n_vertices())
			{
				Mesh::VertexHandle vh = mesh.vertex_handle(target_vh_idx[i]);
				glNormal3dv(mesh.normal(vh).data());
				glVertex3dv(mesh.point(vh).data());
			}
		}
		glColor3d(1.0, 0.0, 0.0);
		for (int i = 0; i < fixed_vh_idx.size(); ++i)
		{
			if (fixed_vh_idx[i] >= 0 && fixed_vh_idx[i] < mesh.n_vertices())
			{
				Mesh::VertexHandle vh = mesh.vertex_handle(fixed_vh_idx[i]);
				glNormal3dv(mesh.normal(vh).data());
				glVertex3dv(mesh.point(vh).data());
			}
		}
		glEnd();

		glEnable(GL_LIGHTING);
	}
	else
	{
		glBegin(GL_POINTS);
		glColor3d(0.0, 1.0, 0.0);
		for (int i = 0; i < target_vh_idx.size(); ++i)
		{
			if (target_vh_idx[i] >= 0 && target_vh_idx[i] < mesh.n_vertices())
			{
				Mesh::VertexHandle vh = mesh.vertex_handle(target_vh_idx[i]);
				glNormal3dv(mesh.normal(vh).data());
				glVertex3dv(mesh.point(vh).data());
			}
		}
		glColor3d(1.0, 0.0, 0.0);
		for (int i = 0; i < fixed_vh_idx.size(); ++i)
		{
			if (fixed_vh_idx[i] >= 0 && fixed_vh_idx[i] < mesh.n_vertices())
			{
				Mesh::VertexHandle vh = mesh.vertex_handle(fixed_vh_idx[i]);
				glNormal3dv(mesh.normal(vh).data());
				glVertex3dv(mesh.point(vh).data());
			}
		}
		glEnd();
	}*/
}

bool MeshViewerWidget::get_Barycentric_Coordinates()
{
	if (Show_Barycentric_Coordinates_Result == Show_Barycentric_Coordinates_Result_backup && strMeshFileName.toStdString() == strMeshFileName_backup)
	{
		return true;
	}
	else
	{
		vh_barycentric_coordinates_position_set = barycentric_coordinates_solver->Update_VH_Position(mesh, target_vh_idx, target_pose, fixed_vh_idx);

		/*std::cout << "--------return fixed pose : " << std::endl;
		for (int i = 0; i < fixed_vh_idx.size(); ++i)
		{
			for (int j = 0; j < 3; ++j)
			{
				std::cout << mesh.point(mesh.vertex_handle(fixed_vh_idx[i])).data()[j] << ",";
			}
			std::cout << std::endl;
		}*/

		UpdateMesh();

		Show_Barycentric_Coordinates_Result_backup = Show_Barycentric_Coordinates_Result;
		strMeshFileName_backup = strMeshFileName.toStdString();

		return true;
	}
}

void MeshViewerWidget::DrawMeshInterpolation(void)
{
	if (!get_Mesh_Interpolation())
	{
		std::cout << "获得结果失败!" << std::endl;
	}

	DrawFlatLines();
}

bool MeshViewerWidget::get_Mesh_Interpolation()
{
	if (all_solved && !need_to_update_source_mesh && !need_to_update_target_mesh && interpolation_method == interpolation_method_backup && interpolation_t == interpolation_t_backup && interpolation_t_max == interpolation_t_max_backup && Show_Mesh_Interpolation == Show_Mesh_Interpolation_backup && strMeshFileName.toStdString() == strMeshFileName_backup)
	{
		return true;
	}
	else
	{
		if (strMeshFileName.toStdString() != strMeshFileName_backup)
		{
			Show_Mesh_Interpolation = false;

			all_solved = false;

			UpdateMesh();
		}

		if (need_to_update_source_mesh)
		{
			Show_Mesh_Interpolation = false;

			mesh_interpolation_solver->update_Source_Mesh(mesh);

			need_to_update_source_mesh = false;

			std::cout << "Source Mesh updated !" << std::endl;
		}
		else if (need_to_update_target_mesh)
		{
			Show_Mesh_Interpolation = false;

			mesh_interpolation_solver->update_Target_Mesh(mesh);

			need_to_update_target_mesh = false;

			std::cout << "Target Mesh updated !" << std::endl;
		}
		else if(Show_Mesh_Interpolation)
		{
			if (!all_solved)
			{
				if (vh_mesh_interpolation_result_set.size() != interpolation_t_max + 1)
				{
					vh_mesh_interpolation_result_set.resize(interpolation_t_max + 1);
				}

				for (int i = 0; i <= interpolation_t_max; ++i)
				{
					vh_mesh_interpolation_result_set[i] = mesh_interpolation_solver->get_Mesh_Interpolation_Result(mesh, interpolation_method, 1.0 * i / interpolation_t_max);

					emit(mesh_interpolation_process_num_updated(i));
				}

				all_solved = true;
			}

			for (int i = 0; i < mesh.n_vertices(); ++i)
			{
				for (int j = 0; j < 3; ++j)
				{
					mesh.point(mesh.vertex_handle(i))[j] = vh_mesh_interpolation_result_set[interpolation_t][i][j];
				}
			}

			UpdateMesh();

			std::cout << "Mesh Interpolation updated !" << std::endl;
		}

		interpolation_method_backup = interpolation_method;
		interpolation_t_backup = interpolation_t;
		interpolation_t_max_backup = interpolation_t_max;
		Show_Mesh_Interpolation_backup = Show_Mesh_Interpolation;
		strMeshFileName_backup = strMeshFileName.toStdString();

		return true;
	}
}

void MeshViewerWidget::DrawMeshSimplification(void)
{
	if (!get_Mesh_Simplification())
	{
		std::cout << "获得结果失败!" << std::endl;
	}

	DrawFlatLines();
}

bool MeshViewerWidget::get_Mesh_Simplification()
{
	if (target_simplification_vh_num == target_simplification_vh_num_backup && strMeshFileName.toStdString() == strMeshFileName_backup)
	{
		return true;
	}
	else
	{
		LoadMesh(strMeshFileName.toStdString());

		mesh_simplification_solver->get_Mesh_Simplification_Result(mesh, target_simplification_vh_num);

		UpdateMesh();

		std::cout << "Mesh Simplification updated !" << std::endl;

		strMeshFileName_backup = strMeshFileName.toStdString();
		target_simplification_vh_num_backup = target_simplification_vh_num;

		return true;
	}
}

void MeshViewerWidget::DrawCrossFields(void)
{
	if (!get_Cross_Fields())
	{
		std::cout << "获得结果失败!" << std::endl;

		DrawFlatLines();

		return;
	}

	DrawFlatLines();

	glColor3d(1.0, 0.0, 0.0);
	for (int i = 0; i < mesh.n_faces(); ++i)
	{
		if (glIsEnabled(GL_LIGHTING))
		{
			glDisable(GL_LIGHTING);

			glBegin(GL_LINES);

			std::vector<Mesh::Point> point_list;

			point_list.resize(2 * cross_fields_num);

			std::vector<Matrix<double, 1, 3>> vector_list;

			vector_list.resize(2 * cross_fields_num);

			for (int j = 0; j < cross_fields_num; ++j)
			{
				vector_list[2 * j] = cross_fields_solver->face_bary_center_set[i] - output.block(i, 3 * j, 1, 3);
				vector_list[2 * j + 1] = cross_fields_solver->face_bary_center_set[i] + output.block(i, 3 * j, 1, 3);
			}

			for (int j = 0; j < cross_fields_num; ++j)
			{
				for (int k = 0; k < 3; ++k)
				{
					point_list[2 * j][k] = vector_list[2 * j](k);
					point_list[2 * j + 1][k] = vector_list[2 * j + 1](k);
				}
			}

			for (int j = 0; j < cross_fields_num; ++j)
			{
				glNormal3dv(mesh.normal(mesh.face_handle(i)).data());
				glVertex3dv(point_list[2 * j].data());
				glNormal3dv(mesh.normal(mesh.face_handle(i)).data());
				glVertex3dv(point_list[2 * j + 1].data());
			}
			glEnd();

			glEnable(GL_LIGHTING);
		}
		else
		{
			glBegin(GL_LINES);

			std::vector<Mesh::Point> point_list;

			point_list.resize(2 * cross_fields_num);

			std::vector<Matrix<double, 1, 3>> vector_list;

			vector_list.resize(2 * cross_fields_num);

			for (int j = 0; j < cross_fields_num; ++j)
			{
				vector_list[2 * j] = cross_fields_solver->face_bary_center_set[i] - output.block(i, 3 * j, 1, 3);
				vector_list[2 * j + 1] = cross_fields_solver->face_bary_center_set[i] + output.block(i, 3 * j, 1, 3);
			}

			for (int j = 0; j < cross_fields_num; ++j)
			{
				for (int k = 0; k < 3; ++k)
				{
					point_list[2 * j][k] = vector_list[2 * j](k);
					point_list[2 * j + 1][k] = vector_list[2 * j + 1](k);
				}
			}

			for (int j = 0; j < cross_fields_num; ++j)
			{
				glNormal3dv(mesh.normal(mesh.face_handle(i)).data());
				glVertex3dv(point_list[2 * j].data());
				glNormal3dv(mesh.normal(mesh.face_handle(i)).data());
				glVertex3dv(point_list[2 * j + 1].data());
			}
			glEnd();
		}
	}
}

bool MeshViewerWidget::get_Cross_Fields()
{
	if (strMeshFileName.toStdString() == strMeshFileName_backup)
	{
		return true;
	}
	else
	{
		cross_fields_solver->get_Cross_Fields_Result(mesh, output, cross_fields_num);

		UpdateMesh();

		std::cout << "Cross Fields updated !" << std::endl;

		strMeshFileName_backup = strMeshFileName.toStdString();

		return true;
	}
}

void MeshViewerWidget::DrawRemeshing(void)
{
	if (!get_Remeshing())
	{
		std::cout << "获得结果失败!" << std::endl;

		DrawFlatLines();

		return;
	}

	DrawFlatLines();
}

bool MeshViewerWidget::get_Remeshing()
{
	if (target_remeshing_edge_length == target_remeshing_edge_length_backup && strMeshFileName.toStdString() == strMeshFileName_backup)
	{
		return true;
	}
	else
	{
		LoadMesh(strMeshFileName.toStdString());

		remeshing_solver->get_Remeshing_Result(mesh, target_remeshing_edge_length);

		UpdateMesh();

		std::cout << "Remeshing updated !" << std::endl;

		target_remeshing_edge_length_backup = target_remeshing_edge_length;
		strMeshFileName_backup = strMeshFileName.toStdString();

		return true;
	}
}

void MeshViewerWidget::DrawOptimalDelaunayTriangulation(void)
{
	if (!get_Optimal_Delaunay_Triangulation())
	{
		std::cout << "获得结果失败!" << std::endl;

		DrawFlatLines();

		return;
	}

	DrawFlatLines();
}

bool MeshViewerWidget::get_Optimal_Delaunay_Triangulation()
{
	if (!need_to_update_optimal_delaunay_triangulation && target_optimal_delaunay_triangulation_solve_num == target_optimal_delaunay_triangulation_solve_num_backup && strMeshFileName.toStdString() == strMeshFileName_backup)
	{
		return true;
	}
	else
	{
		optimal_delaunay_triangulation_solver->get_Optimal_Delaunay_Triangulation_Result(mesh, target_optimal_delaunay_triangulation_solve_num);

		UpdateMesh();

		std::cout << "Optimal Delaunay Triangulation updated !" << std::endl;

		need_to_update_optimal_delaunay_triangulation = false;
		target_optimal_delaunay_triangulation_solve_num_backup = target_optimal_delaunay_triangulation_solve_num;
		strMeshFileName_backup = strMeshFileName.toStdString();

		return true;
	}
}

void MeshViewerWidget::DrawLloydIterationAlgorithm(void)
{
	if (!get_Lloyd_Iteration_Algorithm())
	{
		std::cout << "获得结果失败!" << std::endl;

		DrawFlatLines();

		return;
	}

	DrawFlatLines();
}

bool MeshViewerWidget::get_Lloyd_Iteration_Algorithm()
{
	if (!need_to_update_lloyd_iteration_algorithm && target_lloyd_iteration_algorithm_solve_num == target_lloyd_iteration_algorithm_solve_num_backup && strMeshFileName.toStdString() == strMeshFileName_backup)
	{
		return true;
	}
	else
	{
		lloyd_iteration_algorithm_solver->get_Lloyd_Iteration_Algorithm_Result(mesh, target_lloyd_iteration_algorithm_solve_num);

		UpdateMesh();

		std::cout << "Lloyd Iteration Algorithm updated !" << std::endl;

		need_to_update_lloyd_iteration_algorithm = false;
		target_lloyd_iteration_algorithm_solve_num_backup = target_lloyd_iteration_algorithm_solve_num;
		strMeshFileName_backup = strMeshFileName.toStdString();

		return true;
	}
}

void MeshViewerWidget::DrawGeometricOptimization(void)
{
	if (!get_Geometric_Optimization())
	{
		std::cout << "获得结果失败!" << std::endl;
	}

	DrawFlatLines();
}

bool MeshViewerWidget::get_Geometric_Optimization()
{
	if (Show_Geometric_Optimization_Result == Show_Geometric_Optimization_Result_backup && geometric_optimization_iteration_num == geometric_optimization_iteration_num_backup && Geometric_Optimization_Solver_Used && strMeshFileName.toStdString() == strMeshFileName_backup)
	{
		if (Show_Parameterization_1_Result)
		{
			Show_Parameterization_1_Result_backup = false;
		}
		else
		{
			Show_Parameterization_1_Result_backup = true;
		}

		if (Show_Parameterization_2_Result)
		{
			Show_Parameterization_2_Result_backup = false;
		}
		else
		{
			Show_Parameterization_2_Result_backup = true;
		}

		return true;
	}
	else
	{
		if (strMeshFileName.toStdString() != strMeshFileName_backup || !Geometric_Optimization_Solver_Used || geometric_optimization_iteration_num != geometric_optimization_iteration_num_backup)
		{
			LoadMesh(strMeshFileName.toStdString());

			vh_geometric_optimization_position_set = geometric_optimization_solver->Update_VH_Position(mesh, geometric_optimization_iteration_num);

			Geometric_Optimization_Solver_Used = true;

			if (strMeshFileName.toStdString() != strMeshFileName_backup)
			{
				Param1_Solver_Used = false;

				Param2_Solver_Used = false;
			}

			need_to_update_optimal_delaunay_triangulation = true;
		}

		if (Show_Geometric_Optimization_Result)
		{
			for (int i = 0; i < mesh.n_vertices(); ++i)
			{
				for (int j = 0; j < 3; ++j)
				{
					mesh.point(mesh.vertex_handle(i))[j] = vh_geometric_optimization_position_set[i][j];
				}
			}

			UpdateMesh();
		}
		else
		{
			LoadMesh(strMeshFileName.toStdString());

			UpdateMesh();
		}

		Show_Geometric_Optimization_Result_backup = Show_Geometric_Optimization_Result;
		strMeshFileName_backup = strMeshFileName.toStdString();
		geometric_optimization_iteration_num_backup = geometric_optimization_iteration_num;

		return true;
	}
}
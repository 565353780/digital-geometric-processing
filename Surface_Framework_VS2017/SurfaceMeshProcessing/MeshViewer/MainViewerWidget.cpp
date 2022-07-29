#include "MainViewerWidget.h"
#include "MeshParamWidget.h"
#include "InteractiveViewerWidget.h"
#include <QLayout>
#include <QMessageBox>
#include <iostream>

MainViewerWidget::MainViewerWidget(QWidget* _parent/* =0 */)
	:loadmeshsuccess(false)
{
	InitViewerWindow();

	connect(meshparamwidget->Text_input_vh_btn, SIGNAL(clicked()), this, SLOT(GetNumFromText()));

	connect(meshparamwidget->Btn_mean_curvature, SIGNAL(clicked()), this, SLOT(SetSolveMeanCurvature()));
	connect(meshparamwidget->Btn_absolute_mean_curvature, SIGNAL(clicked()), this, SLOT(SetSolveAbsoluteMeanCurvature()));
	connect(meshparamwidget->Btn_gaussian_curvature, SIGNAL(clicked()), this, SLOT(SetSolveGaussianCurvature()));

	color_bar_widget = new ColorBarWidget();
	color_bar_widget->setWindowTitle("Color Bar Widget");
	color_bar_widget->resize(500, 500);

	connect(meshparamwidget->Btn_mesh_denoising, SIGNAL(clicked()), this, SLOT(Get4ParamFromText()));

	connect(meshparamwidget->Btn_show_mesh_param_1, SIGNAL(clicked()), this, SLOT(ShowMeshParam1Result()));
	connect(meshparamwidget->Btn_show_source_mesh_1, SIGNAL(clicked()), this, SLOT(ShowSourceMesh1()));

	connect(meshparamwidget->Btn_show_mesh_param_2, SIGNAL(clicked()), this, SLOT(ShowMeshParam2Result()));
	connect(meshparamwidget->Btn_show_source_mesh_2, SIGNAL(clicked()), this, SLOT(ShowSourceMesh2()));

	connect(meshparamwidget->Btn_update_arap_modeling, SIGNAL(clicked()), this, SLOT(UpdateARAPParams()));

	connect(meshparamwidget->Btn_save_as_source_mesh, SIGNAL(clicked()), this, SLOT(SaveAsSourceMesh()));
	connect(meshparamwidget->Btn_save_as_target_mesh, SIGNAL(clicked()), this, SLOT(SaveAsTargetMesh()));
	connect(meshparamwidget->Btn_show_interpolation, SIGNAL(clicked()), this, SLOT(ShowInterpolationResult()));
	connect(meshparamwidget->Slider_mesh_interpolation, SIGNAL(valueChanged(int)), this, SLOT(UpdateInterpolationParam(int)));
	connect(meshviewerwidget, SIGNAL(mesh_interpolation_process_num_updated(int)), this, SLOT(UpdateInterpolationProcessBar(int)));

	connect(meshparamwidget->Btn_show_simplification_result, SIGNAL(clicked()), this, SLOT(ShowSimplificationResult()));

	connect(meshparamwidget->Btn_show_remeshing_result, SIGNAL(clicked()), this, SLOT(UpdateRemeshingTargetEdgeLength()));

	connect(meshparamwidget->Btn_show_optimal_delaunay_triangulation_result, SIGNAL(clicked()), this, SLOT(UpdateOptimalDelaunayTriangulationSolveNum()));

	connect(meshparamwidget->Btn_show_lloyd_iteration_algorithm_result, SIGNAL(clicked()), this, SLOT(UpdateLloydIterationAlgorithmSolveNum()));

	connect(meshparamwidget->Btn_show_geometric_optimization_result, SIGNAL(clicked()), this, SLOT(ShowMeshGeometricOptimizationResult()));
	connect(meshparamwidget->Btn_show_source_mesh_geometric_optimization, SIGNAL(clicked()), this, SLOT(ShowSourceMeshGeometricOptimization()));
}

MainViewerWidget::~MainViewerWidget(void)
{
}

void MainViewerWidget::InitViewerWindow(void)
{
	CreateViewerDialog();
	CreateParamWidget();

	QHBoxLayout* main_layout = new QHBoxLayout();
	main_layout->addWidget(meshparamwidget);
	main_layout->addWidget(meshviewerwidget, 10);
	this->setLayout(main_layout);
}

void MainViewerWidget::CreateParamWidget(void)
{
	meshparamwidget = new MeshParamWidget();
	connect(meshparamwidget, SIGNAL(PrintInfoSignal()), meshviewerwidget, SLOT(PrintMeshInfo()));
}

void MainViewerWidget::CreateViewerDialog(void)
{
	meshviewerwidget = new InteractiveViewerWidget(NULL);
	meshviewerwidget->setAcceptDrops(true);
	connect(meshviewerwidget, SIGNAL(LoadMeshOKSignal(bool, QString)), SLOT(LoadMeshFromInner(bool, QString)));
}

void MainViewerWidget::OpenMeshGUI(const QString & fname)
{
	if (fname.isEmpty() || !meshviewerwidget->LoadMesh(fname.toStdString()))
	{
		QString msg = "Cannot read mesh from file:\n '" + fname + "'";
		QMessageBox::critical(NULL, windowTitle(), msg);
	}
	else
	{
		loadmeshsuccess = true;
		emit(haveLoadMesh(fname));
	}
}

void MainViewerWidget::SaveMeshGUI(const QString & fname)
{
	if (fname.isEmpty() || !meshviewerwidget->SaveMesh(fname.toStdString()))
	{
		QString msg = "Cannot read mesh from file:\n '" + fname + "'";
		QMessageBox::critical(NULL, windowTitle(), msg);
	}
}

void MainViewerWidget::LoadMeshFromInner(bool OK, QString fname)
{
	loadmeshsuccess = OK;
	emit(haveLoadMesh(fname));
}

void MainViewerWidget::Open(void)
{
	QString fileName = QFileDialog::getOpenFileName(this,
		tr("Open mesh file"),
		tr(""),
		tr("Mesh Files (*.obj *.off *.ply *.stl);;"
		"OFF Files (*.off);;"
		"OBJ Files (*.obj);;"
		"PLY Files (*.ply);;"
		"STL Files (*.stl);;"
		"All Files (*)"));
	if (!fileName.isEmpty())
	{
		OpenMeshGUI(fileName);
	}
}

void MainViewerWidget::Save(void)
{
	QString fileName = QFileDialog::getSaveFileName(this,
		tr("Save mesh file"),
		tr("untitled.obj"),
		tr("OBJ Files (*.obj);;"
		"OFF Files (*.off);;"
		"PLY Files (*.ply);;"
		"STL Files (*.stl);;"
		"All Files (*)"));
	if (!fileName.isEmpty())
	{
		SaveMeshGUI(fileName);
	}
}

void MainViewerWidget::ClearMesh(void)
{
	if (loadmeshsuccess)
	{
		loadmeshsuccess = false;
		meshviewerwidget->Clear();
	}
}

void MainViewerWidget::Screenshot(void)
{
	meshviewerwidget->ScreenShot();
}

void MainViewerWidget::ShowPoints(void)
{
	meshviewerwidget->SetDrawMode(InteractiveViewerWidget::POINTS);
}

void MainViewerWidget::ShowWireframe(void)
{
	meshviewerwidget->SetDrawMode(InteractiveViewerWidget::WIREFRAME);
}

void MainViewerWidget::ShowHiddenLines(void)
{
	meshviewerwidget->SetDrawMode(InteractiveViewerWidget::HIDDENLINES);
}

void MainViewerWidget::ShowFlatLines(void)
{
	meshviewerwidget->SetDrawMode(InteractiveViewerWidget::FLATLINES);
}

void MainViewerWidget::ShowFlat(void)
{
	meshviewerwidget->SetDrawMode(InteractiveViewerWidget::FLAT);
}

void MainViewerWidget::ShowSmooth(void)
{
	meshviewerwidget->SetDrawMode(InteractiveViewerWidget::SMOOTH);
}

void MainViewerWidget::Lighting(bool b)
{
	meshviewerwidget->EnableLighting(b);
}

void MainViewerWidget::DoubleSideLighting(bool b)
{
	meshviewerwidget->EnableDoubleSide(b);
}

void MainViewerWidget::ShowBoundingBox(bool b)
{
	meshviewerwidget->SetDrawBoundingBox(b);
}

void MainViewerWidget::ShowBoundary(bool b)
{
	meshviewerwidget->SetDrawBoundary(b);
}

void MainViewerWidget::ResetView(void)
{
	meshviewerwidget->ResetView();
}

void MainViewerWidget::ViewCenter(void)
{
	meshviewerwidget->ViewCenter();
}

void MainViewerWidget::CopyRotation(void)
{
	meshviewerwidget->CopyRotation();
}

void MainViewerWidget::LoadRotation(void)
{
	meshviewerwidget->LoadRotation();
}

void MainViewerWidget::ShowShortestPath(void)
{
	meshviewerwidget->SetDrawMode(InteractiveViewerWidget::SHORTESET_PATH);
}

void MainViewerWidget::ShowDiscreteCurvature(void)
{
	meshviewerwidget->SetDrawMode(InteractiveViewerWidget::DISCRETE_CURVATURE);
}

void MainViewerWidget::ShowMeshDenoising(void)
{
	meshviewerwidget->SetDrawMode(InteractiveViewerWidget::MESH_DENOISING);
}

void MainViewerWidget::ShowMeshParameterization1(void)
{
	meshviewerwidget->SetDrawMode(InteractiveViewerWidget::MESH_PARAMETERIZATION_1);
}

void MainViewerWidget::ShowMeshParameterization2(void)
{
	meshviewerwidget->SetDrawMode(InteractiveViewerWidget::MESH_PARAMETERIZATION_2);
}

void MainViewerWidget::ShowARAPSurfaceModeling(void)
{
	meshviewerwidget->SetDrawMode(InteractiveViewerWidget::ARAP_SURFACE_MODELING);
}

void MainViewerWidget::ShowBarycentricCoordinates(void)
{
	meshviewerwidget->SetDrawMode(InteractiveViewerWidget::BARYCENTRIC_COORDINATES);
}

void MainViewerWidget::ShowMeshInterpolation(void)
{
	meshviewerwidget->SetDrawMode(InteractiveViewerWidget::MESH_INTERPOLATION);
}

void MainViewerWidget::ShowMeshSimplification(void)
{
	meshviewerwidget->SetDrawMode(InteractiveViewerWidget::MESH_SIMPLIFICATION);
}

void MainViewerWidget::ShowCrossFields(void)
{
	meshviewerwidget->SetDrawMode(InteractiveViewerWidget::CROSS_FIELDS);
}

void MainViewerWidget::ShowRemeshing(void)
{
	meshviewerwidget->SetDrawMode(InteractiveViewerWidget::REMESHING);
}

void MainViewerWidget::ShowOptimalDelaunayTriangulation(void)
{
	meshviewerwidget->SetDrawMode(InteractiveViewerWidget::OPTIMAL_DELAUNAY_TRIANGULATION);
}

void MainViewerWidget::ShowLloydIterationAlgorithm(void)
{
	meshviewerwidget->SetDrawMode(InteractiveViewerWidget::LLOYD_ITERATION_ALGORITHM);
}

void MainViewerWidget::ShowGeometricOptimization(void)
{
	meshviewerwidget->SetDrawMode(InteractiveViewerWidget::GEOMETRIC_OPTIMIZATION);
}

void MainViewerWidget::GetNumFromText(void)
{
	QString vh_string = meshparamwidget->Text_input_vh_idx->toPlainText();
	QString search_num_string = meshparamwidget->Text_input_search_num->toPlainText();

	if (vh_string.contains(","))
	{
		QStringList num_input_list = vh_string.split(",");

		meshviewerwidget->vh_idx_set.clear();

		for (int i = 0; i < num_input_list.size(); ++i)
		{
			if (num_input_list[i].size() > 0)
			{
				int num_insert = num_input_list[i].toInt();

				bool have_Same_Num = false;

				for (int j = 0; j < meshviewerwidget->vh_idx_set.size(); ++j)
				{
					if (meshviewerwidget->vh_idx_set[j] == num_insert)
					{
						have_Same_Num = true;

						break;
					}
				}

				if (!have_Same_Num)
				{
					meshviewerwidget->vh_idx_set.emplace_back(num_input_list[i].toInt());
				}
			}
		}
	}

	if (search_num_string.toStdString() != "")
	{
		meshviewerwidget->search_num = search_num_string.toInt();
	}
	else
	{
		meshviewerwidget->search_num = -1;
	}

	meshviewerwidget->update();
}

void MainViewerWidget::SetSolveMeanCurvature(void)
{
	meshviewerwidget->solve_mode = MeanCurvature;
}

void MainViewerWidget::SetSolveAbsoluteMeanCurvature(void)
{
	meshviewerwidget->solve_mode = AbsoluteMeanCurvature;
}

void MainViewerWidget::SetSolveGaussianCurvature(void)
{
	meshviewerwidget->solve_mode = GaussianCurvature;
}

void MainViewerWidget::Show_Color_Bar_Widget(void)
{
	color_bar_widget->show();
}

void MainViewerWidget::Get4ParamFromText(void)
{
	QString fh_normal_iteration_num_string = meshparamwidget->Text_input_fh_normal_iteration_num->toPlainText();
	QString vh_position_iteration_num_string = meshparamwidget->Text_input_vh_position_iteration_num->toPlainText();
	QString sigma_space_string = meshparamwidget->Text_input_sigma_space->toPlainText();
	QString sigma_normal_string = meshparamwidget->Text_input_sigma_normal->toPlainText();

	if (fh_normal_iteration_num_string.toStdString() != "")
	{
		meshviewerwidget->fh_normal_iteration_num = fh_normal_iteration_num_string.toInt();
	}
	else
	{
		meshviewerwidget->fh_normal_iteration_num = 0;
	}

	if (vh_position_iteration_num_string.toStdString() != "")
	{
		meshviewerwidget->vh_position_iteration_num = vh_position_iteration_num_string.toInt();
	}
	else
	{
		meshviewerwidget->vh_position_iteration_num = 0;
	}

	if (sigma_space_string.toStdString() != "")
	{
		meshviewerwidget->sigma_space = sigma_space_string.toDouble();
	}
	else
	{
		meshviewerwidget->sigma_space = 1.0;
	}

	if (sigma_normal_string.toStdString() != "")
	{
		meshviewerwidget->sigma_normal = sigma_normal_string.toDouble();
	}
	else
	{
		meshviewerwidget->sigma_normal = 1.0;
	}

	meshviewerwidget->update();
}

void MainViewerWidget::ShowMeshParam1Result(void)
{
	meshviewerwidget->Show_Parameterization_1_Result = true;

	meshviewerwidget->update();
}

void MainViewerWidget::ShowSourceMesh1(void)
{
	meshviewerwidget->Show_Parameterization_1_Result = false;

	meshviewerwidget->update();
}

void MainViewerWidget::ShowMeshParam2Result(void)
{
	QString param2_iteration_num_string = meshparamwidget->Text_input_param2_iteration_num->toPlainText();

	if (param2_iteration_num_string.toStdString() != "")
	{
		meshviewerwidget->param2_iteration_num = param2_iteration_num_string.toInt();
	}

	meshviewerwidget->Show_Parameterization_2_Result = true;

	meshviewerwidget->update();
}

void MainViewerWidget::ShowSourceMesh2(void)
{
	meshviewerwidget->Show_Parameterization_2_Result = false;

	meshviewerwidget->update();
}

void MainViewerWidget::UpdateARAPParams(void)
{
	QString target_vh_idx_string = meshparamwidget->Text_input_arap_modeling_target_vh_idx->toPlainText();
	QString target_pose_string = meshparamwidget->Text_input_arap_modeling_target_pose->toPlainText();
	QString fixed_vh_idx_string = meshparamwidget->Text_input_arap_modeling_fixed_vh_idx->toPlainText();

	if (target_vh_idx_string.contains(","))
	{
		QStringList target_vh_idx_string_list = target_vh_idx_string.split(",");

		meshviewerwidget->target_vh_idx.clear();

		for (int i = 0; i < target_vh_idx_string_list.size(); ++i)
		{
			if (target_vh_idx_string_list[i].size() > 0)
			{
				int target_vh_idx = target_vh_idx_string_list[i].toInt();

				bool have_Same_Num = false;

				for (int j = 0; j < meshviewerwidget->target_vh_idx.size(); ++j)
				{
					if (meshviewerwidget->target_vh_idx[j] == target_vh_idx)
					{
						have_Same_Num = true;

						break;
					}
				}

				if (!have_Same_Num)
				{
					meshviewerwidget->target_vh_idx.emplace_back(target_vh_idx);
				}
			}
		}
	}
	else if (target_vh_idx_string != "")
	{
		meshviewerwidget->target_vh_idx.resize(0);

		meshviewerwidget->target_vh_idx.emplace_back(target_vh_idx_string.toInt());
	}
	else
	{
		meshviewerwidget->target_vh_idx.resize(0);
	}

	if (target_pose_string.contains(","))
	{
		QStringList target_pose_string_list = target_pose_string.split(",");

		meshviewerwidget->target_pose.clear();

		meshviewerwidget->target_pose.resize(meshviewerwidget->target_vh_idx.size());

		if (target_pose_string_list.size() == 3)
		{
			for (int i = 0; i < 3; ++i)
			{
				if (target_pose_string_list[i].size() > 0)
				{
					double target_pose = target_pose_string_list[i].toDouble();

					for (int j = 0; j < meshviewerwidget->target_pose.size(); ++j)
					{
						meshviewerwidget->target_pose[j].emplace_back(target_pose);
					}
				}
			}
		}
	}
	else
	{
		meshviewerwidget->target_pose.resize(0);
	}

	if (fixed_vh_idx_string.contains(","))
	{
		QStringList fixed_vh_idx_string_list = fixed_vh_idx_string.split(",");

		meshviewerwidget->fixed_vh_idx.clear();

		for (int i = 0; i < fixed_vh_idx_string_list.size(); ++i)
		{
			if (fixed_vh_idx_string_list[i].size() > 0)
			{
				int fixed_vh_idx = fixed_vh_idx_string_list[i].toInt();

				bool have_Same_Num = false;

				for (int j = 0; j < meshviewerwidget->fixed_vh_idx.size(); ++j)
				{
					if (meshviewerwidget->fixed_vh_idx[j] == fixed_vh_idx)
					{
						have_Same_Num = true;

						break;
					}
				}

				if (!have_Same_Num)
				{
					meshviewerwidget->fixed_vh_idx.emplace_back(fixed_vh_idx);
				}
			}
		}
	}
	else if (target_vh_idx_string != "")
	{
		meshviewerwidget->fixed_vh_idx.resize(0);

		meshviewerwidget->fixed_vh_idx.emplace_back(fixed_vh_idx_string.toInt());
	}
	else
	{
		meshviewerwidget->fixed_vh_idx.resize(0);
	}

	meshviewerwidget->update();
}

void MainViewerWidget::ShowInterpolationResult(void)
{
	meshviewerwidget->Show_Mesh_Interpolation = true;
	meshviewerwidget->interpolation_t_max = 50;

	QString interpolation_method_string = meshparamwidget->Text_input_interpolation_method->toPlainText();

	if (interpolation_method_string != "")
	{
		meshviewerwidget->interpolation_method = interpolation_method_string.toInt();
	}
	else
	{
		meshviewerwidget->interpolation_method = 1;
	}

	meshviewerwidget->interpolation_t = 0;

	meshviewerwidget->update();
}

void MainViewerWidget::SaveAsSourceMesh(void)
{
	meshviewerwidget->need_to_update_source_mesh = true;

	meshviewerwidget->update();
}

void MainViewerWidget::SaveAsTargetMesh(void)
{
	meshviewerwidget->need_to_update_target_mesh = true;

	meshviewerwidget->update();
}

void MainViewerWidget::UpdateInterpolationParam(int param_t)
{
	QString interpolation_method_string = meshparamwidget->Text_input_interpolation_method->toPlainText();

	if (interpolation_method_string != "")
	{
		meshviewerwidget->interpolation_method = interpolation_method_string.toInt();
	}
	else
	{
		meshviewerwidget->interpolation_method = 1;
	}

	meshviewerwidget->interpolation_t = param_t;

	meshviewerwidget->update();
}

void MainViewerWidget::UpdateInterpolationProcessBar(int process_num)
{
	meshparamwidget->Bar_mesh_interpolation->setValue(process_num);
}

void MainViewerWidget::ShowSimplificationResult(void)
{
	QString simplification_vh_num_string = meshparamwidget->Text_input_simplification_vertex_num->toPlainText();

	if (simplification_vh_num_string != "")
	{
		meshviewerwidget->target_simplification_vh_num = simplification_vh_num_string.toInt();
	}
	else
	{
		meshviewerwidget->target_simplification_vh_num = 2;
	}

	meshviewerwidget->update();
}

void MainViewerWidget::UpdateRemeshingTargetEdgeLength(void)
{
	QString target_edge_length_string = meshparamwidget->Text_input_remeshing_target_edge_length->toPlainText();

	if (target_edge_length_string != "")
	{
		meshviewerwidget->target_remeshing_edge_length = target_edge_length_string.toDouble();
	}
	else
	{
		meshviewerwidget->target_remeshing_edge_length = 1.0;
	}

	meshviewerwidget->update();
}

void MainViewerWidget::UpdateOptimalDelaunayTriangulationSolveNum()
{
	QString solve_num_string = meshparamwidget->Text_input_optimal_delaunay_triangulation_solve_num->toPlainText();

	if (solve_num_string != "")
	{
		meshviewerwidget->target_optimal_delaunay_triangulation_solve_num = solve_num_string.toInt();
	}
	else
	{
		meshviewerwidget->target_optimal_delaunay_triangulation_solve_num = 1;
	}

	meshviewerwidget->need_to_update_optimal_delaunay_triangulation = true;

	meshviewerwidget->update();
}

void MainViewerWidget::UpdateLloydIterationAlgorithmSolveNum()
{
	QString solve_num_string = meshparamwidget->Text_input_lloyd_iteration_algorithm_solve_num->toPlainText();

	if (solve_num_string != "")
	{
		meshviewerwidget->target_lloyd_iteration_algorithm_solve_num = solve_num_string.toInt();
	}
	else
	{
		meshviewerwidget->target_lloyd_iteration_algorithm_solve_num = 1;
	}

	meshviewerwidget->need_to_update_lloyd_iteration_algorithm = true;

	meshviewerwidget->update();
}

void MainViewerWidget::ShowMeshGeometricOptimizationResult(void)
{
	QString geometric_optimization_iteration_num_string = meshparamwidget->Text_input_geometric_optimization_iteration_num->toPlainText();

	if (geometric_optimization_iteration_num_string.toStdString() != "")
	{
		meshviewerwidget->geometric_optimization_iteration_num = geometric_optimization_iteration_num_string.toInt();
	}

	meshviewerwidget->Show_Geometric_Optimization_Result = true;

	meshviewerwidget->update();
}

void MainViewerWidget::ShowSourceMeshGeometricOptimization(void)
{
	meshviewerwidget->Show_Geometric_Optimization_Result = false;

	meshviewerwidget->update();
}
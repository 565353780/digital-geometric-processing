#include "MeshParamWidget.h"

MeshParamWidget::MeshParamWidget(QWidget *parent)
	: QWidget(parent)
{
	CreateTabWidget();
	CreateLayout();
}

MeshParamWidget::~MeshParamWidget()
{
}

void MeshParamWidget::CreateTabWidget(void)
{
	pbPrintInfo = new QPushButton(tr("Print Mesh Info"));
	connect(pbPrintInfo, SIGNAL(clicked()), SIGNAL(PrintInfoSignal()));

	Text_input_vh_idx = new QTextEdit();
	Text_input_search_num = new QTextEdit();
	Text_input_vh_btn = new QPushButton(tr("Solve"));

	Text_input_vh_idx->setFixedHeight(27);
	Text_input_search_num->setFixedHeight(27);

	QVBoxLayout* shortest_path_input_group_layout = new QVBoxLayout();
	shortest_path_input_group_layout->addWidget(Text_input_vh_idx);
	shortest_path_input_group_layout->addWidget(Text_input_search_num);

	QGroupBox* shortest_path_input_group = new QGroupBox();
	shortest_path_input_group->setLayout(shortest_path_input_group_layout);

	QHBoxLayout* shortest_path_btn_group_layout = new QHBoxLayout();
	shortest_path_btn_group_layout->addWidget(shortest_path_input_group);
	shortest_path_btn_group_layout->addWidget(Text_input_vh_btn);

	QGroupBox* shortest_path_group = new QGroupBox(tr("Shortest Path"));
	shortest_path_group->setLayout(shortest_path_btn_group_layout);

	Btn_mean_curvature = new QPushButton(tr("Mean Curvature"));
	Btn_absolute_mean_curvature = new QPushButton(tr("Absolute Mean Curvature"));
	Btn_gaussian_curvature = new QPushButton(tr("Gaussian Curvature"));

	QHBoxLayout* discrete_curvature_btn_group_layout = new QHBoxLayout();
	discrete_curvature_btn_group_layout->addWidget(Btn_mean_curvature);
	discrete_curvature_btn_group_layout->addWidget(Btn_absolute_mean_curvature);
	discrete_curvature_btn_group_layout->addWidget(Btn_gaussian_curvature);

	QGroupBox* discrete_curvature_btn_group = new QGroupBox(tr("Discrete Curvature"));
	discrete_curvature_btn_group->setLayout(discrete_curvature_btn_group_layout);

	Text_input_fh_normal_iteration_num = new QTextEdit();
	Text_input_vh_position_iteration_num = new QTextEdit();
	Text_input_sigma_space = new QTextEdit();
	Text_input_sigma_normal = new QTextEdit();
	Btn_mesh_denoising = new QPushButton(tr("Solve"));

	Text_input_fh_normal_iteration_num->setFixedHeight(27);
	Text_input_vh_position_iteration_num->setFixedHeight(27);
	Text_input_sigma_space->setFixedHeight(27);
	Text_input_sigma_normal->setFixedHeight(27);

	QVBoxLayout* iteration_group_layout = new QVBoxLayout();
	iteration_group_layout->addWidget(Text_input_fh_normal_iteration_num);
	iteration_group_layout->addWidget(Text_input_vh_position_iteration_num);

	QGroupBox* iteration_group = new QGroupBox();
	iteration_group->setLayout(iteration_group_layout);

	QVBoxLayout* sigma_group_layout = new QVBoxLayout();
	sigma_group_layout->addWidget(Text_input_sigma_space);
	sigma_group_layout->addWidget(Text_input_sigma_normal);

	QGroupBox* sigma_group = new QGroupBox();
	sigma_group->setLayout(sigma_group_layout);

	QHBoxLayout* mesh_denoising_group_layout = new QHBoxLayout();
	mesh_denoising_group_layout->addWidget(iteration_group);
	mesh_denoising_group_layout->addWidget(sigma_group);
	mesh_denoising_group_layout->addWidget(Btn_mesh_denoising);

	QGroupBox* mesh_denoising_group = new QGroupBox(tr("Mesh Denoising"));
	mesh_denoising_group->setLayout(mesh_denoising_group_layout);

	Btn_show_mesh_param_1 = new QPushButton(tr("Show Mesh Parameterization 1 Result"));
	Btn_show_source_mesh_1 = new QPushButton(tr("Show Source Mesh"));
	QHBoxLayout* mesh_parameterization_1_group_layout = new QHBoxLayout();
	mesh_parameterization_1_group_layout->addWidget(Btn_show_mesh_param_1);
	mesh_parameterization_1_group_layout->addWidget(Btn_show_source_mesh_1);

	QGroupBox* mesh_parameterization_1_group = new QGroupBox(tr("Mesh Parameterization 1"));
	mesh_parameterization_1_group->setLayout(mesh_parameterization_1_group_layout);

	Text_input_param2_iteration_num = new QTextEdit();
	Btn_show_mesh_param_2 = new QPushButton(tr("Show Mesh Parameterization 2 Result"));
	Btn_show_source_mesh_2 = new QPushButton(tr("Show Source Mesh"));

	Text_input_param2_iteration_num->setFixedHeight(27);

	QHBoxLayout* mesh_parameterization_2_group_layout = new QHBoxLayout();
	mesh_parameterization_2_group_layout->addWidget(Text_input_param2_iteration_num);
	mesh_parameterization_2_group_layout->addWidget(Btn_show_mesh_param_2);
	mesh_parameterization_2_group_layout->addWidget(Btn_show_source_mesh_2);

	QGroupBox* mesh_parameterization_2_group = new QGroupBox(tr("Mesh Parameterization 2"));
	mesh_parameterization_2_group->setLayout(mesh_parameterization_2_group_layout);

	Text_input_arap_modeling_target_vh_idx = new QTextEdit();
	Text_input_arap_modeling_target_pose = new QTextEdit();
	Text_input_arap_modeling_fixed_vh_idx = new QTextEdit();
	Text_input_arap_modeling_target_vh_idx->setFixedHeight(27);
	Text_input_arap_modeling_target_pose->setFixedHeight(27);
	Text_input_arap_modeling_fixed_vh_idx->setFixedHeight(27);
	Btn_update_arap_modeling = new QPushButton(tr("Show ARAP Modeling Result"));

	QHBoxLayout* arap_modeling_group_layout = new QHBoxLayout();
	arap_modeling_group_layout->addWidget(Text_input_arap_modeling_target_vh_idx);
	arap_modeling_group_layout->addWidget(Text_input_arap_modeling_target_pose);
	arap_modeling_group_layout->addWidget(Text_input_arap_modeling_fixed_vh_idx);
	arap_modeling_group_layout->addWidget(Btn_update_arap_modeling);

	QGroupBox* arap_modeling_group = new QGroupBox(tr("ARAP Modeling"));
	arap_modeling_group->setLayout(arap_modeling_group_layout);

	Btn_get_barycentric_coordinates = new QPushButton(tr("Get Barycentric Coordinates"));

	QHBoxLayout* barycentric_coordinates_group_layout = new QHBoxLayout();
	barycentric_coordinates_group_layout->addWidget(Btn_get_barycentric_coordinates);

	QGroupBox* barycentric_coordinates_group = new QGroupBox(tr("Barycentric Coordinates"));
	barycentric_coordinates_group->setLayout(barycentric_coordinates_group_layout);

	Slider_mesh_interpolation = new QSlider();
	Slider_mesh_interpolation->setOrientation(Qt::Horizontal);
	Slider_mesh_interpolation->setMinimum(0);
	Slider_mesh_interpolation->setMaximum(50);
	Slider_mesh_interpolation->setSingleStep(1);

	Text_input_interpolation_method = new QTextEdit();
	Text_input_interpolation_method->setFixedHeight(27);

	Bar_mesh_interpolation = new QProgressBar();
	Bar_mesh_interpolation->setRange(0, 50);
	Bar_mesh_interpolation->setValue(0);
	Bar_mesh_interpolation->setOrientation(Qt::Horizontal);

	Btn_show_interpolation = new QPushButton(tr("Show Interpolation"));
	Btn_save_as_source_mesh = new QPushButton(tr("Save as source mesh"));
	Btn_save_as_target_mesh = new QPushButton(tr("Save as target mesh"));

	QHBoxLayout* mesh_interpolation_slider_group_layout = new QHBoxLayout();
	mesh_interpolation_slider_group_layout->addWidget(Slider_mesh_interpolation);
	mesh_interpolation_slider_group_layout->addWidget(Text_input_interpolation_method);
	mesh_interpolation_slider_group_layout->addWidget(Btn_show_interpolation);

	QGroupBox* mesh_interpolation_slider_group = new QGroupBox();
	mesh_interpolation_slider_group->setLayout(mesh_interpolation_slider_group_layout);

	QHBoxLayout* mesh_save_group_layout = new QHBoxLayout();
	mesh_save_group_layout->addWidget(Bar_mesh_interpolation);
	mesh_save_group_layout->addWidget(Btn_save_as_source_mesh);
	mesh_save_group_layout->addWidget(Btn_save_as_target_mesh);

	QGroupBox* mesh_save_group = new QGroupBox();
	mesh_save_group->setLayout(mesh_save_group_layout);

	QVBoxLayout* mesh_interpolation_group_layout = new QVBoxLayout();
	mesh_interpolation_group_layout->addWidget(mesh_interpolation_slider_group);
	mesh_interpolation_group_layout->addWidget(mesh_save_group);

	QGroupBox* mesh_interpolation_group = new QGroupBox(tr("Mesh Interpolation"));
	mesh_interpolation_group->setLayout(mesh_interpolation_group_layout);

	Text_input_simplification_vertex_num = new QTextEdit();
	Text_input_simplification_vertex_num->setFixedHeight(27);

	Btn_show_simplification_result = new QPushButton(tr("Show Result"));

	QHBoxLayout* mesh_simplification_group_layout = new QHBoxLayout();
	mesh_simplification_group_layout->addWidget(Text_input_simplification_vertex_num);
	mesh_simplification_group_layout->addWidget(Btn_show_simplification_result);

	QGroupBox* mesh_simplification_group = new QGroupBox(tr("Mesh Simplification"));
	mesh_simplification_group->setLayout(mesh_simplification_group_layout);

	Btn_show_cross_fields_result = new QPushButton(tr("Show Result"));

	QHBoxLayout* cross_fields_group_layout = new QHBoxLayout();
	cross_fields_group_layout->addWidget(Btn_show_cross_fields_result);

	QGroupBox* cross_fields_group = new QGroupBox(tr("Cross Fields"));
	cross_fields_group->setLayout(cross_fields_group_layout);

	Text_input_remeshing_target_edge_length = new QTextEdit();
	Text_input_remeshing_target_edge_length->setFixedHeight(27);
	Btn_show_remeshing_result = new QPushButton(tr("Show Result"));

	QHBoxLayout* remeshing_group_layout = new QHBoxLayout();
	remeshing_group_layout->addWidget(Text_input_remeshing_target_edge_length);
	remeshing_group_layout->addWidget(Btn_show_remeshing_result);

	QGroupBox* remeshing_group = new QGroupBox(tr("Remeshing"));
	remeshing_group->setLayout(remeshing_group_layout);

	Text_input_optimal_delaunay_triangulation_solve_num = new QTextEdit();
	Text_input_optimal_delaunay_triangulation_solve_num->setFixedHeight(27);
	Btn_show_optimal_delaunay_triangulation_result = new QPushButton(tr("Show Result"));

	QHBoxLayout* optimal_delaunay_triangulation_group_layout = new QHBoxLayout();
	optimal_delaunay_triangulation_group_layout->addWidget(Text_input_optimal_delaunay_triangulation_solve_num);
	optimal_delaunay_triangulation_group_layout->addWidget(Btn_show_optimal_delaunay_triangulation_result);

	QGroupBox* optimal_delaunay_triangulation_group = new QGroupBox(tr("Optimal Delaunay Triangulation"));
	optimal_delaunay_triangulation_group->setLayout(optimal_delaunay_triangulation_group_layout);

	Text_input_lloyd_iteration_algorithm_solve_num = new QTextEdit();
	Text_input_lloyd_iteration_algorithm_solve_num->setFixedHeight(27);
	Btn_show_lloyd_iteration_algorithm_result = new QPushButton(tr("Show Result"));

	QHBoxLayout* lloyd_iteration_algorithm_group_layout = new QHBoxLayout();
	lloyd_iteration_algorithm_group_layout->addWidget(Text_input_lloyd_iteration_algorithm_solve_num);
	lloyd_iteration_algorithm_group_layout->addWidget(Btn_show_lloyd_iteration_algorithm_result);

	QGroupBox* lloyd_iteration_algorithm_group = new QGroupBox(tr("Lloyd Iteration Algorithm"));
	lloyd_iteration_algorithm_group->setLayout(lloyd_iteration_algorithm_group_layout);

	Text_input_geometric_optimization_iteration_num = new QTextEdit();
	Text_input_geometric_optimization_iteration_num->setFixedHeight(27);

	Btn_show_geometric_optimization_result = new QPushButton(tr("Show Result"));
	Btn_show_source_mesh_geometric_optimization = new QPushButton(tr("Show Source Mesh"));

	QHBoxLayout* geometric_optimization_group_layout = new QHBoxLayout();
	geometric_optimization_group_layout->addWidget(Text_input_geometric_optimization_iteration_num);
	geometric_optimization_group_layout->addWidget(Btn_show_geometric_optimization_result);
	geometric_optimization_group_layout->addWidget(Btn_show_source_mesh_geometric_optimization);

	QGroupBox* geometric_optimization_group = new QGroupBox(tr("Geometric Optimization"));
	geometric_optimization_group->setLayout(geometric_optimization_group_layout);

	QGroupBox* empty_group = new QGroupBox();

	QVBoxLayout *layout = new QVBoxLayout();
	layout->addWidget(pbPrintInfo, 1);

	layout->addWidget(shortest_path_group, 1);
	layout->addWidget(discrete_curvature_btn_group, 1);
	layout->addWidget(mesh_denoising_group, 1);
	layout->addWidget(mesh_parameterization_1_group, 1);
	layout->addWidget(mesh_parameterization_2_group, 1);
	layout->addWidget(arap_modeling_group, 1);
	layout->addWidget(barycentric_coordinates_group, 1);
	layout->addWidget(mesh_interpolation_group, 1);
	layout->addWidget(mesh_simplification_group, 1);
	layout->addWidget(cross_fields_group, 1);
	layout->addWidget(remeshing_group, 1);
	layout->addWidget(optimal_delaunay_triangulation_group, 1);
	layout->addWidget(lloyd_iteration_algorithm_group, 1);
	layout->addWidget(geometric_optimization_group, 1);

	layout->addWidget(empty_group, 10);

	layout->addStretch();
	wParam = new QWidget();
	wParam->setLayout(layout);
	saParam = new QScrollArea();
	saParam->setFocusPolicy(Qt::NoFocus);
	saParam->setFrameStyle(QFrame::NoFrame);
	saParam->setWidget(wParam);
	saParam->setWidgetResizable(true);
}

void MeshParamWidget::CreateLayout(void)
{
	twParam = new QTabWidget();
	twParam->addTab(saParam, "Tab");
	QGridLayout *layout = new QGridLayout();
	layout->addWidget(twParam, 0, 0, 1, 1);
	this->setLayout(layout);
}

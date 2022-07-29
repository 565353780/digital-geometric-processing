#pragma once

#include <QWidget>
#include <QtGui>
#include <QtWidgets>

class MeshParamWidget : public QWidget
{
	Q_OBJECT

public:
	MeshParamWidget(QWidget *parent = 0);
	~MeshParamWidget(void);
private:
	void CreateTabWidget(void);
	void CreateLayout(void);
signals:
	void PrintInfoSignal();
private:
	QTabWidget *twParam;
	QWidget *wParam;
	QScrollArea *saParam;
	QPushButton *pbPrintInfo;

public:
	QTextEdit* Text_input_vh_idx;
	QTextEdit* Text_input_search_num;
	QPushButton* Text_input_vh_btn;

public:
	QPushButton* Btn_mean_curvature;
	QPushButton* Btn_absolute_mean_curvature;
	QPushButton* Btn_gaussian_curvature;

public:
	QTextEdit* Text_input_fh_normal_iteration_num;
	QTextEdit* Text_input_vh_position_iteration_num;
	QTextEdit* Text_input_sigma_space;
	QTextEdit* Text_input_sigma_normal;
	QPushButton* Btn_mesh_denoising;

public:
	QPushButton* Btn_show_mesh_param_1;
	QPushButton* Btn_show_source_mesh_1;

public:
	QTextEdit* Text_input_param2_iteration_num;
	QPushButton* Btn_show_mesh_param_2;
	QPushButton* Btn_show_source_mesh_2;

public:
	QTextEdit* Text_input_arap_modeling_target_vh_idx;
	QTextEdit* Text_input_arap_modeling_target_pose;
	QTextEdit* Text_input_arap_modeling_fixed_vh_idx;
	QPushButton* Btn_update_arap_modeling;

public:
	QPushButton* Btn_get_barycentric_coordinates;

public:
	QSlider* Slider_mesh_interpolation;
	QTextEdit* Text_input_interpolation_method;
	QProgressBar* Bar_mesh_interpolation;
	QPushButton* Btn_show_interpolation;
	QPushButton* Btn_save_as_source_mesh;
	QPushButton* Btn_save_as_target_mesh;

public:
	QTextEdit* Text_input_simplification_vertex_num;
	QPushButton* Btn_show_simplification_result;

public:
	QPushButton* Btn_show_cross_fields_result;

public:
	QTextEdit* Text_input_remeshing_target_edge_length;
	QPushButton* Btn_show_remeshing_result;

public:
	QTextEdit* Text_input_optimal_delaunay_triangulation_solve_num;
	QPushButton* Btn_show_optimal_delaunay_triangulation_result;

public:
	QTextEdit* Text_input_lloyd_iteration_algorithm_solve_num;
	QPushButton* Btn_show_lloyd_iteration_algorithm_result;

public:
	QTextEdit* Text_input_geometric_optimization_iteration_num;
	QPushButton* Btn_show_geometric_optimization_result;
	QPushButton* Btn_show_source_mesh_geometric_optimization;
};

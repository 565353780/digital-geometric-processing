#pragma once
#include <QString>
#include "QGLViewerWidget.h"
#include "MeshDefinition.h"

#include "../chLi/shortest_path_and_minimal_spanning_tree.h"
#include "../chLi/discrete_curvature.h"
#include "../chLi/mesh_denoising.h"
#include "../chLi/mesh_parameterization_1.h"
#include "../chLi/mesh_parameterization_2.h"
#include "../chLi/arap_surface_modeling.h"
#include "../chLi/barycentric_coordinates.h"
#include "../chLi/mesh_interpolation.h"
#include "../chLi/mesh_simplification.h"
#include "../chLi/cross_fields.h"
#include "../chLi/remeshing.h"
#include "../chLi/optimal_delaunay_triangulation.h"
#include "../chLi/Lloyd_iteration_algorithm.h"
#include "../chLi/geometric_optimization.h"

class MeshViewerWidget : public QGLViewerWidget
{
	Q_OBJECT
public:
	MeshViewerWidget(QWidget* parent = 0);
	virtual ~MeshViewerWidget(void);
	bool LoadMesh(const std::string & filename);
	void Clear(void);
	void UpdateMesh(void);
	bool SaveMesh(const std::string & filename);
	bool ScreenShot(void);
	void SetDrawBoundingBox(bool b);
	void SetDrawBoundary(bool b);
	void EnableLighting(bool b);
	void EnableDoubleSide(bool b);
	void ResetView(void);
	void ViewCenter(void);
	void CopyRotation(void);
	void LoadRotation(void);
signals:
	void LoadMeshOKSignal(bool, QString);
public slots:
	void PrintMeshInfo(void);
protected:
	virtual void DrawScene(void) override;
	void DrawSceneMesh(void);

private:
	void DrawPoints(void) const;
	void DrawWireframe(void) const;
	void DrawHiddenLines(void) const;
	void DrawFlatLines(void) const;
	void DrawFlat(void) const;
	void DrawSmooth(void) const;
	void DrawBoundingBox(void) const;
	void DrawBoundary(void) const;
protected:
	Mesh mesh;
	QString strMeshFileName;
	QString strMeshBaseName;
	QString strMeshPath;
	Mesh::Point ptMin;
	Mesh::Point ptMax;
	bool isEnableLighting;
	bool isTwoSideLighting;
	bool isDrawBoundingBox;
	bool isDrawBoundary;

private:
	void DrawShortestPath(void);
protected:
	bool get_Shortest_Path_Set();
	Shortest_Path_And_Minimal_Spanning_Tree* shortest_path_solver;
	std::vector<Shortest_Path> shortest_path_set;
	std::vector<int> vh_idx_set_backup;
	int search_num_backup;
	std::string strMeshFileName_backup;
public:
	std::vector<int> vh_idx_set;
	int search_num;

private:
	void DrawDiscreteCurvature(void);
protected:
	bool get_Color_On_VH();
	Discrete_Curvature* discrete_curvature_solver;
	std::vector<std::vector<double>> vh_color_set;
	int vh_lowest_color_idx = -1;
	SolveMode solve_mode_backup;
public:
	SolveMode solve_mode;

private:
	void DrawMeshDenoising(void);
protected:
	bool get_Denoising_Mesh();
	Mesh_Denoising* mesh_denoising_solver;
	int fh_normal_iteration_num_backup;
	int vh_position_iteration_num_backup;
	double sigma_space_backup;
	double sigma_normal_backup;
public:
	int fh_normal_iteration_num;
	int vh_position_iteration_num;
	double sigma_space;
	double sigma_normal;

private:
	void DrawMeshParameterization1(void);
protected:
	bool get_Mesh_Parameterization_1();
	Mesh_Parameterization_1* mesh_parameterization_1_solver;
	std::vector<std::vector<double>> vh_param_1_position_set;
	bool Show_Parameterization_1_Result_backup;
	bool Param1_Solver_Used;
public:
	bool Show_Parameterization_1_Result;

private:
	void DrawMeshParameterization2(void);
protected:
	bool get_Mesh_Parameterization_2();
	Mesh_Parameterization_2* mesh_parameterization_2_solver;
	std::vector<std::vector<double>> vh_param_2_position_set;
	bool Show_Parameterization_2_Result_backup;
	int param2_iteration_num_backup;
	bool Param2_Solver_Used;
public:
	bool Show_Parameterization_2_Result;
	int param2_iteration_num;

private:
	void DrawARAPSurfaceModeling(void);
protected:
	bool get_ARAP_Surface_Modeling();
	ARAP_Surface_Modeling* arap_surface_modeling_solver;
	std::vector<std::vector<double>> vh_arap_modeling_position_set;
	bool Show_ARAP_Surface_Modeling_Result_backup;
	std::vector<int> target_vh_idx_backup;
	std::vector<std::vector<double>> target_pose_backup;
	std::vector<int> fixed_vh_idx_backup;
public:
	bool Show_ARAP_Surface_Modeling_Result;
	std::vector<int> target_vh_idx;
	std::vector<std::vector<double>> target_pose;
	std::vector<int> fixed_vh_idx;

private:
	void DrawBarycentricCoordinates(void);
protected:
	bool get_Barycentric_Coordinates();
	Barycentric_Coordinates* barycentric_coordinates_solver;
	std::vector<std::vector<double>> vh_barycentric_coordinates_position_set;
	bool Show_Barycentric_Coordinates_Result_backup;
public:
	bool Show_Barycentric_Coordinates_Result;

private:
	void DrawMeshInterpolation(void);
protected:
	bool get_Mesh_Interpolation();
	Mesh_Interpolation* mesh_interpolation_solver;
	std::vector<std::vector<std::vector<double>>> vh_mesh_interpolation_result_set;
	bool Show_Mesh_Interpolation_backup;
	int interpolation_t_backup;
	int interpolation_t_max_backup;
	int interpolation_method_backup;
	bool all_solved;
public:
	bool Show_Mesh_Interpolation;
	int interpolation_t_max;
	int interpolation_t;
	int interpolation_method;
	bool need_to_update_source_mesh;
	bool need_to_update_target_mesh;
signals:
	void mesh_interpolation_process_num_updated(int);

private:
	void DrawMeshSimplification(void);
protected:
	bool get_Mesh_Simplification();
	Mesh_Simplification* mesh_simplification_solver;
	int target_simplification_vh_num_backup;
public:
	int target_simplification_vh_num;

private:
	void DrawCrossFields(void);
protected:
	bool get_Cross_Fields();
	Cross_Fields* cross_fields_solver;
	MatrixXd output;
	int cross_fields_num;

private:
	void DrawRemeshing(void);
protected:
	bool get_Remeshing();
	Remeshing* remeshing_solver;
	double target_remeshing_edge_length_backup;
public:
	double target_remeshing_edge_length;

private:
	void DrawOptimalDelaunayTriangulation(void);
protected:
	bool get_Optimal_Delaunay_Triangulation();
	Optimal_Delaunay_Triangulation* optimal_delaunay_triangulation_solver;
	int target_optimal_delaunay_triangulation_solve_num_backup;
public:
	bool need_to_update_optimal_delaunay_triangulation;
	int target_optimal_delaunay_triangulation_solve_num;

private:
	void DrawLloydIterationAlgorithm(void);
protected:
	bool get_Lloyd_Iteration_Algorithm();
	Lloyd_Iteration_Algorithm* lloyd_iteration_algorithm_solver;
	int target_lloyd_iteration_algorithm_solve_num_backup;
public:
	bool need_to_update_lloyd_iteration_algorithm;
	int target_lloyd_iteration_algorithm_solve_num;

private:
	void DrawGeometricOptimization(void);
protected:
	bool get_Geometric_Optimization();
	Geometric_Optimization* geometric_optimization_solver;
	std::vector<std::vector<double>> vh_geometric_optimization_position_set;
	bool Show_Geometric_Optimization_Result_backup;
	int geometric_optimization_iteration_num_backup;
	bool Geometric_Optimization_Solver_Used;
public:
	bool Show_Geometric_Optimization_Result;
	int geometric_optimization_iteration_num;
};

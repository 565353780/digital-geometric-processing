#pragma once
#include <QtGui>
#include <QString>
#include <QFileDialog>

#include "../chLi/ColorBarWidget.h"

class MeshParamWidget;
class InteractiveViewerWidget;
class MainViewerWidget : public QDialog
{
	Q_OBJECT
public:
	MainViewerWidget(QWidget* _parent = 0);
	~MainViewerWidget(void);

protected:
	virtual void InitViewerWindow(void);
	virtual void CreateParamWidget(void);
	virtual void CreateViewerDialog(void);
	virtual void OpenMeshGUI(const QString & fname);
	virtual void SaveMeshGUI(const QString & fname);

private slots:
	void LoadMeshFromInner(bool OK, QString fname);
public slots:
	void Open(void);
	void Save(void);
	void ClearMesh(void);
	void Screenshot(void);

	void ShowPoints(void);
	void ShowWireframe(void);
	void ShowHiddenLines(void);
	void ShowFlatLines(void);
	void ShowFlat(void);
	void ShowSmooth(void);
	void Lighting(bool b);
	void DoubleSideLighting(bool b);
	void ShowBoundingBox(bool b);
	void ShowBoundary(bool b);
	void ResetView(void);
	void ViewCenter(void);
	void CopyRotation(void);
	void LoadRotation(void);

signals:
	void haveLoadMesh(QString filePath);

protected:
	bool loadmeshsuccess;

private:
	MeshParamWidget* meshparamwidget;
	InteractiveViewerWidget* meshviewerwidget;

public slots:
	void ShowShortestPath(void);
	void GetNumFromText(void);

public slots:
	void ShowDiscreteCurvature(void);
	void SetSolveMeanCurvature(void);
	void SetSolveAbsoluteMeanCurvature(void);
	void SetSolveGaussianCurvature(void);
	void Show_Color_Bar_Widget(void);
protected:
	ColorBarWidget* color_bar_widget;

public slots:
	void ShowMeshDenoising(void);
	void Get4ParamFromText(void);

public slots:
	void ShowMeshParameterization1(void);
	void ShowMeshParam1Result(void);
	void ShowSourceMesh1(void);

public slots:
	void ShowMeshParameterization2(void);
	void ShowMeshParam2Result(void);
	void ShowSourceMesh2(void);

public slots:
	void ShowARAPSurfaceModeling(void);
	void UpdateARAPParams(void);

public slots:
	void ShowBarycentricCoordinates(void);

public slots:
	void ShowMeshInterpolation(void);
	void ShowInterpolationResult(void);
	void SaveAsSourceMesh(void);
	void SaveAsTargetMesh(void);
	void UpdateInterpolationParam(int);
	void UpdateInterpolationProcessBar(int);

public slots:
	void ShowMeshSimplification(void);
	void ShowSimplificationResult(void);

public slots:
	void ShowCrossFields(void);

public slots:
	void ShowRemeshing(void);
	void UpdateRemeshingTargetEdgeLength(void);

public slots:
	void ShowOptimalDelaunayTriangulation(void);
	void UpdateOptimalDelaunayTriangulationSolveNum(void);

public slots:
	void ShowLloydIterationAlgorithm(void);
	void UpdateLloydIterationAlgorithmSolveNum(void);

public slots:
	void ShowGeometricOptimization(void);
	void ShowMeshGeometricOptimizationResult(void);
	void ShowSourceMeshGeometricOptimization(void);
};

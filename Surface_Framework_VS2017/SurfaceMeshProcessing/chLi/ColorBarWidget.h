#pragma once

#include <QWidget>
#include <QtGui>
#include <QtWidgets>

#include <QPainter>

class ColorBarWidget : public QWidget
{
	Q_OBJECT

public:
	ColorBarWidget(QWidget *parent = 0);
	~ColorBarWidget(void);

protected:
	void paintEvent(QPaintEvent* event);
};

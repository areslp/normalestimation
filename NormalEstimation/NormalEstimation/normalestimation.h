#ifndef NORMALESTIMATION_H
#define NORMALESTIMATION_H

//#define EIGEN_USE_MKL_ALL

#include <QtGui/QMainWindow>
#include "ui_normalestimation.h"

class NormalEstimation : public QMainWindow
{
	Q_OBJECT

public:
	NormalEstimation(QWidget *parent = 0, Qt::WFlags flags = 0);
	~NormalEstimation();

public slots: 
	void open(); 

private:
	Ui::NormalEstimationClass ui;
};

#endif // NORMALESTIMATION_H

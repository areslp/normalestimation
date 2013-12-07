#include "normalestimation.h"
#include <QtGui/QApplication>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	NormalEstimation w;
	w.show();
	return a.exec();
}


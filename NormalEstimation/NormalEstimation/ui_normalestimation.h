/********************************************************************************
** Form generated from reading UI file 'normalestimation.ui'
**
** Created: Sun Dec 8 01:21:15 2013
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_NORMALESTIMATION_H
#define UI_NORMALESTIMATION_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QHeaderView>
#include <QtGui/QMainWindow>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QPushButton>
#include <QtGui/QStatusBar>
#include <QtGui/QToolBar>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_NormalEstimationClass
{
public:
    QAction *actionOpen;
    QWidget *centralWidget;
    QPushButton *pushButton;
    QMenuBar *menuBar;
    QMenu *menuFile;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *NormalEstimationClass)
    {
        if (NormalEstimationClass->objectName().isEmpty())
            NormalEstimationClass->setObjectName(QString::fromUtf8("NormalEstimationClass"));
        NormalEstimationClass->resize(600, 400);
        actionOpen = new QAction(NormalEstimationClass);
        actionOpen->setObjectName(QString::fromUtf8("actionOpen"));
        centralWidget = new QWidget(NormalEstimationClass);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        pushButton = new QPushButton(centralWidget);
        pushButton->setObjectName(QString::fromUtf8("pushButton"));
        pushButton->setGeometry(QRect(30, 10, 75, 23));
        NormalEstimationClass->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(NormalEstimationClass);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 600, 23));
        menuFile = new QMenu(menuBar);
        menuFile->setObjectName(QString::fromUtf8("menuFile"));
        NormalEstimationClass->setMenuBar(menuBar);
        mainToolBar = new QToolBar(NormalEstimationClass);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        NormalEstimationClass->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(NormalEstimationClass);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        NormalEstimationClass->setStatusBar(statusBar);

        menuBar->addAction(menuFile->menuAction());

        retranslateUi(NormalEstimationClass);
        QObject::connect(pushButton, SIGNAL(clicked()), NormalEstimationClass, SLOT(open()));

        QMetaObject::connectSlotsByName(NormalEstimationClass);
    } // setupUi

    void retranslateUi(QMainWindow *NormalEstimationClass)
    {
        NormalEstimationClass->setWindowTitle(QApplication::translate("NormalEstimationClass", "NormalEstimation", 0, QApplication::UnicodeUTF8));
        actionOpen->setText(QApplication::translate("NormalEstimationClass", "Open...", 0, QApplication::UnicodeUTF8));
        pushButton->setText(QApplication::translate("NormalEstimationClass", "Open", 0, QApplication::UnicodeUTF8));
        menuFile->setTitle(QApplication::translate("NormalEstimationClass", "File", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class NormalEstimationClass: public Ui_NormalEstimationClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_NORMALESTIMATION_H

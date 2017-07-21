#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtWidgets/QMainWindow>
#include "ui_MainWindow.h"
#include "Canvas.h"

class MainWindow : public QMainWindow {
	Q_OBJECT

public:
	Ui::MainWindowClass ui;
	canvas::Canvas* canvas;

public:
	MainWindow(QWidget *parent = 0);
	~MainWindow();

public slots:
	void onNew();
	void onOpen();
	void onSave();
	void onUndo();
	void onRedo();
	void onCopy();
	void onPaste();
	void onDelete();
	void onSelectAll();
	void onCircularRepeat();
	void onModeChanged();
	void onLayerChanged();
	void onCalculateSolution4RLinkage();
	void onCalculateSolutionSliderCrank();
	void onRun();
	void onRunBackward();
	void onStop();
	void onStepForward();
	void onStepBackward();
	void onCollisionCheck();
	void keyPressEvent(QKeyEvent* e);
	void keyReleaseEvent(QKeyEvent* e);
};

#endif // MAINWINDOW_H

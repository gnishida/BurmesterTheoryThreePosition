#include "LinkageSynthesisOptionDialog.h"

LinkageSynthesisOptionDialog::LinkageSynthesisOptionDialog(QWidget *parent) : QDialog(parent) {
	ui.setupUi(this);

	ui.lineEditNumSamples->setText("1000");
	ui.lineEditStdDev->setText("1");
	ui.checkBoxAvoidBranchDefect->setChecked(true);
	ui.checkBoxRotatableCrank->setChecked(false);

	connect(ui.pushButtonOK, SIGNAL(clicked()), this, SLOT(onOK()));
	connect(ui.pushButtonCancel, SIGNAL(clicked()), this, SLOT(onCancel()));
}

LinkageSynthesisOptionDialog::~LinkageSynthesisOptionDialog() {
}

void LinkageSynthesisOptionDialog::onOK() {
	accept();
}

void LinkageSynthesisOptionDialog::onCancel() {
	reject();
}
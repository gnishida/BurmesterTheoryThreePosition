/********************************************************************************
** Form generated from reading UI file 'LinkageSynthesisOptionDialog.ui'
**
** Created by: Qt User Interface Compiler version 5.6.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_LINKAGESYNTHESISOPTIONDIALOG_H
#define UI_LINKAGESYNTHESISOPTIONDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QDialog>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>

QT_BEGIN_NAMESPACE

class Ui_LinkageSynthesisOptionDialog
{
public:
    QLabel *label;
    QLineEdit *lineEditNumSamples;
    QLabel *label_2;
    QLineEdit *lineEditStdDev;
    QCheckBox *checkBoxAvoidBranchDefect;
    QCheckBox *checkBoxRotatableCrank;
    QPushButton *pushButtonOK;
    QPushButton *pushButtonCancel;

    void setupUi(QDialog *LinkageSynthesisOptionDialog)
    {
        if (LinkageSynthesisOptionDialog->objectName().isEmpty())
            LinkageSynthesisOptionDialog->setObjectName(QStringLiteral("LinkageSynthesisOptionDialog"));
        LinkageSynthesisOptionDialog->resize(231, 171);
        label = new QLabel(LinkageSynthesisOptionDialog);
        label->setObjectName(QStringLiteral("label"));
        label->setGeometry(QRect(20, 10, 61, 21));
        lineEditNumSamples = new QLineEdit(LinkageSynthesisOptionDialog);
        lineEditNumSamples->setObjectName(QStringLiteral("lineEditNumSamples"));
        lineEditNumSamples->setGeometry(QRect(100, 10, 101, 20));
        label_2 = new QLabel(LinkageSynthesisOptionDialog);
        label_2->setObjectName(QStringLiteral("label_2"));
        label_2->setGeometry(QRect(20, 40, 61, 21));
        lineEditStdDev = new QLineEdit(LinkageSynthesisOptionDialog);
        lineEditStdDev->setObjectName(QStringLiteral("lineEditStdDev"));
        lineEditStdDev->setGeometry(QRect(100, 40, 101, 20));
        checkBoxAvoidBranchDefect = new QCheckBox(LinkageSynthesisOptionDialog);
        checkBoxAvoidBranchDefect->setObjectName(QStringLiteral("checkBoxAvoidBranchDefect"));
        checkBoxAvoidBranchDefect->setGeometry(QRect(20, 70, 131, 17));
        checkBoxRotatableCrank = new QCheckBox(LinkageSynthesisOptionDialog);
        checkBoxRotatableCrank->setObjectName(QStringLiteral("checkBoxRotatableCrank"));
        checkBoxRotatableCrank->setGeometry(QRect(20, 90, 131, 17));
        pushButtonOK = new QPushButton(LinkageSynthesisOptionDialog);
        pushButtonOK->setObjectName(QStringLiteral("pushButtonOK"));
        pushButtonOK->setGeometry(QRect(10, 130, 91, 31));
        pushButtonCancel = new QPushButton(LinkageSynthesisOptionDialog);
        pushButtonCancel->setObjectName(QStringLiteral("pushButtonCancel"));
        pushButtonCancel->setGeometry(QRect(130, 130, 91, 31));

        retranslateUi(LinkageSynthesisOptionDialog);

        QMetaObject::connectSlotsByName(LinkageSynthesisOptionDialog);
    } // setupUi

    void retranslateUi(QDialog *LinkageSynthesisOptionDialog)
    {
        LinkageSynthesisOptionDialog->setWindowTitle(QApplication::translate("LinkageSynthesisOptionDialog", "LinkageSynthesisOptionDialog", 0));
        label->setText(QApplication::translate("LinkageSynthesisOptionDialog", "# samples:", 0));
        label_2->setText(QApplication::translate("LinkageSynthesisOptionDialog", "Stdev:", 0));
        checkBoxAvoidBranchDefect->setText(QApplication::translate("LinkageSynthesisOptionDialog", "Avoid branch defect", 0));
        checkBoxRotatableCrank->setText(QApplication::translate("LinkageSynthesisOptionDialog", "Fully rotatable crank", 0));
        pushButtonOK->setText(QApplication::translate("LinkageSynthesisOptionDialog", "OK", 0));
        pushButtonCancel->setText(QApplication::translate("LinkageSynthesisOptionDialog", "Cancel", 0));
    } // retranslateUi

};

namespace Ui {
    class LinkageSynthesisOptionDialog: public Ui_LinkageSynthesisOptionDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_LINKAGESYNTHESISOPTIONDIALOG_H

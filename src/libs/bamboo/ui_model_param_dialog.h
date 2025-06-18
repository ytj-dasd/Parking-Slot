/********************************************************************************
** Form generated from reading UI file 'model_param_dialog.ui'
**
** Created by: Qt User Interface Compiler version 5.14.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MODEL_PARAM_DIALOG_H
#define UI_MODEL_PARAM_DIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDialog>
#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QSpacerItem>

QT_BEGIN_NAMESPACE

class Ui_ModelParamDialog
{
public:
    QGridLayout *gridLayout;
    QDialogButtonBox *buttonBox;
    QSpacerItem *horizontalSpacer;
    QGroupBox *groupBox_5;
    QGridLayout *gridLayout_10;
    QLabel *label_10;
    QComboBox *comboBox_model_colormode;
    QLabel *label_11;
    QDoubleSpinBox *doubleSpinBox_model_min_ratio;
    QLabel *label_12;
    QDoubleSpinBox *doubleSpinBox_model_max_ratio;

    void setupUi(QDialog *ModelParamDialog)
    {
        if (ModelParamDialog->objectName().isEmpty())
            ModelParamDialog->setObjectName(QString::fromUtf8("ModelParamDialog"));
        ModelParamDialog->resize(222, 165);
        gridLayout = new QGridLayout(ModelParamDialog);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        buttonBox = new QDialogButtonBox(ModelParamDialog);
        buttonBox->setObjectName(QString::fromUtf8("buttonBox"));
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(buttonBox->sizePolicy().hasHeightForWidth());
        buttonBox->setSizePolicy(sizePolicy);
        buttonBox->setOrientation(Qt::Horizontal);
        buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Ok);

        gridLayout->addWidget(buttonBox, 1, 1, 1, 1);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        gridLayout->addItem(horizontalSpacer, 1, 0, 1, 1);

        groupBox_5 = new QGroupBox(ModelParamDialog);
        groupBox_5->setObjectName(QString::fromUtf8("groupBox_5"));
        gridLayout_10 = new QGridLayout(groupBox_5);
        gridLayout_10->setObjectName(QString::fromUtf8("gridLayout_10"));
        label_10 = new QLabel(groupBox_5);
        label_10->setObjectName(QString::fromUtf8("label_10"));
        QSizePolicy sizePolicy1(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(label_10->sizePolicy().hasHeightForWidth());
        label_10->setSizePolicy(sizePolicy1);
        label_10->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        gridLayout_10->addWidget(label_10, 0, 0, 1, 1);

        comboBox_model_colormode = new QComboBox(groupBox_5);
        comboBox_model_colormode->setObjectName(QString::fromUtf8("comboBox_model_colormode"));
        QSizePolicy sizePolicy2(QSizePolicy::Expanding, QSizePolicy::Fixed);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(comboBox_model_colormode->sizePolicy().hasHeightForWidth());
        comboBox_model_colormode->setSizePolicy(sizePolicy2);

        gridLayout_10->addWidget(comboBox_model_colormode, 0, 1, 1, 2);

        label_11 = new QLabel(groupBox_5);
        label_11->setObjectName(QString::fromUtf8("label_11"));
        QSizePolicy sizePolicy3(QSizePolicy::Fixed, QSizePolicy::Preferred);
        sizePolicy3.setHorizontalStretch(0);
        sizePolicy3.setVerticalStretch(0);
        sizePolicy3.setHeightForWidth(label_11->sizePolicy().hasHeightForWidth());
        label_11->setSizePolicy(sizePolicy3);
        label_11->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        gridLayout_10->addWidget(label_11, 1, 0, 1, 2);

        doubleSpinBox_model_min_ratio = new QDoubleSpinBox(groupBox_5);
        doubleSpinBox_model_min_ratio->setObjectName(QString::fromUtf8("doubleSpinBox_model_min_ratio"));
        sizePolicy2.setHeightForWidth(doubleSpinBox_model_min_ratio->sizePolicy().hasHeightForWidth());
        doubleSpinBox_model_min_ratio->setSizePolicy(sizePolicy2);
        doubleSpinBox_model_min_ratio->setDecimals(2);
        doubleSpinBox_model_min_ratio->setMinimum(0.000000000000000);
        doubleSpinBox_model_min_ratio->setMaximum(1.000000000000000);
        doubleSpinBox_model_min_ratio->setSingleStep(0.050000000000000);
        doubleSpinBox_model_min_ratio->setValue(0.050000000000000);

        gridLayout_10->addWidget(doubleSpinBox_model_min_ratio, 1, 2, 1, 1);

        label_12 = new QLabel(groupBox_5);
        label_12->setObjectName(QString::fromUtf8("label_12"));
        sizePolicy3.setHeightForWidth(label_12->sizePolicy().hasHeightForWidth());
        label_12->setSizePolicy(sizePolicy3);
        label_12->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        gridLayout_10->addWidget(label_12, 2, 0, 1, 2);

        doubleSpinBox_model_max_ratio = new QDoubleSpinBox(groupBox_5);
        doubleSpinBox_model_max_ratio->setObjectName(QString::fromUtf8("doubleSpinBox_model_max_ratio"));
        sizePolicy2.setHeightForWidth(doubleSpinBox_model_max_ratio->sizePolicy().hasHeightForWidth());
        doubleSpinBox_model_max_ratio->setSizePolicy(sizePolicy2);
        doubleSpinBox_model_max_ratio->setDecimals(2);
        doubleSpinBox_model_max_ratio->setMinimum(0.000000000000000);
        doubleSpinBox_model_max_ratio->setMaximum(1.000000000000000);
        doubleSpinBox_model_max_ratio->setSingleStep(0.050000000000000);
        doubleSpinBox_model_max_ratio->setValue(0.900000000000000);

        gridLayout_10->addWidget(doubleSpinBox_model_max_ratio, 2, 2, 1, 1);


        gridLayout->addWidget(groupBox_5, 0, 0, 1, 2);


        retranslateUi(ModelParamDialog);
        QObject::connect(buttonBox, SIGNAL(accepted()), ModelParamDialog, SLOT(accept()));
        QObject::connect(buttonBox, SIGNAL(rejected()), ModelParamDialog, SLOT(reject()));

        QMetaObject::connectSlotsByName(ModelParamDialog);
    } // setupUi

    void retranslateUi(QDialog *ModelParamDialog)
    {
        ModelParamDialog->setWindowTitle(QCoreApplication::translate("ModelParamDialog", "Dialog", nullptr));
        groupBox_5->setTitle(QString());
        label_10->setText(QCoreApplication::translate("ModelParamDialog", "color_mode:", nullptr));
        label_11->setText(QCoreApplication::translate("ModelParamDialog", "min_ratio:", nullptr));
        label_12->setText(QCoreApplication::translate("ModelParamDialog", "max_ratio:", nullptr));
    } // retranslateUi

};

namespace Ui {
    class ModelParamDialog: public Ui_ModelParamDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MODEL_PARAM_DIALOG_H

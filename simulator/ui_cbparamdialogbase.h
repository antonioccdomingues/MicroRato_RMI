/********************************************************************************
** Form generated from reading UI file 'cbparamdialogbase.ui'
**
** Created by: Qt User Interface Compiler version 4.8.7
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_CBPARAMDIALOGBASE_H
#define UI_CBPARAMDIALOGBASE_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QDialog>
#include <QtGui/QDialogButtonBox>
#include <QtGui/QFormLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QTabWidget>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_cbParamDialogBase
{
public:
    QVBoxLayout *verticalLayout;
    QTabWidget *tabWidget;
    QWidget *tab_1;
    QFormLayout *formLayout;
    QLabel *label;
    QLineEdit *lnEditSimTime;
    QLabel *label_2;
    QLabel *label_3;
    QLineEdit *lnEditKeyTime;
    QLineEdit *lnEditSimStep;
    QWidget *tab_2;
    QFormLayout *formLayout_2;
    QLabel *label_4;
    QLineEdit *lnEditObstacleNoise;
    QLabel *label_5;
    QLineEdit *lnEditCompassNoise;
    QLabel *label_6;
    QLineEdit *lnEditBeaconNoise;
    QLabel *label_7;
    QLineEdit *lnEditMotorsNoise;
    QLabel *label_8;
    QLineEdit *lnEditGPSLinNoise;
    QLabel *label_9;
    QLineEdit *lnEditGPSDirNoise;
    QWidget *tab_3;
    QFormLayout *formLayout_3;
    QLabel *label_10;
    QLineEdit *lnEditReqPCycle;
    QLabel *label_11;
    QCheckBox *cBoxReqIR;
    QCheckBox *cBoxReqBeacon;
    QCheckBox *cBoxReqGround;
    QCheckBox *cBoxReqCompass;
    QCheckBox *cBoxReqCollision;
    QCheckBox *cBoxReqGPS;
    QWidget *tab_4;
    QFormLayout *formLayout_4;
    QLabel *label_12;
    QLineEdit *lnEditLatIR;
    QLabel *label_13;
    QLineEdit *lnEditLatBeacon;
    QLabel *label_14;
    QLineEdit *lnEditLatCompass;
    QLabel *label_15;
    QLineEdit *lnEditLatGround;
    QLabel *label_16;
    QLineEdit *lnEditLatCollision;
    QLabel *label_17;
    QLineEdit *lnEditLatGPS;
    QWidget *tab_5;
    QFormLayout *formLayout_5;
    QLabel *label_18;
    QLineEdit *lineEdit_ReturnPenalty;
    QLabel *label_19;
    QLineEdit *lineEdit_ArrivalPenalty;
    QLabel *label_20;
    QLineEdit *lineEdit_WallCollPenalty;
    QLabel *label_21;
    QLineEdit *lineEdit_RobotCollPenalty;
    QLabel *label_22;
    QLineEdit *lineEdit_TargetReward;
    QLabel *label_23;
    QLineEdit *lineEdit_HomeReward;
    QDialogButtonBox *buttonBox;

    void setupUi(QDialog *cbParamDialogBase)
    {
        if (cbParamDialogBase->objectName().isEmpty())
            cbParamDialogBase->setObjectName(QString::fromUtf8("cbParamDialogBase"));
        cbParamDialogBase->resize(313, 269);
        QIcon icon;
        icon.addFile(QString::fromUtf8(":/images/buttons/config"), QSize(), QIcon::Normal, QIcon::Off);
        cbParamDialogBase->setWindowIcon(icon);
        verticalLayout = new QVBoxLayout(cbParamDialogBase);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        tabWidget = new QTabWidget(cbParamDialogBase);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        tab_1 = new QWidget();
        tab_1->setObjectName(QString::fromUtf8("tab_1"));
        formLayout = new QFormLayout(tab_1);
        formLayout->setObjectName(QString::fromUtf8("formLayout"));
        formLayout->setLabelAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        formLayout->setFormAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        label = new QLabel(tab_1);
        label->setObjectName(QString::fromUtf8("label"));

        formLayout->setWidget(0, QFormLayout::LabelRole, label);

        lnEditSimTime = new QLineEdit(tab_1);
        lnEditSimTime->setObjectName(QString::fromUtf8("lnEditSimTime"));

        formLayout->setWidget(0, QFormLayout::FieldRole, lnEditSimTime);

        label_2 = new QLabel(tab_1);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        formLayout->setWidget(1, QFormLayout::LabelRole, label_2);

        label_3 = new QLabel(tab_1);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        formLayout->setWidget(2, QFormLayout::LabelRole, label_3);

        lnEditKeyTime = new QLineEdit(tab_1);
        lnEditKeyTime->setObjectName(QString::fromUtf8("lnEditKeyTime"));

        formLayout->setWidget(1, QFormLayout::FieldRole, lnEditKeyTime);

        lnEditSimStep = new QLineEdit(tab_1);
        lnEditSimStep->setObjectName(QString::fromUtf8("lnEditSimStep"));

        formLayout->setWidget(2, QFormLayout::FieldRole, lnEditSimStep);

        tabWidget->addTab(tab_1, QString());
        tab_2 = new QWidget();
        tab_2->setObjectName(QString::fromUtf8("tab_2"));
        formLayout_2 = new QFormLayout(tab_2);
        formLayout_2->setObjectName(QString::fromUtf8("formLayout_2"));
        formLayout_2->setLabelAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        formLayout_2->setFormAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        label_4 = new QLabel(tab_2);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        formLayout_2->setWidget(0, QFormLayout::LabelRole, label_4);

        lnEditObstacleNoise = new QLineEdit(tab_2);
        lnEditObstacleNoise->setObjectName(QString::fromUtf8("lnEditObstacleNoise"));

        formLayout_2->setWidget(0, QFormLayout::FieldRole, lnEditObstacleNoise);

        label_5 = new QLabel(tab_2);
        label_5->setObjectName(QString::fromUtf8("label_5"));

        formLayout_2->setWidget(1, QFormLayout::LabelRole, label_5);

        lnEditCompassNoise = new QLineEdit(tab_2);
        lnEditCompassNoise->setObjectName(QString::fromUtf8("lnEditCompassNoise"));

        formLayout_2->setWidget(1, QFormLayout::FieldRole, lnEditCompassNoise);

        label_6 = new QLabel(tab_2);
        label_6->setObjectName(QString::fromUtf8("label_6"));

        formLayout_2->setWidget(2, QFormLayout::LabelRole, label_6);

        lnEditBeaconNoise = new QLineEdit(tab_2);
        lnEditBeaconNoise->setObjectName(QString::fromUtf8("lnEditBeaconNoise"));

        formLayout_2->setWidget(2, QFormLayout::FieldRole, lnEditBeaconNoise);

        label_7 = new QLabel(tab_2);
        label_7->setObjectName(QString::fromUtf8("label_7"));

        formLayout_2->setWidget(3, QFormLayout::LabelRole, label_7);

        lnEditMotorsNoise = new QLineEdit(tab_2);
        lnEditMotorsNoise->setObjectName(QString::fromUtf8("lnEditMotorsNoise"));

        formLayout_2->setWidget(3, QFormLayout::FieldRole, lnEditMotorsNoise);

        label_8 = new QLabel(tab_2);
        label_8->setObjectName(QString::fromUtf8("label_8"));

        formLayout_2->setWidget(4, QFormLayout::LabelRole, label_8);

        lnEditGPSLinNoise = new QLineEdit(tab_2);
        lnEditGPSLinNoise->setObjectName(QString::fromUtf8("lnEditGPSLinNoise"));

        formLayout_2->setWidget(4, QFormLayout::FieldRole, lnEditGPSLinNoise);

        label_9 = new QLabel(tab_2);
        label_9->setObjectName(QString::fromUtf8("label_9"));

        formLayout_2->setWidget(5, QFormLayout::LabelRole, label_9);

        lnEditGPSDirNoise = new QLineEdit(tab_2);
        lnEditGPSDirNoise->setObjectName(QString::fromUtf8("lnEditGPSDirNoise"));

        formLayout_2->setWidget(5, QFormLayout::FieldRole, lnEditGPSDirNoise);

        tabWidget->addTab(tab_2, QString());
        tab_3 = new QWidget();
        tab_3->setObjectName(QString::fromUtf8("tab_3"));
        formLayout_3 = new QFormLayout(tab_3);
        formLayout_3->setObjectName(QString::fromUtf8("formLayout_3"));
        formLayout_3->setLabelAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        formLayout_3->setFormAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        label_10 = new QLabel(tab_3);
        label_10->setObjectName(QString::fromUtf8("label_10"));

        formLayout_3->setWidget(0, QFormLayout::LabelRole, label_10);

        lnEditReqPCycle = new QLineEdit(tab_3);
        lnEditReqPCycle->setObjectName(QString::fromUtf8("lnEditReqPCycle"));

        formLayout_3->setWidget(0, QFormLayout::FieldRole, lnEditReqPCycle);

        label_11 = new QLabel(tab_3);
        label_11->setObjectName(QString::fromUtf8("label_11"));

        formLayout_3->setWidget(1, QFormLayout::LabelRole, label_11);

        cBoxReqIR = new QCheckBox(tab_3);
        cBoxReqIR->setObjectName(QString::fromUtf8("cBoxReqIR"));

        formLayout_3->setWidget(2, QFormLayout::LabelRole, cBoxReqIR);

        cBoxReqBeacon = new QCheckBox(tab_3);
        cBoxReqBeacon->setObjectName(QString::fromUtf8("cBoxReqBeacon"));

        formLayout_3->setWidget(3, QFormLayout::LabelRole, cBoxReqBeacon);

        cBoxReqGround = new QCheckBox(tab_3);
        cBoxReqGround->setObjectName(QString::fromUtf8("cBoxReqGround"));

        formLayout_3->setWidget(4, QFormLayout::LabelRole, cBoxReqGround);

        cBoxReqCompass = new QCheckBox(tab_3);
        cBoxReqCompass->setObjectName(QString::fromUtf8("cBoxReqCompass"));

        formLayout_3->setWidget(5, QFormLayout::LabelRole, cBoxReqCompass);

        cBoxReqCollision = new QCheckBox(tab_3);
        cBoxReqCollision->setObjectName(QString::fromUtf8("cBoxReqCollision"));

        formLayout_3->setWidget(6, QFormLayout::LabelRole, cBoxReqCollision);

        cBoxReqGPS = new QCheckBox(tab_3);
        cBoxReqGPS->setObjectName(QString::fromUtf8("cBoxReqGPS"));

        formLayout_3->setWidget(7, QFormLayout::LabelRole, cBoxReqGPS);

        tabWidget->addTab(tab_3, QString());
        tab_4 = new QWidget();
        tab_4->setObjectName(QString::fromUtf8("tab_4"));
        formLayout_4 = new QFormLayout(tab_4);
        formLayout_4->setObjectName(QString::fromUtf8("formLayout_4"));
        formLayout_4->setLabelAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        formLayout_4->setFormAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        label_12 = new QLabel(tab_4);
        label_12->setObjectName(QString::fromUtf8("label_12"));

        formLayout_4->setWidget(0, QFormLayout::LabelRole, label_12);

        lnEditLatIR = new QLineEdit(tab_4);
        lnEditLatIR->setObjectName(QString::fromUtf8("lnEditLatIR"));

        formLayout_4->setWidget(0, QFormLayout::FieldRole, lnEditLatIR);

        label_13 = new QLabel(tab_4);
        label_13->setObjectName(QString::fromUtf8("label_13"));

        formLayout_4->setWidget(1, QFormLayout::LabelRole, label_13);

        lnEditLatBeacon = new QLineEdit(tab_4);
        lnEditLatBeacon->setObjectName(QString::fromUtf8("lnEditLatBeacon"));

        formLayout_4->setWidget(1, QFormLayout::FieldRole, lnEditLatBeacon);

        label_14 = new QLabel(tab_4);
        label_14->setObjectName(QString::fromUtf8("label_14"));

        formLayout_4->setWidget(2, QFormLayout::LabelRole, label_14);

        lnEditLatCompass = new QLineEdit(tab_4);
        lnEditLatCompass->setObjectName(QString::fromUtf8("lnEditLatCompass"));

        formLayout_4->setWidget(2, QFormLayout::FieldRole, lnEditLatCompass);

        label_15 = new QLabel(tab_4);
        label_15->setObjectName(QString::fromUtf8("label_15"));

        formLayout_4->setWidget(3, QFormLayout::LabelRole, label_15);

        lnEditLatGround = new QLineEdit(tab_4);
        lnEditLatGround->setObjectName(QString::fromUtf8("lnEditLatGround"));

        formLayout_4->setWidget(3, QFormLayout::FieldRole, lnEditLatGround);

        label_16 = new QLabel(tab_4);
        label_16->setObjectName(QString::fromUtf8("label_16"));

        formLayout_4->setWidget(4, QFormLayout::LabelRole, label_16);

        lnEditLatCollision = new QLineEdit(tab_4);
        lnEditLatCollision->setObjectName(QString::fromUtf8("lnEditLatCollision"));

        formLayout_4->setWidget(4, QFormLayout::FieldRole, lnEditLatCollision);

        label_17 = new QLabel(tab_4);
        label_17->setObjectName(QString::fromUtf8("label_17"));

        formLayout_4->setWidget(5, QFormLayout::LabelRole, label_17);

        lnEditLatGPS = new QLineEdit(tab_4);
        lnEditLatGPS->setObjectName(QString::fromUtf8("lnEditLatGPS"));

        formLayout_4->setWidget(5, QFormLayout::FieldRole, lnEditLatGPS);

        tabWidget->addTab(tab_4, QString());
        tab_5 = new QWidget();
        tab_5->setObjectName(QString::fromUtf8("tab_5"));
        formLayout_5 = new QFormLayout(tab_5);
        formLayout_5->setObjectName(QString::fromUtf8("formLayout_5"));
        formLayout_5->setLabelAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        formLayout_5->setFormAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        label_18 = new QLabel(tab_5);
        label_18->setObjectName(QString::fromUtf8("label_18"));

        formLayout_5->setWidget(0, QFormLayout::LabelRole, label_18);

        lineEdit_ReturnPenalty = new QLineEdit(tab_5);
        lineEdit_ReturnPenalty->setObjectName(QString::fromUtf8("lineEdit_ReturnPenalty"));

        formLayout_5->setWidget(0, QFormLayout::FieldRole, lineEdit_ReturnPenalty);

        label_19 = new QLabel(tab_5);
        label_19->setObjectName(QString::fromUtf8("label_19"));

        formLayout_5->setWidget(1, QFormLayout::LabelRole, label_19);

        lineEdit_ArrivalPenalty = new QLineEdit(tab_5);
        lineEdit_ArrivalPenalty->setObjectName(QString::fromUtf8("lineEdit_ArrivalPenalty"));

        formLayout_5->setWidget(1, QFormLayout::FieldRole, lineEdit_ArrivalPenalty);

        label_20 = new QLabel(tab_5);
        label_20->setObjectName(QString::fromUtf8("label_20"));

        formLayout_5->setWidget(2, QFormLayout::LabelRole, label_20);

        lineEdit_WallCollPenalty = new QLineEdit(tab_5);
        lineEdit_WallCollPenalty->setObjectName(QString::fromUtf8("lineEdit_WallCollPenalty"));

        formLayout_5->setWidget(2, QFormLayout::FieldRole, lineEdit_WallCollPenalty);

        label_21 = new QLabel(tab_5);
        label_21->setObjectName(QString::fromUtf8("label_21"));

        formLayout_5->setWidget(3, QFormLayout::LabelRole, label_21);

        lineEdit_RobotCollPenalty = new QLineEdit(tab_5);
        lineEdit_RobotCollPenalty->setObjectName(QString::fromUtf8("lineEdit_RobotCollPenalty"));

        formLayout_5->setWidget(3, QFormLayout::FieldRole, lineEdit_RobotCollPenalty);

        label_22 = new QLabel(tab_5);
        label_22->setObjectName(QString::fromUtf8("label_22"));

        formLayout_5->setWidget(4, QFormLayout::LabelRole, label_22);

        lineEdit_TargetReward = new QLineEdit(tab_5);
        lineEdit_TargetReward->setObjectName(QString::fromUtf8("lineEdit_TargetReward"));

        formLayout_5->setWidget(4, QFormLayout::FieldRole, lineEdit_TargetReward);

        label_23 = new QLabel(tab_5);
        label_23->setObjectName(QString::fromUtf8("label_23"));

        formLayout_5->setWidget(5, QFormLayout::LabelRole, label_23);

        lineEdit_HomeReward = new QLineEdit(tab_5);
        lineEdit_HomeReward->setObjectName(QString::fromUtf8("lineEdit_HomeReward"));

        formLayout_5->setWidget(5, QFormLayout::FieldRole, lineEdit_HomeReward);

        tabWidget->addTab(tab_5, QString());

        verticalLayout->addWidget(tabWidget);

        buttonBox = new QDialogButtonBox(cbParamDialogBase);
        buttonBox->setObjectName(QString::fromUtf8("buttonBox"));
        buttonBox->setOrientation(Qt::Horizontal);
        buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Ok|QDialogButtonBox::RestoreDefaults);

        verticalLayout->addWidget(buttonBox);


        retranslateUi(cbParamDialogBase);
        QObject::connect(buttonBox, SIGNAL(accepted()), cbParamDialogBase, SLOT(accept()));
        QObject::connect(buttonBox, SIGNAL(rejected()), cbParamDialogBase, SLOT(reject()));

        tabWidget->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(cbParamDialogBase);
    } // setupUi

    void retranslateUi(QDialog *cbParamDialogBase)
    {
        cbParamDialogBase->setWindowTitle(QApplication::translate("cbParamDialogBase", "Parameters Configuration", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("cbParamDialogBase", "Simulation Time (0 = unlimited)", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("cbParamDialogBase", "Deadline for Free Run", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("cbParamDialogBase", "Simulation Step (ms)", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(tab_1), QApplication::translate("cbParamDialogBase", "Time", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("cbParamDialogBase", "Obstacle Sensors Noise", 0, QApplication::UnicodeUTF8));
        label_5->setText(QApplication::translate("cbParamDialogBase", "Compass Noise", 0, QApplication::UnicodeUTF8));
        label_6->setText(QApplication::translate("cbParamDialogBase", "Beacon Sensor Noise", 0, QApplication::UnicodeUTF8));
        label_7->setText(QApplication::translate("cbParamDialogBase", "Motors Noise", 0, QApplication::UnicodeUTF8));
        label_8->setText(QApplication::translate("cbParamDialogBase", "GPS Linear Noise", 0, QApplication::UnicodeUTF8));
        label_9->setText(QApplication::translate("cbParamDialogBase", "GPS Dir Noise", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(tab_2), QApplication::translate("cbParamDialogBase", "Noise", 0, QApplication::UnicodeUTF8));
        label_10->setText(QApplication::translate("cbParamDialogBase", "Request per cycle", 0, QApplication::UnicodeUTF8));
        label_11->setText(QApplication::translate("cbParamDialogBase", "Requestable Sensors:", 0, QApplication::UnicodeUTF8));
        cBoxReqIR->setText(QApplication::translate("cbParamDialogBase", "Obstacle Sensors", 0, QApplication::UnicodeUTF8));
        cBoxReqBeacon->setText(QApplication::translate("cbParamDialogBase", "Beacon Sensors", 0, QApplication::UnicodeUTF8));
        cBoxReqGround->setText(QApplication::translate("cbParamDialogBase", "Ground", 0, QApplication::UnicodeUTF8));
        cBoxReqCompass->setText(QApplication::translate("cbParamDialogBase", "Compass", 0, QApplication::UnicodeUTF8));
        cBoxReqCollision->setText(QApplication::translate("cbParamDialogBase", "Collision", 0, QApplication::UnicodeUTF8));
        cBoxReqGPS->setText(QApplication::translate("cbParamDialogBase", "GPS", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(tab_3), QApplication::translate("cbParamDialogBase", "Requests", 0, QApplication::UnicodeUTF8));
        label_12->setText(QApplication::translate("cbParamDialogBase", "Obstacle Sensors", 0, QApplication::UnicodeUTF8));
        label_13->setText(QApplication::translate("cbParamDialogBase", "Beacon Sensors", 0, QApplication::UnicodeUTF8));
        label_14->setText(QApplication::translate("cbParamDialogBase", "Compass", 0, QApplication::UnicodeUTF8));
        label_15->setText(QApplication::translate("cbParamDialogBase", "Ground", 0, QApplication::UnicodeUTF8));
        label_16->setText(QApplication::translate("cbParamDialogBase", "Collision", 0, QApplication::UnicodeUTF8));
        label_17->setText(QApplication::translate("cbParamDialogBase", "GPS", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(tab_4), QApplication::translate("cbParamDialogBase", "Latency", 0, QApplication::UnicodeUTF8));
        label_18->setText(QApplication::translate("cbParamDialogBase", "Return Time Penalty", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        lineEdit_ReturnPenalty->setToolTip(QApplication::translate("cbParamDialogBase", "Maximum time allowed to exceed return trip time", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        label_19->setText(QApplication::translate("cbParamDialogBase", "Arrival Time Penaty", 0, QApplication::UnicodeUTF8));
        label_20->setText(QApplication::translate("cbParamDialogBase", "Wall Collision Penalty", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        lineEdit_WallCollPenalty->setToolTip(QApplication::translate("cbParamDialogBase", "Points added when Robot collides with a Wall", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        label_21->setText(QApplication::translate("cbParamDialogBase", "Robot Collision Penalty", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        lineEdit_RobotCollPenalty->setToolTip(QApplication::translate("cbParamDialogBase", "Points added when Robot collides with another Robot", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        label_22->setText(QApplication::translate("cbParamDialogBase", "At Target Reward", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        lineEdit_TargetReward->setToolTip(QApplication::translate("cbParamDialogBase", "Points subtracted when Robot reaches a Target Area", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        label_23->setText(QApplication::translate("cbParamDialogBase", "At Home Reward", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        lineEdit_HomeReward->setToolTip(QApplication::translate("cbParamDialogBase", "Points subtracted when Robot reaches Home Area", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        tabWidget->setTabText(tabWidget->indexOf(tab_5), QApplication::translate("cbParamDialogBase", "Scores", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class cbParamDialogBase: public Ui_cbParamDialogBase {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_CBPARAMDIALOGBASE_H

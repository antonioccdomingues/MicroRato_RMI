/********************************************************************************
** Form generated from reading UI file 'cbcontrolpanel.ui'
**
** Created by: Qt User Interface Compiler version 4.8.7
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_CBCONTROLPANEL_H
#define UI_CBCONTROLPANEL_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QFrame>
#include <QtGui/QGridLayout>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QPushButton>
#include <QtGui/QSpacerItem>
#include <QtGui/QSpinBox>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_cbControlPanel
{
public:
    QGridLayout *gridLayout;
    QHBoxLayout *horizontalLayout_1;
    QPushButton *pushButton_Reset;
    QPushButton *pushButton_StartStop;
    QFrame *line_1;
    QSpinBox *spinBox_Robot;
    QPushButton *pushButton_Remove;
    QFrame *line_2;
    QPushButton *pushButton_Time;
    QPushButton *pushButton_Collisions;
    QSpacerItem *horizontalSpacer;
    QPushButton *pushButton_Lock;
    QPushButton *pushButton_ManageRobots;

    void setupUi(QWidget *cbControlPanel)
    {
        if (cbControlPanel->objectName().isEmpty())
            cbControlPanel->setObjectName(QString::fromUtf8("cbControlPanel"));
        cbControlPanel->resize(330, 42);
        cbControlPanel->setMaximumSize(QSize(16777215, 356));
        gridLayout = new QGridLayout(cbControlPanel);
        gridLayout->setContentsMargins(4, 4, 4, 4);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        gridLayout->setHorizontalSpacing(4);
        gridLayout->setVerticalSpacing(0);
        horizontalLayout_1 = new QHBoxLayout();
        horizontalLayout_1->setSpacing(2);
        horizontalLayout_1->setObjectName(QString::fromUtf8("horizontalLayout_1"));
        pushButton_Reset = new QPushButton(cbControlPanel);
        pushButton_Reset->setObjectName(QString::fromUtf8("pushButton_Reset"));
        QIcon icon;
        icon.addFile(QString::fromUtf8(":/images/control/reset"), QSize(), QIcon::Normal, QIcon::Off);
        pushButton_Reset->setIcon(icon);
        pushButton_Reset->setIconSize(QSize(24, 24));

        horizontalLayout_1->addWidget(pushButton_Reset);

        pushButton_StartStop = new QPushButton(cbControlPanel);
        pushButton_StartStop->setObjectName(QString::fromUtf8("pushButton_StartStop"));
        pushButton_StartStop->setEnabled(false);
        QIcon icon1;
        icon1.addFile(QString::fromUtf8(":/images/control/start"), QSize(), QIcon::Normal, QIcon::Off);
        icon1.addFile(QString::fromUtf8(":/images/control/stop"), QSize(), QIcon::Normal, QIcon::On);
        pushButton_StartStop->setIcon(icon1);
        pushButton_StartStop->setIconSize(QSize(24, 24));

        horizontalLayout_1->addWidget(pushButton_StartStop);

        line_1 = new QFrame(cbControlPanel);
        line_1->setObjectName(QString::fromUtf8("line_1"));
        line_1->setFrameShape(QFrame::VLine);
        line_1->setFrameShadow(QFrame::Sunken);

        horizontalLayout_1->addWidget(line_1);

        spinBox_Robot = new QSpinBox(cbControlPanel);
        spinBox_Robot->setObjectName(QString::fromUtf8("spinBox_Robot"));

        horizontalLayout_1->addWidget(spinBox_Robot);

        pushButton_Remove = new QPushButton(cbControlPanel);
        pushButton_Remove->setObjectName(QString::fromUtf8("pushButton_Remove"));
        QIcon icon2;
        icon2.addFile(QString::fromUtf8(":/images/control/remove"), QSize(), QIcon::Normal, QIcon::Off);
        pushButton_Remove->setIcon(icon2);
        pushButton_Remove->setIconSize(QSize(24, 24));

        horizontalLayout_1->addWidget(pushButton_Remove);

        line_2 = new QFrame(cbControlPanel);
        line_2->setObjectName(QString::fromUtf8("line_2"));
        line_2->setLineWidth(1);
        line_2->setFrameShape(QFrame::VLine);
        line_2->setFrameShadow(QFrame::Sunken);

        horizontalLayout_1->addWidget(line_2);

        pushButton_Time = new QPushButton(cbControlPanel);
        pushButton_Time->setObjectName(QString::fromUtf8("pushButton_Time"));
        QIcon icon3;
        icon3.addFile(QString::fromUtf8(":/images/buttons/time_off"), QSize(), QIcon::Normal, QIcon::Off);
        icon3.addFile(QString::fromUtf8(":/images/buttons/time_on"), QSize(), QIcon::Normal, QIcon::On);
        pushButton_Time->setIcon(icon3);
        pushButton_Time->setIconSize(QSize(24, 24));
        pushButton_Time->setCheckable(true);
        pushButton_Time->setChecked(true);

        horizontalLayout_1->addWidget(pushButton_Time);

        pushButton_Collisions = new QPushButton(cbControlPanel);
        pushButton_Collisions->setObjectName(QString::fromUtf8("pushButton_Collisions"));
        QIcon icon4;
        icon4.addFile(QString::fromUtf8(":/images/buttons/collisions_off"), QSize(), QIcon::Normal, QIcon::Off);
        icon4.addFile(QString::fromUtf8(":/images/buttons/collisions_on"), QSize(), QIcon::Normal, QIcon::On);
        pushButton_Collisions->setIcon(icon4);
        pushButton_Collisions->setIconSize(QSize(24, 24));
        pushButton_Collisions->setCheckable(true);
        pushButton_Collisions->setChecked(true);

        horizontalLayout_1->addWidget(pushButton_Collisions);

        horizontalSpacer = new QSpacerItem(0, 1, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_1->addItem(horizontalSpacer);

        pushButton_Lock = new QPushButton(cbControlPanel);
        pushButton_Lock->setObjectName(QString::fromUtf8("pushButton_Lock"));
        pushButton_Lock->setEnabled(false);
        QIcon icon5;
        icon5.addFile(QString::fromUtf8(":/images/buttons/lock_closed"), QSize(), QIcon::Normal, QIcon::Off);
        icon5.addFile(QString::fromUtf8(":/images/buttons/lock_open"), QSize(), QIcon::Normal, QIcon::On);
        pushButton_Lock->setIcon(icon5);
        pushButton_Lock->setIconSize(QSize(24, 24));
        pushButton_Lock->setCheckable(true);
        pushButton_Lock->setChecked(true);

        horizontalLayout_1->addWidget(pushButton_Lock);

        pushButton_ManageRobots = new QPushButton(cbControlPanel);
        pushButton_ManageRobots->setObjectName(QString::fromUtf8("pushButton_ManageRobots"));
        QIcon icon6;
        icon6.addFile(QString::fromUtf8(":/images/buttons/mouse"), QSize(), QIcon::Normal, QIcon::Off);
        pushButton_ManageRobots->setIcon(icon6);
        pushButton_ManageRobots->setIconSize(QSize(24, 24));
        pushButton_ManageRobots->setCheckable(true);

        horizontalLayout_1->addWidget(pushButton_ManageRobots);


        gridLayout->addLayout(horizontalLayout_1, 0, 1, 1, 1);


        retranslateUi(cbControlPanel);

        QMetaObject::connectSlotsByName(cbControlPanel);
    } // setupUi

    void retranslateUi(QWidget *cbControlPanel)
    {
        cbControlPanel->setWindowTitle(QApplication::translate("cbControlPanel", "CiberRato Control Panel", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        pushButton_Reset->setToolTip(QApplication::translate("cbControlPanel", "Reset", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        pushButton_Reset->setText(QString());
#ifndef QT_NO_TOOLTIP
        pushButton_StartStop->setToolTip(QApplication::translate("cbControlPanel", "Start", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        pushButton_StartStop->setText(QString());
#ifndef QT_NO_TOOLTIP
        pushButton_Remove->setToolTip(QApplication::translate("cbControlPanel", "Kill Robot", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        pushButton_Remove->setText(QString());
#ifndef QT_NO_TOOLTIP
        pushButton_Time->setToolTip(QApplication::translate("cbControlPanel", "Toggle Time Limit", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        pushButton_Time->setText(QString());
#ifndef QT_NO_TOOLTIP
        pushButton_Collisions->setToolTip(QApplication::translate("cbControlPanel", "Toggle Collisions between Robots", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        pushButton_Collisions->setText(QString());
#ifndef QT_NO_TOOLTIP
        pushButton_Lock->setToolTip(QApplication::translate("cbControlPanel", "Toggle Robot Registrations", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        pushButton_Lock->setText(QString());
#ifndef QT_NO_TOOLTIP
        pushButton_ManageRobots->setToolTip(QApplication::translate("cbControlPanel", "Robot Management", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        pushButton_ManageRobots->setText(QString());
    } // retranslateUi

};

namespace Ui {
    class cbControlPanel: public Ui_cbControlPanel {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_CBCONTROLPANEL_H

/********************************************************************************
** Form generated from reading UI file 'cbsimulatorGUI.ui'
**
** Created by: Qt User Interface Compiler version 4.8.7
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_CBSIMULATORGUI_H
#define UI_CBSIMULATORGUI_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QComboBox>
#include <QtGui/QDockWidget>
#include <QtGui/QGridLayout>
#include <QtGui/QGroupBox>
#include <QtGui/QHeaderView>
#include <QtGui/QLCDNumber>
#include <QtGui/QLabel>
#include <QtGui/QMainWindow>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QPushButton>
#include <QtGui/QSpacerItem>
#include <QtGui/QStatusBar>
#include <QtGui/QTextEdit>
#include <QtGui/QToolBar>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_cbSimulatorGUI
{
public:
    QAction *actionReset;
    QAction *actionSimulator_Help;
    QAction *actionAbout_Simulator;
    QAction *actionExit;
    QAction *actionGPS_Enabled;
    QAction *actionScore_Sensor_Enabled;
    QAction *actionTime_Enabled;
    QAction *actionOpen_Lab;
    QAction *actionEdit_Configuration;
    QAction *actionOpen_Configuration;
    QAction *actionSave_Configuration;
    QAction *actionLaunch_Viewer;
    QAction *actionRegistrations_Open;
    QAction *actionStartStop;
    QAction *actionManageRobots;
    QAction *actionControl_Panel;
    QAction *actionLoad_Defaults;
    QAction *actionCollisions_Enabled;
    QWidget *centralwidget;
    QVBoxLayout *verticalLayout;
    QTextEdit *messages;
    QMenuBar *menubar;
    QMenu *menuFile;
    QMenu *menuConfiguration;
    QMenu *menuCommand;
    QMenu *menuOptions;
    QMenu *menuHelp;
    QStatusBar *statusbar;
    QToolBar *toolBar_1;
    QDockWidget *dockWidget_Score;
    QWidget *dockWidgetContents_Score;
    QGridLayout *gridLayout;
    QLabel *label_2;
    QGridLayout *gridLayout_Scores;
    QSpacerItem *horizontalSpacer_2;
    QLCDNumber *lcdNumber_TimeRemain;
    QSpacerItem *verticalSpacer_2;
    QToolBar *toolBar_2;
    QDockWidget *dockWidget_Mailbox;
    QWidget *dockWidgetContents_Mail;
    QVBoxLayout *verticalLayout_2;
    QGroupBox *groupBox;
    QGridLayout *gridLayout_3;
    QComboBox *comboBox_Robot;
    QCheckBox *checkBox_Actions;
    QCheckBox *checkBox_Measures;
    QCheckBox *checkBox_Positions;
    QLabel *label;
    QPushButton *pushButton_ClearMailBox;
    QSpacerItem *horizontalSpacer;
    QTextEdit *textEdit_Mailbox;

    void setupUi(QMainWindow *cbSimulatorGUI)
    {
        if (cbSimulatorGUI->objectName().isEmpty())
            cbSimulatorGUI->setObjectName(QString::fromUtf8("cbSimulatorGUI"));
        cbSimulatorGUI->resize(994, 750);
        actionReset = new QAction(cbSimulatorGUI);
        actionReset->setObjectName(QString::fromUtf8("actionReset"));
        QIcon icon;
        icon.addFile(QString::fromUtf8(":/images/control/reset"), QSize(), QIcon::Normal, QIcon::Off);
        actionReset->setIcon(icon);
        actionSimulator_Help = new QAction(cbSimulatorGUI);
        actionSimulator_Help->setObjectName(QString::fromUtf8("actionSimulator_Help"));
        actionAbout_Simulator = new QAction(cbSimulatorGUI);
        actionAbout_Simulator->setObjectName(QString::fromUtf8("actionAbout_Simulator"));
        actionExit = new QAction(cbSimulatorGUI);
        actionExit->setObjectName(QString::fromUtf8("actionExit"));
        QIcon icon1;
        icon1.addFile(QString::fromUtf8(":/images/buttons/exit"), QSize(), QIcon::Normal, QIcon::Off);
        actionExit->setIcon(icon1);
        actionGPS_Enabled = new QAction(cbSimulatorGUI);
        actionGPS_Enabled->setObjectName(QString::fromUtf8("actionGPS_Enabled"));
        actionGPS_Enabled->setCheckable(true);
        actionGPS_Enabled->setChecked(true);
        actionScore_Sensor_Enabled = new QAction(cbSimulatorGUI);
        actionScore_Sensor_Enabled->setObjectName(QString::fromUtf8("actionScore_Sensor_Enabled"));
        actionScore_Sensor_Enabled->setCheckable(true);
        actionTime_Enabled = new QAction(cbSimulatorGUI);
        actionTime_Enabled->setObjectName(QString::fromUtf8("actionTime_Enabled"));
        actionTime_Enabled->setCheckable(true);
        actionTime_Enabled->setChecked(true);
        actionTime_Enabled->setEnabled(true);
        QIcon icon2;
        icon2.addFile(QString::fromUtf8(":/images/buttons/time_off"), QSize(), QIcon::Normal, QIcon::Off);
        icon2.addFile(QString::fromUtf8(":/images/buttons/time_on"), QSize(), QIcon::Normal, QIcon::On);
        actionTime_Enabled->setIcon(icon2);
        actionOpen_Lab = new QAction(cbSimulatorGUI);
        actionOpen_Lab->setObjectName(QString::fromUtf8("actionOpen_Lab"));
        QIcon icon3;
        icon3.addFile(QString::fromUtf8(":/images/buttons/lab"), QSize(), QIcon::Normal, QIcon::Off);
        actionOpen_Lab->setIcon(icon3);
        actionEdit_Configuration = new QAction(cbSimulatorGUI);
        actionEdit_Configuration->setObjectName(QString::fromUtf8("actionEdit_Configuration"));
        QIcon icon4;
        icon4.addFile(QString::fromUtf8(":/images/buttons/config"), QSize(), QIcon::Normal, QIcon::Off);
        actionEdit_Configuration->setIcon(icon4);
        actionOpen_Configuration = new QAction(cbSimulatorGUI);
        actionOpen_Configuration->setObjectName(QString::fromUtf8("actionOpen_Configuration"));
        QIcon icon5;
        icon5.addFile(QString::fromUtf8(":/images/buttons/open"), QSize(), QIcon::Normal, QIcon::Off);
        actionOpen_Configuration->setIcon(icon5);
        actionSave_Configuration = new QAction(cbSimulatorGUI);
        actionSave_Configuration->setObjectName(QString::fromUtf8("actionSave_Configuration"));
        QIcon icon6;
        icon6.addFile(QString::fromUtf8(":/images/buttons/save"), QSize(), QIcon::Normal, QIcon::Off);
        actionSave_Configuration->setIcon(icon6);
        actionLaunch_Viewer = new QAction(cbSimulatorGUI);
        actionLaunch_Viewer->setObjectName(QString::fromUtf8("actionLaunch_Viewer"));
        actionLaunch_Viewer->setCheckable(true);
        QIcon icon7;
        icon7.addFile(QString::fromUtf8(":/images/buttons/viewer_close"), QSize(), QIcon::Normal, QIcon::Off);
        icon7.addFile(QString::fromUtf8(":/images/buttons/viewer_open"), QSize(), QIcon::Normal, QIcon::On);
        actionLaunch_Viewer->setIcon(icon7);
        actionRegistrations_Open = new QAction(cbSimulatorGUI);
        actionRegistrations_Open->setObjectName(QString::fromUtf8("actionRegistrations_Open"));
        actionRegistrations_Open->setCheckable(true);
        actionRegistrations_Open->setChecked(true);
        actionRegistrations_Open->setEnabled(false);
        QIcon icon8;
        icon8.addFile(QString::fromUtf8(":/images/buttons/lock_closed"), QSize(), QIcon::Normal, QIcon::Off);
        icon8.addFile(QString::fromUtf8(":/images/buttons/lock_open"), QSize(), QIcon::Normal, QIcon::On);
        actionRegistrations_Open->setIcon(icon8);
        actionStartStop = new QAction(cbSimulatorGUI);
        actionStartStop->setObjectName(QString::fromUtf8("actionStartStop"));
        actionStartStop->setEnabled(false);
        QIcon icon9;
        icon9.addFile(QString::fromUtf8(":/images/control/start"), QSize(), QIcon::Normal, QIcon::Off);
        icon9.addFile(QString::fromUtf8(":/images/control/stop"), QSize(), QIcon::Normal, QIcon::On);
        actionStartStop->setIcon(icon9);
        actionManageRobots = new QAction(cbSimulatorGUI);
        actionManageRobots->setObjectName(QString::fromUtf8("actionManageRobots"));
        actionManageRobots->setCheckable(true);
        QIcon icon10;
        icon10.addFile(QString::fromUtf8(":/images/buttons/mouse"), QSize(), QIcon::Normal, QIcon::Off);
        actionManageRobots->setIcon(icon10);
        actionControl_Panel = new QAction(cbSimulatorGUI);
        actionControl_Panel->setObjectName(QString::fromUtf8("actionControl_Panel"));
        actionControl_Panel->setCheckable(true);
        QIcon icon11;
        icon11.addFile(QString::fromUtf8(":/images/buttons/panel"), QSize(), QIcon::Normal, QIcon::Off);
        actionControl_Panel->setIcon(icon11);
        actionLoad_Defaults = new QAction(cbSimulatorGUI);
        actionLoad_Defaults->setObjectName(QString::fromUtf8("actionLoad_Defaults"));
        QIcon icon12;
        icon12.addFile(QString::fromUtf8(":/images/buttons/defaults"), QSize(), QIcon::Normal, QIcon::Off);
        actionLoad_Defaults->setIcon(icon12);
        actionCollisions_Enabled = new QAction(cbSimulatorGUI);
        actionCollisions_Enabled->setObjectName(QString::fromUtf8("actionCollisions_Enabled"));
        actionCollisions_Enabled->setCheckable(true);
        actionCollisions_Enabled->setChecked(true);
        QIcon icon13;
        icon13.addFile(QString::fromUtf8(":/images/buttons/collisions_off"), QSize(), QIcon::Normal, QIcon::Off);
        icon13.addFile(QString::fromUtf8(":/images/buttons/collisions_on"), QSize(), QIcon::Normal, QIcon::On);
        actionCollisions_Enabled->setIcon(icon13);
        centralwidget = new QWidget(cbSimulatorGUI);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        verticalLayout = new QVBoxLayout(centralwidget);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        messages = new QTextEdit(centralwidget);
        messages->setObjectName(QString::fromUtf8("messages"));
        messages->setReadOnly(true);

        verticalLayout->addWidget(messages);

        cbSimulatorGUI->setCentralWidget(centralwidget);
        menubar = new QMenuBar(cbSimulatorGUI);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 994, 25));
        menuFile = new QMenu(menubar);
        menuFile->setObjectName(QString::fromUtf8("menuFile"));
        menuConfiguration = new QMenu(menuFile);
        menuConfiguration->setObjectName(QString::fromUtf8("menuConfiguration"));
        menuCommand = new QMenu(menubar);
        menuCommand->setObjectName(QString::fromUtf8("menuCommand"));
        menuOptions = new QMenu(menubar);
        menuOptions->setObjectName(QString::fromUtf8("menuOptions"));
        menuHelp = new QMenu(menubar);
        menuHelp->setObjectName(QString::fromUtf8("menuHelp"));
        cbSimulatorGUI->setMenuBar(menubar);
        statusbar = new QStatusBar(cbSimulatorGUI);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        cbSimulatorGUI->setStatusBar(statusbar);
        toolBar_1 = new QToolBar(cbSimulatorGUI);
        toolBar_1->setObjectName(QString::fromUtf8("toolBar_1"));
        toolBar_1->setIconSize(QSize(24, 24));
        cbSimulatorGUI->addToolBar(Qt::TopToolBarArea, toolBar_1);
        dockWidget_Score = new QDockWidget(cbSimulatorGUI);
        dockWidget_Score->setObjectName(QString::fromUtf8("dockWidget_Score"));
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Minimum);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(dockWidget_Score->sizePolicy().hasHeightForWidth());
        dockWidget_Score->setSizePolicy(sizePolicy);
        QIcon icon14;
        icon14.addFile(QString::fromUtf8(":/images/icons/score"), QSize(), QIcon::Normal, QIcon::Off);
        dockWidget_Score->setWindowIcon(icon14);
        dockWidget_Score->setFloating(false);
        dockWidget_Score->setFeatures(QDockWidget::AllDockWidgetFeatures);
        dockWidgetContents_Score = new QWidget();
        dockWidgetContents_Score->setObjectName(QString::fromUtf8("dockWidgetContents_Score"));
        gridLayout = new QGridLayout(dockWidgetContents_Score);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        label_2 = new QLabel(dockWidgetContents_Score);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        QFont font;
        font.setPointSize(11);
        font.setBold(false);
        font.setWeight(50);
        label_2->setFont(font);

        gridLayout->addWidget(label_2, 0, 0, 1, 1);

        gridLayout_Scores = new QGridLayout();
        gridLayout_Scores->setObjectName(QString::fromUtf8("gridLayout_Scores"));

        gridLayout->addLayout(gridLayout_Scores, 0, 2, 3, 2);

        horizontalSpacer_2 = new QSpacerItem(60, 1, QSizePolicy::Fixed, QSizePolicy::Minimum);

        gridLayout->addItem(horizontalSpacer_2, 0, 1, 1, 1);

        lcdNumber_TimeRemain = new QLCDNumber(dockWidgetContents_Score);
        lcdNumber_TimeRemain->setObjectName(QString::fromUtf8("lcdNumber_TimeRemain"));
        QSizePolicy sizePolicy1(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(lcdNumber_TimeRemain->sizePolicy().hasHeightForWidth());
        lcdNumber_TimeRemain->setSizePolicy(sizePolicy1);
        lcdNumber_TimeRemain->setMinimumSize(QSize(100, 27));
        lcdNumber_TimeRemain->setMaximumSize(QSize(100, 27));
        QPalette palette;
        QBrush brush(QColor(0, 0, 0, 255));
        brush.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::WindowText, brush);
        QBrush brush1(QColor(255, 255, 255, 255));
        brush1.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::Button, brush1);
        palette.setBrush(QPalette::Active, QPalette::Light, brush1);
        palette.setBrush(QPalette::Active, QPalette::Midlight, brush1);
        QBrush brush2(QColor(127, 127, 127, 255));
        brush2.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::Dark, brush2);
        QBrush brush3(QColor(170, 170, 170, 255));
        brush3.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::Mid, brush3);
        palette.setBrush(QPalette::Active, QPalette::Text, brush);
        palette.setBrush(QPalette::Active, QPalette::BrightText, brush1);
        palette.setBrush(QPalette::Active, QPalette::ButtonText, brush);
        palette.setBrush(QPalette::Active, QPalette::Base, brush1);
        palette.setBrush(QPalette::Active, QPalette::Window, brush1);
        palette.setBrush(QPalette::Active, QPalette::Shadow, brush);
        palette.setBrush(QPalette::Active, QPalette::AlternateBase, brush1);
        QBrush brush4(QColor(255, 255, 220, 255));
        brush4.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::ToolTipBase, brush4);
        palette.setBrush(QPalette::Active, QPalette::ToolTipText, brush);
        palette.setBrush(QPalette::Inactive, QPalette::WindowText, brush);
        palette.setBrush(QPalette::Inactive, QPalette::Button, brush1);
        palette.setBrush(QPalette::Inactive, QPalette::Light, brush1);
        palette.setBrush(QPalette::Inactive, QPalette::Midlight, brush1);
        palette.setBrush(QPalette::Inactive, QPalette::Dark, brush2);
        palette.setBrush(QPalette::Inactive, QPalette::Mid, brush3);
        palette.setBrush(QPalette::Inactive, QPalette::Text, brush);
        palette.setBrush(QPalette::Inactive, QPalette::BrightText, brush1);
        palette.setBrush(QPalette::Inactive, QPalette::ButtonText, brush);
        palette.setBrush(QPalette::Inactive, QPalette::Base, brush1);
        palette.setBrush(QPalette::Inactive, QPalette::Window, brush1);
        palette.setBrush(QPalette::Inactive, QPalette::Shadow, brush);
        palette.setBrush(QPalette::Inactive, QPalette::AlternateBase, brush1);
        palette.setBrush(QPalette::Inactive, QPalette::ToolTipBase, brush4);
        palette.setBrush(QPalette::Inactive, QPalette::ToolTipText, brush);
        palette.setBrush(QPalette::Disabled, QPalette::WindowText, brush2);
        palette.setBrush(QPalette::Disabled, QPalette::Button, brush1);
        palette.setBrush(QPalette::Disabled, QPalette::Light, brush1);
        palette.setBrush(QPalette::Disabled, QPalette::Midlight, brush1);
        palette.setBrush(QPalette::Disabled, QPalette::Dark, brush2);
        palette.setBrush(QPalette::Disabled, QPalette::Mid, brush3);
        palette.setBrush(QPalette::Disabled, QPalette::Text, brush2);
        palette.setBrush(QPalette::Disabled, QPalette::BrightText, brush1);
        palette.setBrush(QPalette::Disabled, QPalette::ButtonText, brush2);
        palette.setBrush(QPalette::Disabled, QPalette::Base, brush1);
        palette.setBrush(QPalette::Disabled, QPalette::Window, brush1);
        palette.setBrush(QPalette::Disabled, QPalette::Shadow, brush);
        palette.setBrush(QPalette::Disabled, QPalette::AlternateBase, brush1);
        palette.setBrush(QPalette::Disabled, QPalette::ToolTipBase, brush4);
        palette.setBrush(QPalette::Disabled, QPalette::ToolTipText, brush);
        lcdNumber_TimeRemain->setPalette(palette);
        lcdNumber_TimeRemain->setFrameShape(QFrame::NoFrame);
        lcdNumber_TimeRemain->setDigitCount(4);
        lcdNumber_TimeRemain->setSegmentStyle(QLCDNumber::Flat);

        gridLayout->addWidget(lcdNumber_TimeRemain, 1, 0, 1, 1);

        verticalSpacer_2 = new QSpacerItem(1, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);

        gridLayout->addItem(verticalSpacer_2, 2, 0, 1, 2);

        dockWidget_Score->setWidget(dockWidgetContents_Score);
        cbSimulatorGUI->addDockWidget(static_cast<Qt::DockWidgetArea>(8), dockWidget_Score);
        toolBar_2 = new QToolBar(cbSimulatorGUI);
        toolBar_2->setObjectName(QString::fromUtf8("toolBar_2"));
        toolBar_2->setIconSize(QSize(24, 24));
        cbSimulatorGUI->addToolBar(Qt::TopToolBarArea, toolBar_2);
        dockWidget_Mailbox = new QDockWidget(cbSimulatorGUI);
        dockWidget_Mailbox->setObjectName(QString::fromUtf8("dockWidget_Mailbox"));
        dockWidget_Mailbox->setEnabled(true);
        sizePolicy.setHeightForWidth(dockWidget_Mailbox->sizePolicy().hasHeightForWidth());
        dockWidget_Mailbox->setSizePolicy(sizePolicy);
        QIcon icon15;
        icon15.addFile(QString::fromUtf8(":/images/icons/message"), QSize(), QIcon::Normal, QIcon::Off);
        dockWidget_Mailbox->setWindowIcon(icon15);
        dockWidget_Mailbox->setFloating(false);
        dockWidgetContents_Mail = new QWidget();
        dockWidgetContents_Mail->setObjectName(QString::fromUtf8("dockWidgetContents_Mail"));
        verticalLayout_2 = new QVBoxLayout(dockWidgetContents_Mail);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        groupBox = new QGroupBox(dockWidgetContents_Mail);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        gridLayout_3 = new QGridLayout(groupBox);
        gridLayout_3->setObjectName(QString::fromUtf8("gridLayout_3"));
        comboBox_Robot = new QComboBox(groupBox);
        comboBox_Robot->setObjectName(QString::fromUtf8("comboBox_Robot"));
        comboBox_Robot->setMinimumSize(QSize(120, 0));
        comboBox_Robot->setInsertPolicy(QComboBox::InsertAlphabetically);

        gridLayout_3->addWidget(comboBox_Robot, 0, 4, 1, 1);

        checkBox_Actions = new QCheckBox(groupBox);
        checkBox_Actions->setObjectName(QString::fromUtf8("checkBox_Actions"));
        checkBox_Actions->setChecked(true);

        gridLayout_3->addWidget(checkBox_Actions, 0, 0, 1, 1);

        checkBox_Measures = new QCheckBox(groupBox);
        checkBox_Measures->setObjectName(QString::fromUtf8("checkBox_Measures"));
        checkBox_Measures->setChecked(true);

        gridLayout_3->addWidget(checkBox_Measures, 0, 1, 1, 1);

        checkBox_Positions = new QCheckBox(groupBox);
        checkBox_Positions->setObjectName(QString::fromUtf8("checkBox_Positions"));
        checkBox_Positions->setChecked(false);

        gridLayout_3->addWidget(checkBox_Positions, 0, 2, 1, 1);

        label = new QLabel(groupBox);
        label->setObjectName(QString::fromUtf8("label"));
        label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        gridLayout_3->addWidget(label, 0, 3, 1, 1);

        pushButton_ClearMailBox = new QPushButton(groupBox);
        pushButton_ClearMailBox->setObjectName(QString::fromUtf8("pushButton_ClearMailBox"));

        gridLayout_3->addWidget(pushButton_ClearMailBox, 0, 6, 1, 1);

        horizontalSpacer = new QSpacerItem(0, 1, QSizePolicy::Expanding, QSizePolicy::Minimum);

        gridLayout_3->addItem(horizontalSpacer, 0, 5, 1, 1);


        verticalLayout_2->addWidget(groupBox);

        textEdit_Mailbox = new QTextEdit(dockWidgetContents_Mail);
        textEdit_Mailbox->setObjectName(QString::fromUtf8("textEdit_Mailbox"));
        textEdit_Mailbox->setReadOnly(true);

        verticalLayout_2->addWidget(textEdit_Mailbox);

        dockWidget_Mailbox->setWidget(dockWidgetContents_Mail);
        cbSimulatorGUI->addDockWidget(static_cast<Qt::DockWidgetArea>(8), dockWidget_Mailbox);

        menubar->addAction(menuFile->menuAction());
        menubar->addAction(menuCommand->menuAction());
        menubar->addAction(menuOptions->menuAction());
        menubar->addAction(menuHelp->menuAction());
        menuFile->addAction(menuConfiguration->menuAction());
        menuFile->addAction(actionOpen_Lab);
        menuFile->addAction(actionLoad_Defaults);
        menuFile->addSeparator();
        menuFile->addAction(actionManageRobots);
        menuFile->addAction(actionControl_Panel);
        menuFile->addAction(actionLaunch_Viewer);
        menuFile->addSeparator();
        menuFile->addAction(actionExit);
        menuConfiguration->addAction(actionEdit_Configuration);
        menuConfiguration->addAction(actionOpen_Configuration);
        menuConfiguration->addAction(actionSave_Configuration);
        menuCommand->addAction(actionReset);
        menuCommand->addAction(actionStartStop);
        menuOptions->addAction(actionGPS_Enabled);
        menuOptions->addAction(actionScore_Sensor_Enabled);
        menuOptions->addAction(actionTime_Enabled);
        menuOptions->addAction(actionCollisions_Enabled);
        menuOptions->addAction(actionRegistrations_Open);
        menuHelp->addAction(actionSimulator_Help);
        menuHelp->addAction(actionAbout_Simulator);
        toolBar_1->addAction(actionReset);
        toolBar_1->addAction(actionStartStop);
        toolBar_1->addSeparator();
        toolBar_1->addAction(actionTime_Enabled);
        toolBar_1->addAction(actionCollisions_Enabled);
        toolBar_1->addAction(actionRegistrations_Open);
        toolBar_1->addAction(actionManageRobots);
        toolBar_1->addSeparator();
        toolBar_1->addAction(actionControl_Panel);
        toolBar_1->addAction(actionLaunch_Viewer);
        toolBar_2->addAction(actionEdit_Configuration);
        toolBar_2->addAction(actionOpen_Configuration);
        toolBar_2->addAction(actionSave_Configuration);
        toolBar_2->addAction(actionOpen_Lab);
        toolBar_2->addAction(actionLoad_Defaults);

        retranslateUi(cbSimulatorGUI);

        QMetaObject::connectSlotsByName(cbSimulatorGUI);
    } // setupUi

    void retranslateUi(QMainWindow *cbSimulatorGUI)
    {
        cbSimulatorGUI->setWindowTitle(QApplication::translate("cbSimulatorGUI", "Explorer Simulator 2021", 0, QApplication::UnicodeUTF8));
        actionReset->setText(QApplication::translate("cbSimulatorGUI", "&Reset", 0, QApplication::UnicodeUTF8));
        actionReset->setShortcut(QApplication::translate("cbSimulatorGUI", "Ctrl+R", 0, QApplication::UnicodeUTF8));
        actionSimulator_Help->setText(QApplication::translate("cbSimulatorGUI", "&Help", 0, QApplication::UnicodeUTF8));
        actionSimulator_Help->setShortcut(QApplication::translate("cbSimulatorGUI", "F1", 0, QApplication::UnicodeUTF8));
        actionAbout_Simulator->setText(QApplication::translate("cbSimulatorGUI", "&About Simulator", 0, QApplication::UnicodeUTF8));
        actionExit->setText(QApplication::translate("cbSimulatorGUI", "E&xit", 0, QApplication::UnicodeUTF8));
        actionExit->setShortcut(QApplication::translate("cbSimulatorGUI", "Ctrl+Q", 0, QApplication::UnicodeUTF8));
        actionGPS_Enabled->setText(QApplication::translate("cbSimulatorGUI", "GP&S Enabled", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        actionGPS_Enabled->setToolTip(QApplication::translate("cbSimulatorGUI", "Toggle GPS", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        actionScore_Sensor_Enabled->setText(QApplication::translate("cbSimulatorGUI", "S&core Sensor Enabled", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        actionScore_Sensor_Enabled->setToolTip(QApplication::translate("cbSimulatorGUI", "Toggle Score Sensor", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        actionTime_Enabled->setText(QApplication::translate("cbSimulatorGUI", "T&ime Enabled", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        actionTime_Enabled->setToolTip(QApplication::translate("cbSimulatorGUI", "Toggle Simulation Time Limit", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        actionOpen_Lab->setText(QApplication::translate("cbSimulatorGUI", "Change &Lab", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        actionOpen_Lab->setToolTip(QApplication::translate("cbSimulatorGUI", "Change Lab and Grid", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        actionOpen_Lab->setShortcut(QApplication::translate("cbSimulatorGUI", "Ctrl+L", 0, QApplication::UnicodeUTF8));
        actionEdit_Configuration->setText(QApplication::translate("cbSimulatorGUI", "&Edit Configuration", 0, QApplication::UnicodeUTF8));
        actionEdit_Configuration->setShortcut(QApplication::translate("cbSimulatorGUI", "Ctrl+E", 0, QApplication::UnicodeUTF8));
        actionOpen_Configuration->setText(QApplication::translate("cbSimulatorGUI", "Open &Configuration", 0, QApplication::UnicodeUTF8));
        actionSave_Configuration->setText(QApplication::translate("cbSimulatorGUI", "&Save Configuration", 0, QApplication::UnicodeUTF8));
        actionLaunch_Viewer->setText(QApplication::translate("cbSimulatorGUI", "Launch &Viewer", 0, QApplication::UnicodeUTF8));
        actionLaunch_Viewer->setShortcut(QApplication::translate("cbSimulatorGUI", "Ctrl+V", 0, QApplication::UnicodeUTF8));
        actionRegistrations_Open->setText(QApplication::translate("cbSimulatorGUI", "Re&gistrations Allowed", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        actionRegistrations_Open->setToolTip(QApplication::translate("cbSimulatorGUI", "Toggle Robot Registrations", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        actionStartStop->setText(QApplication::translate("cbSimulatorGUI", "&Start", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        actionStartStop->setToolTip(QApplication::translate("cbSimulatorGUI", "Start", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        actionStartStop->setShortcut(QApplication::translate("cbSimulatorGUI", "Ctrl+S", 0, QApplication::UnicodeUTF8));
        actionManageRobots->setText(QApplication::translate("cbSimulatorGUI", "&Manage Robots", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        actionManageRobots->setToolTip(QApplication::translate("cbSimulatorGUI", "Show Robots Management Window", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        actionManageRobots->setShortcut(QApplication::translate("cbSimulatorGUI", "Ctrl+M", 0, QApplication::UnicodeUTF8));
        actionControl_Panel->setText(QApplication::translate("cbSimulatorGUI", "Control &Panel", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        actionControl_Panel->setToolTip(QApplication::translate("cbSimulatorGUI", "Show Control Panel Window", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        actionControl_Panel->setShortcut(QApplication::translate("cbSimulatorGUI", "F2", 0, QApplication::UnicodeUTF8));
        actionLoad_Defaults->setText(QApplication::translate("cbSimulatorGUI", "Load &Defaults", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        actionLoad_Defaults->setToolTip(QApplication::translate("cbSimulatorGUI", "Sets Lab, Grid and Parameters to Default Values", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        actionLoad_Defaults->setShortcut(QApplication::translate("cbSimulatorGUI", "Ctrl+D", 0, QApplication::UnicodeUTF8));
        actionCollisions_Enabled->setText(QApplication::translate("cbSimulatorGUI", "Co&llisions Enabled", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        actionCollisions_Enabled->setToolTip(QApplication::translate("cbSimulatorGUI", "Toggle Collisions between Robots", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
#ifndef QT_NO_TOOLTIP
        messages->setToolTip(QApplication::translate("cbSimulatorGUI", "Messages", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        menuFile->setTitle(QApplication::translate("cbSimulatorGUI", "&File", 0, QApplication::UnicodeUTF8));
        menuConfiguration->setTitle(QApplication::translate("cbSimulatorGUI", "&Configuration", 0, QApplication::UnicodeUTF8));
        menuCommand->setTitle(QApplication::translate("cbSimulatorGUI", "&Command", 0, QApplication::UnicodeUTF8));
        menuOptions->setTitle(QApplication::translate("cbSimulatorGUI", "&Options", 0, QApplication::UnicodeUTF8));
        menuHelp->setTitle(QApplication::translate("cbSimulatorGUI", "&Help", 0, QApplication::UnicodeUTF8));
        toolBar_1->setWindowTitle(QApplication::translate("cbSimulatorGUI", "Control ToolBar", 0, QApplication::UnicodeUTF8));
        dockWidget_Score->setWindowTitle(QApplication::translate("cbSimulatorGUI", "ScoreBoard", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("cbSimulatorGUI", "Time remaining", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        lcdNumber_TimeRemain->setToolTip(QApplication::translate("cbSimulatorGUI", "Time remaining", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        toolBar_2->setWindowTitle(QApplication::translate("cbSimulatorGUI", "Settings ToolBar", 0, QApplication::UnicodeUTF8));
        dockWidget_Mailbox->setWindowTitle(QApplication::translate("cbSimulatorGUI", "Mailbox", 0, QApplication::UnicodeUTF8));
        groupBox->setTitle(QApplication::translate("cbSimulatorGUI", "Show messages", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        comboBox_Robot->setToolTip(QApplication::translate("cbSimulatorGUI", "Robot", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
#ifndef QT_NO_TOOLTIP
        checkBox_Actions->setToolTip(QApplication::translate("cbSimulatorGUI", "Motor Actions received from robot", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        checkBox_Actions->setText(QApplication::translate("cbSimulatorGUI", "Actions received", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        checkBox_Measures->setToolTip(QApplication::translate("cbSimulatorGUI", "Sensor Measures sent to robot", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        checkBox_Measures->setText(QApplication::translate("cbSimulatorGUI", "Measures sent", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        checkBox_Positions->setToolTip(QApplication::translate("cbSimulatorGUI", "Robot Positions sent to Viewer(s)", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        checkBox_Positions->setText(QApplication::translate("cbSimulatorGUI", "Positions Sent", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("cbSimulatorGUI", "relative to", 0, QApplication::UnicodeUTF8));
        pushButton_ClearMailBox->setText(QApplication::translate("cbSimulatorGUI", "Clear", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class cbSimulatorGUI: public Ui_cbSimulatorGUI {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_CBSIMULATORGUI_H

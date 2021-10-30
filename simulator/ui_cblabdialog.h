/********************************************************************************
** Form generated from reading UI file 'cblabdialog.ui'
**
** Created by: Qt User Interface Compiler version 4.8.7
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_CBLABDIALOG_H
#define UI_CBLABDIALOG_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QDialog>
#include <QtGui/QDialogButtonBox>
#include <QtGui/QGraphicsView>
#include <QtGui/QGridLayout>
#include <QtGui/QGroupBox>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QPushButton>
#include <QtGui/QTreeView>
#include <QtGui/QVBoxLayout>

QT_BEGIN_NAMESPACE

class Ui_cbLabDialog
{
public:
    QGridLayout *gridLayout;
    QTreeView *treeView;
    QGroupBox *groupBox_2;
    QVBoxLayout *verticalLayout_2;
    QLabel *label_Lab;
    QGraphicsView *graphicsView_Lab;
    QGroupBox *groupBox;
    QGridLayout *gridLayout_2;
    QLabel *label_Preview;
    QGraphicsView *graphicsView_Preview;
    QPushButton *pushButton_Apply;
    QDialogButtonBox *buttonBox;

    void setupUi(QDialog *cbLabDialog)
    {
        if (cbLabDialog->objectName().isEmpty())
            cbLabDialog->setObjectName(QString::fromUtf8("cbLabDialog"));
        cbLabDialog->resize(750, 360);
        QIcon icon;
        icon.addFile(QString::fromUtf8(":/images/buttons/lab"), QSize(), QIcon::Normal, QIcon::Off);
        cbLabDialog->setWindowIcon(icon);
        gridLayout = new QGridLayout(cbLabDialog);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        treeView = new QTreeView(cbLabDialog);
        treeView->setObjectName(QString::fromUtf8("treeView"));
        QSizePolicy sizePolicy(QSizePolicy::Minimum, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(treeView->sizePolicy().hasHeightForWidth());
        treeView->setSizePolicy(sizePolicy);

        gridLayout->addWidget(treeView, 0, 1, 1, 1);

        groupBox_2 = new QGroupBox(cbLabDialog);
        groupBox_2->setObjectName(QString::fromUtf8("groupBox_2"));
        verticalLayout_2 = new QVBoxLayout(groupBox_2);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        label_Lab = new QLabel(groupBox_2);
        label_Lab->setObjectName(QString::fromUtf8("label_Lab"));
        QFont font;
        font.setPointSize(12);
        label_Lab->setFont(font);
        label_Lab->setAlignment(Qt::AlignCenter);

        verticalLayout_2->addWidget(label_Lab);

        graphicsView_Lab = new QGraphicsView(groupBox_2);
        graphicsView_Lab->setObjectName(QString::fromUtf8("graphicsView_Lab"));
        graphicsView_Lab->setMinimumSize(QSize(320, 200));

        verticalLayout_2->addWidget(graphicsView_Lab);


        gridLayout->addWidget(groupBox_2, 0, 2, 2, 1);

        groupBox = new QGroupBox(cbLabDialog);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        gridLayout_2 = new QGridLayout(groupBox);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        label_Preview = new QLabel(groupBox);
        label_Preview->setObjectName(QString::fromUtf8("label_Preview"));
        label_Preview->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(label_Preview, 0, 0, 1, 1);

        graphicsView_Preview = new QGraphicsView(groupBox);
        graphicsView_Preview->setObjectName(QString::fromUtf8("graphicsView_Preview"));
        QSizePolicy sizePolicy1(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(graphicsView_Preview->sizePolicy().hasHeightForWidth());
        graphicsView_Preview->setSizePolicy(sizePolicy1);
        graphicsView_Preview->setMinimumSize(QSize(200, 125));
        graphicsView_Preview->setMaximumSize(QSize(200, 160));

        gridLayout_2->addWidget(graphicsView_Preview, 1, 0, 1, 1);

        pushButton_Apply = new QPushButton(groupBox);
        pushButton_Apply->setObjectName(QString::fromUtf8("pushButton_Apply"));
        pushButton_Apply->setEnabled(false);
        QSizePolicy sizePolicy2(QSizePolicy::Minimum, QSizePolicy::Minimum);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(pushButton_Apply->sizePolicy().hasHeightForWidth());
        pushButton_Apply->setSizePolicy(sizePolicy2);

        gridLayout_2->addWidget(pushButton_Apply, 1, 1, 1, 1);


        gridLayout->addWidget(groupBox, 1, 1, 1, 1);

        buttonBox = new QDialogButtonBox(cbLabDialog);
        buttonBox->setObjectName(QString::fromUtf8("buttonBox"));
        buttonBox->setOrientation(Qt::Horizontal);
        buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Ok|QDialogButtonBox::RestoreDefaults);
        buttonBox->setCenterButtons(false);

        gridLayout->addWidget(buttonBox, 3, 1, 1, 2);


        retranslateUi(cbLabDialog);
        QObject::connect(buttonBox, SIGNAL(accepted()), cbLabDialog, SLOT(accept()));
        QObject::connect(buttonBox, SIGNAL(rejected()), cbLabDialog, SLOT(reject()));

        QMetaObject::connectSlotsByName(cbLabDialog);
    } // setupUi

    void retranslateUi(QDialog *cbLabDialog)
    {
        cbLabDialog->setWindowTitle(QApplication::translate("cbLabDialog", "Change Lab", 0, QApplication::UnicodeUTF8));
        groupBox_2->setTitle(QApplication::translate("cbLabDialog", "Lab Configuration", 0, QApplication::UnicodeUTF8));
        label_Lab->setText(QString());
        groupBox->setTitle(QApplication::translate("cbLabDialog", "Preview", 0, QApplication::UnicodeUTF8));
        label_Preview->setText(QString());
#ifndef QT_NO_TOOLTIP
        pushButton_Apply->setToolTip(QApplication::translate("cbLabDialog", "Set As Current Configuration", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        pushButton_Apply->setText(QApplication::translate("cbLabDialog", ">>", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class cbLabDialog: public Ui_cbLabDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_CBLABDIALOG_H

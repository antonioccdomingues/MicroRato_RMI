/********************************************************************************
** Form generated from reading UI file 'logplayerGUI.ui'
**
** Created by: Qt User Interface Compiler version 4.8.7
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_LOGPLAYERGUI_H
#define UI_LOGPLAYERGUI_H

#include <Qt3Support/Q3MimeSourceFactory>
#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QPushButton>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_logplayerGUI
{
public:
    QWidget *Layout3;
    QVBoxLayout *vlayout1;
    QLabel *TextLabel2;
    QLabel *TextLabel3;
    QPushButton *QuitButton;

    void setupUi(QWidget *logplayerGUI)
    {
        if (logplayerGUI->objectName().isEmpty())
            logplayerGUI->setObjectName(QString::fromUtf8("logplayerGUI"));
        logplayerGUI->resize(350, 144);
        Layout3 = new QWidget(logplayerGUI);
        Layout3->setObjectName(QString::fromUtf8("Layout3"));
        Layout3->setGeometry(QRect(10, 10, 272, 65));
        vlayout1 = new QVBoxLayout(Layout3);
        vlayout1->setSpacing(3);
        vlayout1->setContentsMargins(0, 0, 0, 0);
        vlayout1->setObjectName(QString::fromUtf8("vlayout1"));
        vlayout1->setContentsMargins(0, 0, 0, 0);
        TextLabel2 = new QLabel(Layout3);
        TextLabel2->setObjectName(QString::fromUtf8("TextLabel2"));
        TextLabel2->setWordWrap(false);

        vlayout1->addWidget(TextLabel2);

        TextLabel3 = new QLabel(Layout3);
        TextLabel3->setObjectName(QString::fromUtf8("TextLabel3"));
        TextLabel3->setWordWrap(false);

        vlayout1->addWidget(TextLabel3);

        QuitButton = new QPushButton(logplayerGUI);
        QuitButton->setObjectName(QString::fromUtf8("QuitButton"));
        QuitButton->setGeometry(QRect(140, 100, 80, 35));

        retranslateUi(logplayerGUI);

        QMetaObject::connectSlotsByName(logplayerGUI);
    } // setupUi

    void retranslateUi(QWidget *logplayerGUI)
    {
        logplayerGUI->setWindowTitle(QApplication::translate("logplayerGUI", "CiberRato Logplayer", 0, QApplication::UnicodeUTF8));
        TextLabel2->setText(QApplication::translate("logplayerGUI", "Universidade de Aveiro - 2015", 0, QApplication::UnicodeUTF8));
        TextLabel3->setText(QApplication::translate("logplayerGUI", "http://microrato.ua.pt", 0, QApplication::UnicodeUTF8));
        QuitButton->setText(QApplication::translate("logplayerGUI", "Quit", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class logplayerGUI: public Ui_logplayerGUI {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_LOGPLAYERGUI_H

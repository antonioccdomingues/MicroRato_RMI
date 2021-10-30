/****************************************************************************
** Meta object code from reading C++ file 'cbsimulatorGUI.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.7)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "cbsimulatorGUI.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'cbsimulatorGUI.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.7. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_cbSimulatorGUI[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      19,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      16,   15,   15,   15, 0x08,
      32,   15,   15,   15, 0x08,
      51,   15,   15,   15, 0x08,
      81,   15,   15,   15, 0x08,
     121,   15,   15,   15, 0x08,
     161,   15,   15,   15, 0x08,
     201,   15,   15,   15, 0x08,
     246,  238,   15,   15, 0x08,
     271,   15,   15,   15, 0x08,
     307,  302,   15,   15, 0x08,
     344,   15,   15,   15, 0x08,
     389,  387,   15,   15, 0x08,
     432,   15,   15,   15, 0x08,
     453,   15,   15,   15, 0x08,
     471,   15,   15,   15, 0x08,
     492,   15,   15,   15, 0x08,
     519,  513,   15,   15, 0x08,
     562,   15,   15,   15, 0x08,
     598,   15,   15,   15, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_cbSimulatorGUI[] = {
    "cbSimulatorGUI\0\0setSimTime(int)\0"
    "setRemainTime(int)\0on_actionOpen_Lab_triggered()\0"
    "on_actionEdit_Configuration_triggered()\0"
    "on_actionOpen_Configuration_triggered()\0"
    "on_actionSave_Configuration_triggered()\0"
    "on_actionAbout_Simulator_triggered()\0"
    "running\0setStartStopButton(bool)\0"
    "on_actionStartStop_triggered()\0arg1\0"
    "on_actionLaunch_Viewer_toggled(bool)\0"
    "processViewerError(QProcess::ProcessError)\0"
    ",\0handleViewerExit(int,QProcess::ExitStatus)\0"
    "resetRobWidgets(int)\0addRobWidget(int)\0"
    "removeRobWidget(int)\0refreshRobComboBox()\0"
    "index\0on_comboBox_Robot_currentIndexChanged(int)\0"
    "on_actionSimulator_Help_triggered()\0"
    "on_actionLoad_Defaults_triggered()\0"
};

void cbSimulatorGUI::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        cbSimulatorGUI *_t = static_cast<cbSimulatorGUI *>(_o);
        switch (_id) {
        case 0: _t->setSimTime((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 1: _t->setRemainTime((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 2: _t->on_actionOpen_Lab_triggered(); break;
        case 3: _t->on_actionEdit_Configuration_triggered(); break;
        case 4: _t->on_actionOpen_Configuration_triggered(); break;
        case 5: _t->on_actionSave_Configuration_triggered(); break;
        case 6: _t->on_actionAbout_Simulator_triggered(); break;
        case 7: _t->setStartStopButton((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 8: _t->on_actionStartStop_triggered(); break;
        case 9: _t->on_actionLaunch_Viewer_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 10: _t->processViewerError((*reinterpret_cast< QProcess::ProcessError(*)>(_a[1]))); break;
        case 11: _t->handleViewerExit((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< QProcess::ExitStatus(*)>(_a[2]))); break;
        case 12: _t->resetRobWidgets((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 13: _t->addRobWidget((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 14: _t->removeRobWidget((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 15: _t->refreshRobComboBox(); break;
        case 16: _t->on_comboBox_Robot_currentIndexChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 17: _t->on_actionSimulator_Help_triggered(); break;
        case 18: _t->on_actionLoad_Defaults_triggered(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData cbSimulatorGUI::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject cbSimulatorGUI::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_cbSimulatorGUI,
      qt_meta_data_cbSimulatorGUI, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &cbSimulatorGUI::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *cbSimulatorGUI::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *cbSimulatorGUI::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_cbSimulatorGUI))
        return static_cast<void*>(const_cast< cbSimulatorGUI*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int cbSimulatorGUI::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 19)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 19;
    }
    return _id;
}
QT_END_MOC_NAMESPACE

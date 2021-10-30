/****************************************************************************
** Meta object code from reading C++ file 'cbmanagerobots.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.7)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "cbmanagerobots.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'cbmanagerobots.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.7. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_cbManageRobots[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      13,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: signature, parameters, type, tag, flags
      16,   15,   15,   15, 0x05,

 // slots: signature, parameters, type, tag, flags
      25,   15,   15,   15, 0x08,
      46,   15,   15,   15, 0x08,
      70,   67,   15,   15, 0x08,
      88,   67,   15,   15, 0x08,
     111,  109,   15,   15, 0x08,
     177,  156,   15,   15, 0x08,
     201,   15,   15,   15, 0x08,
     235,   15,   15,   15, 0x08,
     277,  271,   15,   15, 0x08,
     323,   15,   15,   15, 0x08,
     356,   15,   15,   15, 0x08,
     401,   15,   15,   15, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_cbManageRobots[] = {
    "cbManageRobots\0\0closed()\0resetRobWidgets(int)\0"
    "refreshPosComboBox()\0id\0addRobWidget(int)\0"
    "removeRobWidget(int)\0,\0"
    "currentAgentChanged(QModelIndex,QModelIndex)\0"
    "registrationsAllowed\0tryEnableAddRobot(bool)\0"
    "on_pushButton_LoadAgent_clicked()\0"
    "on_pushButton_RemoveAgent_clicked()\0"
    "index\0on_comboBox_Position_currentIndexChanged(int)\0"
    "on_pushButton_AddRobot_clicked()\0"
    "on_tableWidget_Robots_itemSelectionChanged()\0"
    "on_pushButton_DeleteRobot_clicked()\0"
};

void cbManageRobots::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        cbManageRobots *_t = static_cast<cbManageRobots *>(_o);
        switch (_id) {
        case 0: _t->closed(); break;
        case 1: _t->resetRobWidgets((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 2: _t->refreshPosComboBox(); break;
        case 3: _t->addRobWidget((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 4: _t->removeRobWidget((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 5: _t->currentAgentChanged((*reinterpret_cast< QModelIndex(*)>(_a[1])),(*reinterpret_cast< QModelIndex(*)>(_a[2]))); break;
        case 6: _t->tryEnableAddRobot((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 7: _t->on_pushButton_LoadAgent_clicked(); break;
        case 8: _t->on_pushButton_RemoveAgent_clicked(); break;
        case 9: _t->on_comboBox_Position_currentIndexChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 10: _t->on_pushButton_AddRobot_clicked(); break;
        case 11: _t->on_tableWidget_Robots_itemSelectionChanged(); break;
        case 12: _t->on_pushButton_DeleteRobot_clicked(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData cbManageRobots::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject cbManageRobots::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_cbManageRobots,
      qt_meta_data_cbManageRobots, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &cbManageRobots::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *cbManageRobots::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *cbManageRobots::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_cbManageRobots))
        return static_cast<void*>(const_cast< cbManageRobots*>(this));
    return QWidget::qt_metacast(_clname);
}

int cbManageRobots::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 13)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 13;
    }
    return _id;
}

// SIGNAL 0
void cbManageRobots::closed()
{
    QMetaObject::activate(this, &staticMetaObject, 0, 0);
}
QT_END_MOC_NAMESPACE

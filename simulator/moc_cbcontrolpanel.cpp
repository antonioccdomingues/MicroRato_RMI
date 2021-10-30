/****************************************************************************
** Meta object code from reading C++ file 'cbcontrolpanel.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.7)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "cbcontrolpanel.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'cbcontrolpanel.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.7. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_cbControlPanel[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: signature, parameters, type, tag, flags
      16,   15,   15,   15, 0x05,

 // slots: signature, parameters, type, tag, flags
      33,   25,   15,   15, 0x08,
      58,   15,   15,   15, 0x08,
      77,   15,   15,   15, 0x08,
     106,   98,   15,   15, 0x08,
     147,   15,   15,   15, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_cbControlPanel[] = {
    "cbControlPanel\0\0closed()\0running\0"
    "setStartStopButton(bool)\0triggerStartStop()\0"
    "resetRobSpinBox(int)\0checked\0"
    "on_pushButton_ManageRobots_toggled(bool)\0"
    "on_pushButton_Remove_clicked()\0"
};

void cbControlPanel::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        cbControlPanel *_t = static_cast<cbControlPanel *>(_o);
        switch (_id) {
        case 0: _t->closed(); break;
        case 1: _t->setStartStopButton((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 2: _t->triggerStartStop(); break;
        case 3: _t->resetRobSpinBox((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 4: _t->on_pushButton_ManageRobots_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 5: _t->on_pushButton_Remove_clicked(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData cbControlPanel::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject cbControlPanel::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_cbControlPanel,
      qt_meta_data_cbControlPanel, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &cbControlPanel::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *cbControlPanel::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *cbControlPanel::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_cbControlPanel))
        return static_cast<void*>(const_cast< cbControlPanel*>(this));
    return QWidget::qt_metacast(_clname);
}

int cbControlPanel::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 6)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 6;
    }
    return _id;
}

// SIGNAL 0
void cbControlPanel::closed()
{
    QMetaObject::activate(this, &staticMetaObject, 0, 0);
}
QT_END_MOC_NAMESPACE

/****************************************************************************
** Meta object code from reading C++ file 'cbrobot.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.7)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "cbrobot.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'cbrobot.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.7. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_cbRobot[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       6,       // signalCount

 // signals: signature, parameters, type, tag, flags
       9,    8,    8,    8, 0x05,
      35,    8,    8,    8, 0x05,
      61,    8,    8,    8, 0x05,
      88,    8,    8,    8, 0x05,
     109,    8,    8,    8, 0x05,
     141,    8,    8,    8, 0x05,

       0        // eod
};

static const char qt_meta_stringdata_cbRobot[] = {
    "cbRobot\0\0robCollisionsChanged(int)\0"
    "robReturnTimeChanged(int)\0"
    "robArrivalTimeChanged(int)\0"
    "robScoreChanged(int)\0"
    "robStateChanged(cbRobot::State)\0"
    "robStateChanged(QString)\0"
};

void cbRobot::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        cbRobot *_t = static_cast<cbRobot *>(_o);
        switch (_id) {
        case 0: _t->robCollisionsChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 1: _t->robReturnTimeChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 2: _t->robArrivalTimeChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 3: _t->robScoreChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 4: _t->robStateChanged((*reinterpret_cast< cbRobot::State(*)>(_a[1]))); break;
        case 5: _t->robStateChanged((*reinterpret_cast< QString(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData cbRobot::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject cbRobot::staticMetaObject = {
    { &cbClient::staticMetaObject, qt_meta_stringdata_cbRobot,
      qt_meta_data_cbRobot, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &cbRobot::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *cbRobot::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *cbRobot::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_cbRobot))
        return static_cast<void*>(const_cast< cbRobot*>(this));
    return cbClient::qt_metacast(_clname);
}

int cbRobot::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = cbClient::qt_metacall(_c, _id, _a);
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
void cbRobot::robCollisionsChanged(int _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void cbRobot::robReturnTimeChanged(int _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void cbRobot::robArrivalTimeChanged(int _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void cbRobot::robScoreChanged(int _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void cbRobot::robStateChanged(cbRobot::State _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}

// SIGNAL 5
void cbRobot::robStateChanged(QString _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 5, _a);
}
QT_END_MOC_NAMESPACE

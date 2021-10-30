/****************************************************************************
** Meta object code from reading C++ file 'cbsimulator.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.7)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "cbsimulator.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'cbsimulator.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.7. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_cbSimulator[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      31,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
      16,       // signalCount

 // signals: signature, parameters, type, tag, flags
      13,   12,   12,   12, 0x05,
      29,   12,   12,   12, 0x05,
      54,   12,   12,   12, 0x05,
      78,   12,   12,   12, 0x05,
      95,   12,   12,   12, 0x05,
     118,   12,   12,   12, 0x05,
     151,  144,   12,   12, 0x05,
     171,   12,   12,   12, 0x05,
     197,  188,   12,   12, 0x05,
     219,   12,   12,   12, 0x05,
     234,   12,   12,   12, 0x05,
     249,   12,   12,   12, 0x05,
     266,   12,   12,   12, 0x05,
     286,   12,   12,   12, 0x05,
     306,   12,   12,   12, 0x05,
     327,   12,   12,   12, 0x05,

 // slots: signature, parameters, type, tag, flags
     345,   12,   12,   12, 0x0a,
     352,   12,   12,   12, 0x0a,
     360,   12,   12,   12, 0x0a,
     368,   12,   12,   12, 0x0a,
     377,  375,   12,   12, 0x0a,
     398,  375,   12,   12, 0x0a,
     411,  375,   12,   12, 0x0a,
     433,   12,   12,   12, 0x0a,
     447,   12,   12,   12, 0x0a,
     467,   12,   12,   12, 0x0a,
     493,  490,   12,   12, 0x0a,
     511,   12,   12,   12, 0x0a,
     532,   12,   12,   12, 0x0a,
     554,   12,   12,   12, 0x0a,
     577,   12,   12,   12, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_cbSimulator[] = {
    "cbSimulator\0\0toggleGPS(bool)\0"
    "toggleBeaconSensor(bool)\0"
    "toggleScoreSensor(bool)\0toggleTime(bool)\0"
    "toggleCollisions(bool)\0toggleRegistrations(bool)\0"
    "newLab\0labChanged(QString)\0gridChanged(int)\0"
    "newState\0stateChanged(QString)\0"
    "simReset(bool)\0simReady(bool)\0"
    "simRunning(bool)\0curTimeChanged(int)\0"
    "simTimeChanged(int)\0robotRegistered(int)\0"
    "robotDeleted(int)\0step()\0reset()\0"
    "start()\0stop()\0g\0setScoreSensor(bool)\0"
    "setGPS(bool)\0setBeaconSensor(bool)\0"
    "setTime(bool)\0setCollisions(bool)\0"
    "setRegistrations(bool)\0id\0deleteRobot(uint)\0"
    "setShowActions(bool)\0setShowMeasures(bool)\0"
    "setShowPositions(bool)\0"
    "printTargetRelativePositions()\0"
};

void cbSimulator::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        cbSimulator *_t = static_cast<cbSimulator *>(_o);
        switch (_id) {
        case 0: _t->toggleGPS((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 1: _t->toggleBeaconSensor((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 2: _t->toggleScoreSensor((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 3: _t->toggleTime((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 4: _t->toggleCollisions((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 5: _t->toggleRegistrations((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 6: _t->labChanged((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 7: _t->gridChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 8: _t->stateChanged((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 9: _t->simReset((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 10: _t->simReady((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 11: _t->simRunning((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 12: _t->curTimeChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 13: _t->simTimeChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 14: _t->robotRegistered((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 15: _t->robotDeleted((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 16: _t->step(); break;
        case 17: _t->reset(); break;
        case 18: _t->start(); break;
        case 19: _t->stop(); break;
        case 20: _t->setScoreSensor((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 21: _t->setGPS((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 22: _t->setBeaconSensor((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 23: _t->setTime((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 24: _t->setCollisions((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 25: _t->setRegistrations((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 26: _t->deleteRobot((*reinterpret_cast< uint(*)>(_a[1]))); break;
        case 27: _t->setShowActions((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 28: _t->setShowMeasures((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 29: _t->setShowPositions((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 30: _t->printTargetRelativePositions(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData cbSimulator::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject cbSimulator::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_cbSimulator,
      qt_meta_data_cbSimulator, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &cbSimulator::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *cbSimulator::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *cbSimulator::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_cbSimulator))
        return static_cast<void*>(const_cast< cbSimulator*>(this));
    return QObject::qt_metacast(_clname);
}

int cbSimulator::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 31)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 31;
    }
    return _id;
}

// SIGNAL 0
void cbSimulator::toggleGPS(bool _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void cbSimulator::toggleBeaconSensor(bool _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void cbSimulator::toggleScoreSensor(bool _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void cbSimulator::toggleTime(bool _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void cbSimulator::toggleCollisions(bool _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}

// SIGNAL 5
void cbSimulator::toggleRegistrations(bool _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 5, _a);
}

// SIGNAL 6
void cbSimulator::labChanged(const QString _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 6, _a);
}

// SIGNAL 7
void cbSimulator::gridChanged(int _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 7, _a);
}

// SIGNAL 8
void cbSimulator::stateChanged(const QString _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 8, _a);
}

// SIGNAL 9
void cbSimulator::simReset(bool _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 9, _a);
}

// SIGNAL 10
void cbSimulator::simReady(bool _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 10, _a);
}

// SIGNAL 11
void cbSimulator::simRunning(bool _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 11, _a);
}

// SIGNAL 12
void cbSimulator::curTimeChanged(int _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 12, _a);
}

// SIGNAL 13
void cbSimulator::simTimeChanged(int _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 13, _a);
}

// SIGNAL 14
void cbSimulator::robotRegistered(int _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 14, _a);
}

// SIGNAL 15
void cbSimulator::robotDeleted(int _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 15, _a);
}
QT_END_MOC_NAMESPACE

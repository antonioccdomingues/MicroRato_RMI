/****************************************************************************
** Meta object code from reading C++ file 'crqrobotinfo.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.7)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "crqrobotinfo.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'crqrobotinfo.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.7. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_CRQRobotInfo[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       5,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       5,       // signalCount

 // signals: signature, parameters, type, tag, flags
      14,   13,   13,   13, 0x05,
      25,   13,   13,   13, 0x05,
      41,   13,   13,   13, 0x05,
      57,   13,   13,   13, 0x05,
      72,   13,   13,   13, 0x05,

       0        // eod
};

static const char qt_meta_stringdata_CRQRobotInfo[] = {
    "CRQRobotInfo\0\0score(int)\0returnTime(int)\0"
    "collisions(int)\0state(QPixmap)\0time(int)\0"
};

void CRQRobotInfo::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        CRQRobotInfo *_t = static_cast<CRQRobotInfo *>(_o);
        switch (_id) {
        case 0: _t->score((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 1: _t->returnTime((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 2: _t->collisions((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 3: _t->state((*reinterpret_cast< const QPixmap(*)>(_a[1]))); break;
        case 4: _t->time((*reinterpret_cast< int(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData CRQRobotInfo::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject CRQRobotInfo::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_CRQRobotInfo,
      qt_meta_data_CRQRobotInfo, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &CRQRobotInfo::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *CRQRobotInfo::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *CRQRobotInfo::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_CRQRobotInfo))
        return static_cast<void*>(const_cast< CRQRobotInfo*>(this));
    return QWidget::qt_metacast(_clname);
}

int CRQRobotInfo::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 5)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 5;
    }
    return _id;
}

// SIGNAL 0
void CRQRobotInfo::score(int _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void CRQRobotInfo::returnTime(int _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void CRQRobotInfo::collisions(int _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void CRQRobotInfo::state(const QPixmap & _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void CRQRobotInfo::time(int _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}
QT_END_MOC_NAMESPACE

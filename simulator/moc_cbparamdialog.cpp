/****************************************************************************
** Meta object code from reading C++ file 'cbparamdialog.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.7)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "cbparamdialog.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'cbparamdialog.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.7. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_cbParamDialog[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       2,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      15,   14,   14,   14, 0x09,
      24,   14,   14,   14, 0x09,

       0        // eod
};

static const char qt_meta_stringdata_cbParamDialog[] = {
    "cbParamDialog\0\0accept()\0restoreDefaults()\0"
};

void cbParamDialog::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        cbParamDialog *_t = static_cast<cbParamDialog *>(_o);
        switch (_id) {
        case 0: _t->accept(); break;
        case 1: _t->restoreDefaults(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

const QMetaObjectExtraData cbParamDialog::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject cbParamDialog::staticMetaObject = {
    { &QDialog::staticMetaObject, qt_meta_stringdata_cbParamDialog,
      qt_meta_data_cbParamDialog, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &cbParamDialog::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *cbParamDialog::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *cbParamDialog::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_cbParamDialog))
        return static_cast<void*>(const_cast< cbParamDialog*>(this));
    return QDialog::qt_metacast(_clname);
}

int cbParamDialog::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QDialog::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 2)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 2;
    }
    return _id;
}
QT_END_MOC_NAMESPACE

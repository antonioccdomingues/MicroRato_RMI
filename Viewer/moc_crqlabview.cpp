/****************************************************************************
** Meta object code from reading C++ file 'crqlabview.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.7)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "crqlabview.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'crqlabview.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.7. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_CRQLabView[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       8,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      12,   11,   11,   11, 0x08,
      54,   46,   11,   11, 0x08,
      85,   11,   11,   11, 0x08,
     112,   11,   11,   11, 0x08,
     151,   11,   11,   11, 0x08,
     200,  192,   11,   11, 0x08,
     236,   11,   11,   11, 0x08,
     269,   11,   11,   11, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_CRQLabView[] = {
    "CRQLabView\0\0on_actionReset_Viewer_triggered()\0"
    "connect\0on_actionConnect_toggled(bool)\0"
    "on_actionAbout_triggered()\0"
    "on_actionLower_walls_color_triggered()\0"
    "on_actionHigher_walls_collor_triggered()\0"
    "soundON\0on_actionSound_On_Off_toggled(bool)\0"
    "on_actionChange_skin_triggered()\0"
    "on_actionStart_triggered()\0"
};

void CRQLabView::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        CRQLabView *_t = static_cast<CRQLabView *>(_o);
        switch (_id) {
        case 0: _t->on_actionReset_Viewer_triggered(); break;
        case 1: _t->on_actionConnect_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 2: _t->on_actionAbout_triggered(); break;
        case 3: _t->on_actionLower_walls_color_triggered(); break;
        case 4: _t->on_actionHigher_walls_collor_triggered(); break;
        case 5: _t->on_actionSound_On_Off_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 6: _t->on_actionChange_skin_triggered(); break;
        case 7: _t->on_actionStart_triggered(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData CRQLabView::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject CRQLabView::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_CRQLabView,
      qt_meta_data_CRQLabView, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &CRQLabView::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *CRQLabView::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *CRQLabView::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_CRQLabView))
        return static_cast<void*>(const_cast< CRQLabView*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int CRQLabView::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 8)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 8;
    }
    return _id;
}
QT_END_MOC_NAMESPACE

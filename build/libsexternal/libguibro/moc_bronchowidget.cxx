/****************************************************************************
** Meta object code from reading C++ file 'bronchowidget.h'
**
** Created: Thu Apr 10 11:49:56 2014
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../libsexternal/libguibro/bronchowidget.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'bronchowidget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_GUIBRO__bronchoWidget[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       9,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      27,   23,   22,   22, 0x08,
      51,   23,   22,   22, 0x08,
      72,   23,   22,   22, 0x08,
      95,   22,   22,   22, 0x08,
     116,   22,   22,   22, 0x08,
     143,  137,   22,   22, 0x08,
     159,  137,   22,   22, 0x08,
     178,   22,   22,   22, 0x08,
     195,   22,   22,   22, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_GUIBRO__bronchoWidget[] = {
    "GUIBRO::bronchoWidget\0\0val\0"
    "alphaSliderChanged(int)\0xiSliderChanged(int)\0"
    "zetaSliderChanged(int)\0zetaSliderChanged1()\0"
    "zetaSliderReleased()\0state\0setNavMode(int)\0"
    "setCameraMode(int)\0collisionCheck()\0"
    "advanceBronchoscope()\0"
};

void GUIBRO::bronchoWidget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        bronchoWidget *_t = static_cast<bronchoWidget *>(_o);
        switch (_id) {
        case 0: _t->alphaSliderChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 1: _t->xiSliderChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 2: _t->zetaSliderChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 3: _t->zetaSliderChanged1(); break;
        case 4: _t->zetaSliderReleased(); break;
        case 5: _t->setNavMode((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 6: _t->setCameraMode((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 7: _t->collisionCheck(); break;
        case 8: _t->advanceBronchoscope(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData GUIBRO::bronchoWidget::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject GUIBRO::bronchoWidget::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_GUIBRO__bronchoWidget,
      qt_meta_data_GUIBRO__bronchoWidget, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &GUIBRO::bronchoWidget::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *GUIBRO::bronchoWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *GUIBRO::bronchoWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_GUIBRO__bronchoWidget))
        return static_cast<void*>(const_cast< bronchoWidget*>(this));
    return QWidget::qt_metacast(_clname);
}

int GUIBRO::bronchoWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 9)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 9;
    }
    return _id;
}
QT_END_MOC_NAMESPACE

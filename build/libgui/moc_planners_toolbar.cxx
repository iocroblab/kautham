/****************************************************************************
** Meta object code from reading C++ file 'planners_toolbar.h'
**
** Created: Thu Apr 10 11:50:33 2014
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../libgui/planners_toolbar.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'planners_toolbar.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_Kautham__PlannerToolBar[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       2,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: signature, parameters, type, tag, flags
      34,   25,   24,   24, 0x05,

 // slots: signature, parameters, type, tag, flags
      60,   24,   24,   24, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_Kautham__PlannerToolBar[] = {
    "Kautham::PlannerToolBar\0\0loc,glob\0"
    "addPlanner(string,string)\0pushAdd()\0"
};

void Kautham::PlannerToolBar::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        PlannerToolBar *_t = static_cast<PlannerToolBar *>(_o);
        switch (_id) {
        case 0: _t->addPlanner((*reinterpret_cast< string(*)>(_a[1])),(*reinterpret_cast< string(*)>(_a[2]))); break;
        case 1: _t->pushAdd(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData Kautham::PlannerToolBar::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject Kautham::PlannerToolBar::staticMetaObject = {
    { &QToolBar::staticMetaObject, qt_meta_stringdata_Kautham__PlannerToolBar,
      qt_meta_data_Kautham__PlannerToolBar, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &Kautham::PlannerToolBar::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *Kautham::PlannerToolBar::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *Kautham::PlannerToolBar::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_Kautham__PlannerToolBar))
        return static_cast<void*>(const_cast< PlannerToolBar*>(this));
    return QToolBar::qt_metacast(_clname);
}

int Kautham::PlannerToolBar::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QToolBar::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 2)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 2;
    }
    return _id;
}

// SIGNAL 0
void Kautham::PlannerToolBar::addPlanner(string _t1, string _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_END_MOC_NAMESPACE

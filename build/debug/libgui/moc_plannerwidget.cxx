/****************************************************************************
** Meta object code from reading C++ file 'plannerwidget.h'
**
** Created: Tue Mar 18 11:05:54 2014
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../libgui/plannerwidget.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'plannerwidget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_Kautham__PlannerWidget[] = {

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
      24,   23,   23,   23, 0x08,
      38,   23,   23,   23, 0x08,
      53,   23,   23,   23, 0x08,
      68,   23,   23,   23, 0x08,
      90,   84,   23,   23, 0x08,
     106,   23,   23,   23, 0x08,
     119,   23,   23,   23, 0x08,
     136,   23,   23,   23, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_Kautham__PlannerWidget[] = {
    "Kautham::PlannerWidget\0\0getPathCall()\0"
    "saveDataCall()\0loadDataCall()\0"
    "moveAlongPath()\0index\0showSample(int)\0"
    "tryConnect()\0chkCameraClick()\0"
    "simulatePath()\0"
};

void Kautham::PlannerWidget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        PlannerWidget *_t = static_cast<PlannerWidget *>(_o);
        switch (_id) {
        case 0: _t->getPathCall(); break;
        case 1: _t->saveDataCall(); break;
        case 2: _t->loadDataCall(); break;
        case 3: _t->moveAlongPath(); break;
        case 4: _t->showSample((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 5: _t->tryConnect(); break;
        case 6: _t->chkCameraClick(); break;
        case 7: _t->simulatePath(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData Kautham::PlannerWidget::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject Kautham::PlannerWidget::staticMetaObject = {
    { &KauthamWidget::staticMetaObject, qt_meta_stringdata_Kautham__PlannerWidget,
      qt_meta_data_Kautham__PlannerWidget, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &Kautham::PlannerWidget::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *Kautham::PlannerWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *Kautham::PlannerWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_Kautham__PlannerWidget))
        return static_cast<void*>(const_cast< PlannerWidget*>(this));
    return KauthamWidget::qt_metacast(_clname);
}

int Kautham::PlannerWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = KauthamWidget::qt_metacall(_c, _id, _a);
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

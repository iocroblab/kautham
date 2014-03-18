/****************************************************************************
** Meta object code from reading C++ file 'invkinwidget.h'
**
** Created: Tue Mar 18 11:05:54 2014
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../libgui/invkinwidget.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'invkinwidget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_Kautham__InvKinWidget[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       1,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      23,   22,   22,   22, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_Kautham__InvKinWidget[] = {
    "Kautham::InvKinWidget\0\0getSolution()\0"
};

void Kautham::InvKinWidget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        InvKinWidget *_t = static_cast<InvKinWidget *>(_o);
        switch (_id) {
        case 0: _t->getSolution(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

const QMetaObjectExtraData Kautham::InvKinWidget::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject Kautham::InvKinWidget::staticMetaObject = {
    { &KauthamWidget::staticMetaObject, qt_meta_stringdata_Kautham__InvKinWidget,
      qt_meta_data_Kautham__InvKinWidget, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &Kautham::InvKinWidget::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *Kautham::InvKinWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *Kautham::InvKinWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_Kautham__InvKinWidget))
        return static_cast<void*>(const_cast< InvKinWidget*>(this));
    return KauthamWidget::qt_metacast(_clname);
}

int Kautham::InvKinWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = KauthamWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 1)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 1;
    }
    return _id;
}
QT_END_MOC_NAMESPACE

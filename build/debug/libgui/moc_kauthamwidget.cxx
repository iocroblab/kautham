/****************************************************************************
** Meta object code from reading C++ file 'kauthamwidget.h'
**
** Created: Tue Mar 18 11:05:53 2014
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../libgui/kauthamwidget.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'kauthamwidget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_Kautham__KauthamWidget[] = {

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
      35,   24,   23,   23, 0x05,

 // slots: signature, parameters, type, tag, flags
      60,   52,   23,   23, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_Kautham__KauthamWidget[] = {
    "Kautham::KauthamWidget\0\0newContent\0"
    "sendText(string)\0row,col\0tableChanged(int,int)\0"
};

void Kautham::KauthamWidget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        KauthamWidget *_t = static_cast<KauthamWidget *>(_o);
        switch (_id) {
        case 0: _t->sendText((*reinterpret_cast< string(*)>(_a[1]))); break;
        case 1: _t->tableChanged((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData Kautham::KauthamWidget::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject Kautham::KauthamWidget::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_Kautham__KauthamWidget,
      qt_meta_data_Kautham__KauthamWidget, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &Kautham::KauthamWidget::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *Kautham::KauthamWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *Kautham::KauthamWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_Kautham__KauthamWidget))
        return static_cast<void*>(const_cast< KauthamWidget*>(this));
    return QWidget::qt_metacast(_clname);
}

int Kautham::KauthamWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
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
void Kautham::KauthamWidget::sendText(string _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_END_MOC_NAMESPACE

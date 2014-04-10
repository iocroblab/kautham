/****************************************************************************
** Meta object code from reading C++ file 'sampleswidget.h'
**
** Created: Thu Apr 10 11:50:35 2014
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../libgui/sampleswidget.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'sampleswidget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_Kautham__SamplesWidget[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      11,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: signature, parameters, type, tag, flags
      35,   24,   23,   23, 0x05,

 // slots: signature, parameters, type, tag, flags
      52,   23,   23,   23, 0x08,
      69,   23,   23,   23, 0x08,
      85,   23,   23,   23, 0x08,
      98,   23,   23,   23, 0x08,
     114,   23,   23,   23, 0x08,
     126,   23,   23,   23, 0x08,
     141,   23,   23,   23, 0x08,
     152,   23,   23,   23, 0x08,
     177,  171,   23,   23, 0x08,
     193,   23,   23,   23, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_Kautham__SamplesWidget[] = {
    "Kautham::SamplesWidget\0\0newContent\0"
    "sendText(string)\0collisionCheck()\0"
    "distanceCheck()\0addCurrent()\0"
    "removeCurrent()\0removeAll()\0removeAllEx2()\0"
    "sampling()\0updateSampleList()\0index\0"
    "showSample(int)\0changeEngine()\0"
};

void Kautham::SamplesWidget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        SamplesWidget *_t = static_cast<SamplesWidget *>(_o);
        switch (_id) {
        case 0: _t->sendText((*reinterpret_cast< string(*)>(_a[1]))); break;
        case 1: _t->collisionCheck(); break;
        case 2: _t->distanceCheck(); break;
        case 3: _t->addCurrent(); break;
        case 4: _t->removeCurrent(); break;
        case 5: _t->removeAll(); break;
        case 6: _t->removeAllEx2(); break;
        case 7: _t->sampling(); break;
        case 8: _t->updateSampleList(); break;
        case 9: _t->showSample((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 10: _t->changeEngine(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData Kautham::SamplesWidget::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject Kautham::SamplesWidget::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_Kautham__SamplesWidget,
      qt_meta_data_Kautham__SamplesWidget, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &Kautham::SamplesWidget::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *Kautham::SamplesWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *Kautham::SamplesWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_Kautham__SamplesWidget))
        return static_cast<void*>(const_cast< SamplesWidget*>(this));
    return QWidget::qt_metacast(_clname);
}

int Kautham::SamplesWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 11)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 11;
    }
    return _id;
}

// SIGNAL 0
void Kautham::SamplesWidget::sendText(string _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_END_MOC_NAMESPACE

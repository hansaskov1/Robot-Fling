/****************************************************************************
** Meta object code from reading C++ file 'Gripper.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.9.5)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../SemesterProjekt/Gripper.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'Gripper.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.9.5. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_Gripper_t {
    QByteArrayData data[12];
    char stringdata0[118];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_Gripper_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_Gripper_t qt_meta_stringdata_Gripper = {
    {
QT_MOC_LITERAL(0, 0, 7), // "Gripper"
QT_MOC_LITERAL(1, 8, 15), // "ToConnectToHost"
QT_MOC_LITERAL(2, 24, 0), // ""
QT_MOC_LITERAL(3, 25, 6), // "hostIp"
QT_MOC_LITERAL(4, 32, 8), // "hostPort"
QT_MOC_LITERAL(5, 41, 10), // "ToSendData"
QT_MOC_LITERAL(6, 52, 4), // "data"
QT_MOC_LITERAL(7, 57, 13), // "ToCloseSocket"
QT_MOC_LITERAL(8, 71, 13), // "ConnectToHost"
QT_MOC_LITERAL(9, 85, 11), // "ReceiveData"
QT_MOC_LITERAL(10, 97, 8), // "SendData"
QT_MOC_LITERAL(11, 106, 11) // "CloseSocket"

    },
    "Gripper\0ToConnectToHost\0\0hostIp\0"
    "hostPort\0ToSendData\0data\0ToCloseSocket\0"
    "ConnectToHost\0ReceiveData\0SendData\0"
    "CloseSocket"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_Gripper[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       7,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    2,   49,    2, 0x06 /* Public */,
       5,    1,   54,    2, 0x06 /* Public */,
       7,    0,   57,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       8,    2,   58,    2, 0x08 /* Private */,
       9,    0,   63,    2, 0x08 /* Private */,
      10,    1,   64,    2, 0x08 /* Private */,
      11,    0,   67,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, QMetaType::QString, QMetaType::UShort,    3,    4,
    QMetaType::Void, QMetaType::QByteArray,    6,
    QMetaType::Void,

 // slots: parameters
    QMetaType::Void, QMetaType::QString, QMetaType::UShort,    3,    4,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QByteArray,    6,
    QMetaType::Void,

       0        // eod
};

void Gripper::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Gripper *_t = static_cast<Gripper *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->ToConnectToHost((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< quint16(*)>(_a[2]))); break;
        case 1: _t->ToSendData((*reinterpret_cast< QByteArray(*)>(_a[1]))); break;
        case 2: _t->ToCloseSocket(); break;
        case 3: _t->ConnectToHost((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< quint16(*)>(_a[2]))); break;
        case 4: _t->ReceiveData(); break;
        case 5: _t->SendData((*reinterpret_cast< QByteArray(*)>(_a[1]))); break;
        case 6: _t->CloseSocket(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            typedef void (Gripper::*_t)(QString , quint16 );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&Gripper::ToConnectToHost)) {
                *result = 0;
                return;
            }
        }
        {
            typedef void (Gripper::*_t)(QByteArray );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&Gripper::ToSendData)) {
                *result = 1;
                return;
            }
        }
        {
            typedef void (Gripper::*_t)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&Gripper::ToCloseSocket)) {
                *result = 2;
                return;
            }
        }
    }
}

const QMetaObject Gripper::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_Gripper.data,
      qt_meta_data_Gripper,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *Gripper::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *Gripper::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_Gripper.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int Gripper::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 7)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 7;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 7)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 7;
    }
    return _id;
}

// SIGNAL 0
void Gripper::ToConnectToHost(QString _t1, quint16 _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void Gripper::ToSendData(QByteArray _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void Gripper::ToCloseSocket()
{
    QMetaObject::activate(this, &staticMetaObject, 2, nullptr);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE

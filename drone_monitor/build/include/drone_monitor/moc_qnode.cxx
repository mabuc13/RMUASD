/****************************************************************************
** Meta object code from reading C++ file 'qnode.hpp'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.7)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../include/drone_monitor/qnode.hpp"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'qnode.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.7. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_drone_monitor__QNode[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      18,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
      18,       // signalCount

 // signals: signature, parameters, type, tag, flags
      22,   21,   21,   21, 0x05,
      39,   21,   21,   21, 0x05,
      61,   53,   21,   21, 0x05,
      84,   77,   21,   21, 0x05,
     104,  100,   21,   21, 0x05,
     129,  124,   21,   21, 0x05,
     150,  146,   21,   21, 0x05,
     173,  169,   21,   21, 0x05,
     205,  197,   21,   21, 0x05,
     229,  225,   21,   21, 0x05,
     259,  252,   21,   21, 0x05,
     287,  278,   21,   21, 0x05,
     313,  308,   21,   21, 0x05,
     335,  308,   21,   21, 0x05,
     361,  308,   21,   21, 0x05,
     442,  388,   21,   21, 0x05,
     504,  498,   21,   21, 0x05,
     534,  529,   21,   21, 0x05,

       0        // eod
};

static const char qt_meta_stringdata_drone_monitor__QNode[] = {
    "drone_monitor::QNode\0\0loggingUpdated()\0"
    "rosShutdown()\0isArmed\0sig_armed(bool)\0"
    "status\0sig_status(int)\0SOC\0"
    "sig_battery(double)\0time\0sig_time(string)\0"
    "sec\0sig_uptime(double)\0RPY\0"
    "sig_RPY(vector<double>)\0heading\0"
    "sig_heading(double)\0pos\0sig_position(gcs::GPS)\0"
    "height\0sig_Height(double)\0velocity\0"
    "sig_velocity(double)\0text\0"
    "sig_autoPilot(string)\0sig_subFlightmode(string)\0"
    "sig_mainFlightmode(string)\0"
    "manual,simulation,stabilized,guided,autoM,test,custom\0"
    "sig_otherFlightmode(bool,bool,bool,bool,bool,bool,bool)\0"
    "state\0sig_mavlinkState(string)\0type\0"
    "sig_mavlinkType(string)\0"
};

void drone_monitor::QNode::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        QNode *_t = static_cast<QNode *>(_o);
        switch (_id) {
        case 0: _t->loggingUpdated(); break;
        case 1: _t->rosShutdown(); break;
        case 2: _t->sig_armed((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 3: _t->sig_status((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 4: _t->sig_battery((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 5: _t->sig_time((*reinterpret_cast< string(*)>(_a[1]))); break;
        case 6: _t->sig_uptime((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 7: _t->sig_RPY((*reinterpret_cast< vector<double>(*)>(_a[1]))); break;
        case 8: _t->sig_heading((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 9: _t->sig_position((*reinterpret_cast< gcs::GPS(*)>(_a[1]))); break;
        case 10: _t->sig_Height((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 11: _t->sig_velocity((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 12: _t->sig_autoPilot((*reinterpret_cast< string(*)>(_a[1]))); break;
        case 13: _t->sig_subFlightmode((*reinterpret_cast< string(*)>(_a[1]))); break;
        case 14: _t->sig_mainFlightmode((*reinterpret_cast< string(*)>(_a[1]))); break;
        case 15: _t->sig_otherFlightmode((*reinterpret_cast< bool(*)>(_a[1])),(*reinterpret_cast< bool(*)>(_a[2])),(*reinterpret_cast< bool(*)>(_a[3])),(*reinterpret_cast< bool(*)>(_a[4])),(*reinterpret_cast< bool(*)>(_a[5])),(*reinterpret_cast< bool(*)>(_a[6])),(*reinterpret_cast< bool(*)>(_a[7]))); break;
        case 16: _t->sig_mavlinkState((*reinterpret_cast< string(*)>(_a[1]))); break;
        case 17: _t->sig_mavlinkType((*reinterpret_cast< string(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData drone_monitor::QNode::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject drone_monitor::QNode::staticMetaObject = {
    { &QThread::staticMetaObject, qt_meta_stringdata_drone_monitor__QNode,
      qt_meta_data_drone_monitor__QNode, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &drone_monitor::QNode::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *drone_monitor::QNode::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *drone_monitor::QNode::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_drone_monitor__QNode))
        return static_cast<void*>(const_cast< QNode*>(this));
    return QThread::qt_metacast(_clname);
}

int drone_monitor::QNode::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QThread::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 18)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 18;
    }
    return _id;
}

// SIGNAL 0
void drone_monitor::QNode::loggingUpdated()
{
    QMetaObject::activate(this, &staticMetaObject, 0, 0);
}

// SIGNAL 1
void drone_monitor::QNode::rosShutdown()
{
    QMetaObject::activate(this, &staticMetaObject, 1, 0);
}

// SIGNAL 2
void drone_monitor::QNode::sig_armed(bool _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void drone_monitor::QNode::sig_status(int _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void drone_monitor::QNode::sig_battery(double _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}

// SIGNAL 5
void drone_monitor::QNode::sig_time(string _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 5, _a);
}

// SIGNAL 6
void drone_monitor::QNode::sig_uptime(double _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 6, _a);
}

// SIGNAL 7
void drone_monitor::QNode::sig_RPY(vector<double> _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 7, _a);
}

// SIGNAL 8
void drone_monitor::QNode::sig_heading(double _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 8, _a);
}

// SIGNAL 9
void drone_monitor::QNode::sig_position(gcs::GPS _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 9, _a);
}

// SIGNAL 10
void drone_monitor::QNode::sig_Height(double _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 10, _a);
}

// SIGNAL 11
void drone_monitor::QNode::sig_velocity(double _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 11, _a);
}

// SIGNAL 12
void drone_monitor::QNode::sig_autoPilot(string _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 12, _a);
}

// SIGNAL 13
void drone_monitor::QNode::sig_subFlightmode(string _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 13, _a);
}

// SIGNAL 14
void drone_monitor::QNode::sig_mainFlightmode(string _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 14, _a);
}

// SIGNAL 15
void drone_monitor::QNode::sig_otherFlightmode(bool _t1, bool _t2, bool _t3, bool _t4, bool _t5, bool _t6, bool _t7)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)), const_cast<void*>(reinterpret_cast<const void*>(&_t4)), const_cast<void*>(reinterpret_cast<const void*>(&_t5)), const_cast<void*>(reinterpret_cast<const void*>(&_t6)), const_cast<void*>(reinterpret_cast<const void*>(&_t7)) };
    QMetaObject::activate(this, &staticMetaObject, 15, _a);
}

// SIGNAL 16
void drone_monitor::QNode::sig_mavlinkState(string _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 16, _a);
}

// SIGNAL 17
void drone_monitor::QNode::sig_mavlinkType(string _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 17, _a);
}
QT_END_MOC_NAMESPACE

/****************************************************************************
** Meta object code from reading C++ file 'main_window.hpp'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.7)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../include/drone_monitor/main_window.hpp"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'main_window.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.7. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_drone_monitor__MainWindow[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      16,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      35,   27,   26,   26, 0x0a,
      58,   51,   26,   26, 0x0a,
      78,   74,   26,   26, 0x0a,
     103,   98,   26,   26, 0x0a,
     124,  120,   26,   26, 0x0a,
     147,  143,   26,   26, 0x0a,
     179,  171,   26,   26, 0x0a,
     203,  199,   26,   26, 0x0a,
     233,  226,   26,   26, 0x0a,
     261,  252,   26,   26, 0x0a,
     287,  282,   26,   26, 0x0a,
     309,  282,   26,   26, 0x0a,
     335,  282,   26,   26, 0x0a,
     416,  362,   26,   26, 0x0a,
     478,  472,   26,   26, 0x0a,
     508,  503,   26,   26, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_drone_monitor__MainWindow[] = {
    "drone_monitor::MainWindow\0\0isArmed\0"
    "set_armed(bool)\0status\0set_status(int)\0"
    "SOC\0set_battery(double)\0time\0"
    "set_time(string)\0sec\0set_uptime(double)\0"
    "RPY\0set_RPY(vector<double>)\0heading\0"
    "set_heading(double)\0pos\0set_position(gcs::GPS)\0"
    "height\0set_Height(double)\0velocity\0"
    "set_velocity(double)\0text\0"
    "set_autoPilot(string)\0set_subFlightmode(string)\0"
    "set_mainFlightmode(string)\0"
    "manual,simulation,stabilized,guided,autoM,test,custom\0"
    "set_otherFlightmode(bool,bool,bool,bool,bool,bool,bool)\0"
    "state\0set_mavlinkState(string)\0type\0"
    "set_mavlinkType(string)\0"
};

void drone_monitor::MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        MainWindow *_t = static_cast<MainWindow *>(_o);
        switch (_id) {
        case 0: _t->set_armed((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 1: _t->set_status((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 2: _t->set_battery((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 3: _t->set_time((*reinterpret_cast< string(*)>(_a[1]))); break;
        case 4: _t->set_uptime((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 5: _t->set_RPY((*reinterpret_cast< vector<double>(*)>(_a[1]))); break;
        case 6: _t->set_heading((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 7: _t->set_position((*reinterpret_cast< gcs::GPS(*)>(_a[1]))); break;
        case 8: _t->set_Height((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 9: _t->set_velocity((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 10: _t->set_autoPilot((*reinterpret_cast< string(*)>(_a[1]))); break;
        case 11: _t->set_subFlightmode((*reinterpret_cast< string(*)>(_a[1]))); break;
        case 12: _t->set_mainFlightmode((*reinterpret_cast< string(*)>(_a[1]))); break;
        case 13: _t->set_otherFlightmode((*reinterpret_cast< bool(*)>(_a[1])),(*reinterpret_cast< bool(*)>(_a[2])),(*reinterpret_cast< bool(*)>(_a[3])),(*reinterpret_cast< bool(*)>(_a[4])),(*reinterpret_cast< bool(*)>(_a[5])),(*reinterpret_cast< bool(*)>(_a[6])),(*reinterpret_cast< bool(*)>(_a[7]))); break;
        case 14: _t->set_mavlinkState((*reinterpret_cast< string(*)>(_a[1]))); break;
        case 15: _t->set_mavlinkType((*reinterpret_cast< string(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData drone_monitor::MainWindow::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject drone_monitor::MainWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_drone_monitor__MainWindow,
      qt_meta_data_drone_monitor__MainWindow, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &drone_monitor::MainWindow::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *drone_monitor::MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *drone_monitor::MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_drone_monitor__MainWindow))
        return static_cast<void*>(const_cast< MainWindow*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int drone_monitor::MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 16)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 16;
    }
    return _id;
}
QT_END_MOC_NAMESPACE

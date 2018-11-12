/********************************************************************************
** Form generated from reading UI file 'main_window.ui'
**
** Created by: Qt User Interface Compiler version 5.7.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAIN_WINDOW_H
#define UI_MAIN_WINDOW_H

#include <QtCore/QLocale>
#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QFormLayout>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QTextBrowser>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindowDesign
{
public:
    QAction *action_Quit;
    QAction *action_Preferences;
    QAction *actionAbout;
    QAction *actionAbout_Qt;
    QWidget *centralwidget;
    QHBoxLayout *hboxLayout;
    QTabWidget *tab_manager;
    QWidget *tab_status;
    QVBoxLayout *verticalLayout_2;
    QHBoxLayout *horizontalLayout;
    QGroupBox *groupBox_position;
    QWidget *horizontalLayoutWidget;
    QHBoxLayout *horizontalLayout_3;
    QVBoxLayout *verticalLayout_4;
    QLabel *label_5;
    QLabel *label;
    QLabel *label_2;
    QLabel *label_3;
    QLabel *label_4;
    QFrame *line;
    QLabel *label_15;
    QSpacerItem *verticalSpacer;
    QVBoxLayout *verticalLayout_5;
    QLabel *label_longitude;
    QLabel *label_latitude;
    QLabel *label_height;
    QLabel *label_velocity;
    QLabel *label_heading;
    QFrame *line_2;
    QLabel *label_RPY;
    QSpacerItem *verticalSpacer_2;
    QGroupBox *groupBox_2;
    QLabel *label_armed;
    QLabel *label_status;
    QWidget *horizontalLayoutWidget_2;
    QHBoxLayout *horizontalLayout_4;
    QVBoxLayout *verticalLayout_6;
    QLabel *label_7;
    QLabel *label_8;
    QLabel *label_9;
    QSpacerItem *verticalSpacer_3;
    QVBoxLayout *verticalLayout_7;
    QLabel *label_battery;
    QLabel *label_time;
    QLabel *label_uptime;
    QSpacerItem *verticalSpacer_4;
    QGroupBox *groupBox_3;
    QWidget *horizontalLayoutWidget_3;
    QHBoxLayout *horizontalLayout_5;
    QVBoxLayout *verticalLayout_8;
    QLabel *label_10;
    QLabel *label_11;
    QLabel *label_12;
    QLabel *label_13;
    QLabel *label_14;
    QSpacerItem *verticalSpacer_5;
    QVBoxLayout *verticalLayout_9;
    QLabel *label_ETA;
    QLabel *label_missinLenght;
    QLabel *label_nextLatitude;
    QLabel *label_nextLongitude;
    QLabel *label_nextAltitude;
    QSpacerItem *verticalSpacer_6;
    QSpacerItem *horizontalSpacer;
    QHBoxLayout *horizontalLayout_2;
    QGroupBox *groupBox_4;
    QWidget *formLayoutWidget;
    QFormLayout *formLayout;
    QLabel *label_16;
    QLabel *label_flightmode_main;
    QLabel *label_18;
    QLabel *label_filghtmode_sub;
    QLabel *label_20;
    QLabel *label_21;
    QLabel *label_flightmode_other;
    QLabel *label_autoPilot;
    QLabel *label_6;
    QLabel *label_mavlinkType;
    QLabel *label_19;
    QLabel *label_mavlinkState;
    QTextBrowser *textBrowser;
    QVBoxLayout *verticalLayout_10;
    QGroupBox *groupBox;
    QGroupBox *groupBox_5;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindowDesign)
    {
        if (MainWindowDesign->objectName().isEmpty())
            MainWindowDesign->setObjectName(QStringLiteral("MainWindowDesign"));
        MainWindowDesign->resize(944, 704);
        QIcon icon;
        icon.addFile(QStringLiteral(":/images/icon.png"), QSize(), QIcon::Normal, QIcon::Off);
        MainWindowDesign->setWindowIcon(icon);
        MainWindowDesign->setStyleSheet(QLatin1String("QGroupBox {\n"
"    border: 1px solid gray;\n"
"    border-radius: 9px;\n"
"    margin-top: 0.5em;\n"
"}\n"
"\n"
"QGroupBox::title {\n"
"    subcontrol-origin: margin;\n"
"    left: 10px;\n"
"    padding: 0 3px 0 3px;\n"
"}"));
        MainWindowDesign->setLocale(QLocale(QLocale::English, QLocale::Australia));
        action_Quit = new QAction(MainWindowDesign);
        action_Quit->setObjectName(QStringLiteral("action_Quit"));
        action_Quit->setShortcutContext(Qt::ApplicationShortcut);
        action_Preferences = new QAction(MainWindowDesign);
        action_Preferences->setObjectName(QStringLiteral("action_Preferences"));
        actionAbout = new QAction(MainWindowDesign);
        actionAbout->setObjectName(QStringLiteral("actionAbout"));
        actionAbout_Qt = new QAction(MainWindowDesign);
        actionAbout_Qt->setObjectName(QStringLiteral("actionAbout_Qt"));
        centralwidget = new QWidget(MainWindowDesign);
        centralwidget->setObjectName(QStringLiteral("centralwidget"));
        hboxLayout = new QHBoxLayout(centralwidget);
        hboxLayout->setObjectName(QStringLiteral("hboxLayout"));
        tab_manager = new QTabWidget(centralwidget);
        tab_manager->setObjectName(QStringLiteral("tab_manager"));
        tab_manager->setMinimumSize(QSize(100, 0));
        tab_manager->setMaximumSize(QSize(720, 720));
        tab_manager->setLocale(QLocale(QLocale::English, QLocale::Australia));
        tab_status = new QWidget();
        tab_status->setObjectName(QStringLiteral("tab_status"));
        verticalLayout_2 = new QVBoxLayout(tab_status);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        groupBox_position = new QGroupBox(tab_status);
        groupBox_position->setObjectName(QStringLiteral("groupBox_position"));
        groupBox_position->setMinimumSize(QSize(200, 0));
        groupBox_position->setMaximumSize(QSize(16777215, 16777215));
        horizontalLayoutWidget = new QWidget(groupBox_position);
        horizontalLayoutWidget->setObjectName(QStringLiteral("horizontalLayoutWidget"));
        horizontalLayoutWidget->setGeometry(QRect(10, 30, 160, 151));
        horizontalLayout_3 = new QHBoxLayout(horizontalLayoutWidget);
        horizontalLayout_3->setObjectName(QStringLiteral("horizontalLayout_3"));
        horizontalLayout_3->setContentsMargins(0, 0, 0, 0);
        verticalLayout_4 = new QVBoxLayout();
        verticalLayout_4->setObjectName(QStringLiteral("verticalLayout_4"));
        label_5 = new QLabel(horizontalLayoutWidget);
        label_5->setObjectName(QStringLiteral("label_5"));

        verticalLayout_4->addWidget(label_5);

        label = new QLabel(horizontalLayoutWidget);
        label->setObjectName(QStringLiteral("label"));

        verticalLayout_4->addWidget(label);

        label_2 = new QLabel(horizontalLayoutWidget);
        label_2->setObjectName(QStringLiteral("label_2"));

        verticalLayout_4->addWidget(label_2);

        label_3 = new QLabel(horizontalLayoutWidget);
        label_3->setObjectName(QStringLiteral("label_3"));

        verticalLayout_4->addWidget(label_3);

        label_4 = new QLabel(horizontalLayoutWidget);
        label_4->setObjectName(QStringLiteral("label_4"));

        verticalLayout_4->addWidget(label_4);

        line = new QFrame(horizontalLayoutWidget);
        line->setObjectName(QStringLiteral("line"));
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);

        verticalLayout_4->addWidget(line);

        label_15 = new QLabel(horizontalLayoutWidget);
        label_15->setObjectName(QStringLiteral("label_15"));

        verticalLayout_4->addWidget(label_15);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_4->addItem(verticalSpacer);


        horizontalLayout_3->addLayout(verticalLayout_4);

        verticalLayout_5 = new QVBoxLayout();
        verticalLayout_5->setObjectName(QStringLiteral("verticalLayout_5"));
        label_longitude = new QLabel(horizontalLayoutWidget);
        label_longitude->setObjectName(QStringLiteral("label_longitude"));

        verticalLayout_5->addWidget(label_longitude);

        label_latitude = new QLabel(horizontalLayoutWidget);
        label_latitude->setObjectName(QStringLiteral("label_latitude"));

        verticalLayout_5->addWidget(label_latitude);

        label_height = new QLabel(horizontalLayoutWidget);
        label_height->setObjectName(QStringLiteral("label_height"));

        verticalLayout_5->addWidget(label_height);

        label_velocity = new QLabel(horizontalLayoutWidget);
        label_velocity->setObjectName(QStringLiteral("label_velocity"));

        verticalLayout_5->addWidget(label_velocity);

        label_heading = new QLabel(horizontalLayoutWidget);
        label_heading->setObjectName(QStringLiteral("label_heading"));

        verticalLayout_5->addWidget(label_heading);

        line_2 = new QFrame(horizontalLayoutWidget);
        line_2->setObjectName(QStringLiteral("line_2"));
        line_2->setFrameShape(QFrame::HLine);
        line_2->setFrameShadow(QFrame::Sunken);

        verticalLayout_5->addWidget(line_2);

        label_RPY = new QLabel(horizontalLayoutWidget);
        label_RPY->setObjectName(QStringLiteral("label_RPY"));

        verticalLayout_5->addWidget(label_RPY);

        verticalSpacer_2 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_5->addItem(verticalSpacer_2);


        horizontalLayout_3->addLayout(verticalLayout_5);


        horizontalLayout->addWidget(groupBox_position);

        groupBox_2 = new QGroupBox(tab_status);
        groupBox_2->setObjectName(QStringLiteral("groupBox_2"));
        groupBox_2->setMinimumSize(QSize(150, 190));
        groupBox_2->setMaximumSize(QSize(150, 16777215));
        label_armed = new QLabel(groupBox_2);
        label_armed->setObjectName(QStringLiteral("label_armed"));
        label_armed->setGeometry(QRect(10, 140, 101, 21));
        label_status = new QLabel(groupBox_2);
        label_status->setObjectName(QStringLiteral("label_status"));
        label_status->setGeometry(QRect(10, 160, 111, 21));
        horizontalLayoutWidget_2 = new QWidget(groupBox_2);
        horizontalLayoutWidget_2->setObjectName(QStringLiteral("horizontalLayoutWidget_2"));
        horizontalLayoutWidget_2->setGeometry(QRect(10, 30, 131, 80));
        horizontalLayout_4 = new QHBoxLayout(horizontalLayoutWidget_2);
        horizontalLayout_4->setObjectName(QStringLiteral("horizontalLayout_4"));
        horizontalLayout_4->setContentsMargins(0, 0, 0, 0);
        verticalLayout_6 = new QVBoxLayout();
        verticalLayout_6->setObjectName(QStringLiteral("verticalLayout_6"));
        label_7 = new QLabel(horizontalLayoutWidget_2);
        label_7->setObjectName(QStringLiteral("label_7"));

        verticalLayout_6->addWidget(label_7);

        label_8 = new QLabel(horizontalLayoutWidget_2);
        label_8->setObjectName(QStringLiteral("label_8"));

        verticalLayout_6->addWidget(label_8);

        label_9 = new QLabel(horizontalLayoutWidget_2);
        label_9->setObjectName(QStringLiteral("label_9"));

        verticalLayout_6->addWidget(label_9);

        verticalSpacer_3 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_6->addItem(verticalSpacer_3);


        horizontalLayout_4->addLayout(verticalLayout_6);

        verticalLayout_7 = new QVBoxLayout();
        verticalLayout_7->setObjectName(QStringLiteral("verticalLayout_7"));
        label_battery = new QLabel(horizontalLayoutWidget_2);
        label_battery->setObjectName(QStringLiteral("label_battery"));

        verticalLayout_7->addWidget(label_battery);

        label_time = new QLabel(horizontalLayoutWidget_2);
        label_time->setObjectName(QStringLiteral("label_time"));

        verticalLayout_7->addWidget(label_time);

        label_uptime = new QLabel(horizontalLayoutWidget_2);
        label_uptime->setObjectName(QStringLiteral("label_uptime"));

        verticalLayout_7->addWidget(label_uptime);

        verticalSpacer_4 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_7->addItem(verticalSpacer_4);


        horizontalLayout_4->addLayout(verticalLayout_7);


        horizontalLayout->addWidget(groupBox_2);

        groupBox_3 = new QGroupBox(tab_status);
        groupBox_3->setObjectName(QStringLiteral("groupBox_3"));
        groupBox_3->setMinimumSize(QSize(250, 0));
        horizontalLayoutWidget_3 = new QWidget(groupBox_3);
        horizontalLayoutWidget_3->setObjectName(QStringLiteral("horizontalLayoutWidget_3"));
        horizontalLayoutWidget_3->setGeometry(QRect(10, 30, 221, 131));
        horizontalLayout_5 = new QHBoxLayout(horizontalLayoutWidget_3);
        horizontalLayout_5->setObjectName(QStringLiteral("horizontalLayout_5"));
        horizontalLayout_5->setContentsMargins(0, 0, 0, 0);
        verticalLayout_8 = new QVBoxLayout();
        verticalLayout_8->setObjectName(QStringLiteral("verticalLayout_8"));
        label_10 = new QLabel(horizontalLayoutWidget_3);
        label_10->setObjectName(QStringLiteral("label_10"));

        verticalLayout_8->addWidget(label_10);

        label_11 = new QLabel(horizontalLayoutWidget_3);
        label_11->setObjectName(QStringLiteral("label_11"));

        verticalLayout_8->addWidget(label_11);

        label_12 = new QLabel(horizontalLayoutWidget_3);
        label_12->setObjectName(QStringLiteral("label_12"));

        verticalLayout_8->addWidget(label_12);

        label_13 = new QLabel(horizontalLayoutWidget_3);
        label_13->setObjectName(QStringLiteral("label_13"));

        verticalLayout_8->addWidget(label_13);

        label_14 = new QLabel(horizontalLayoutWidget_3);
        label_14->setObjectName(QStringLiteral("label_14"));

        verticalLayout_8->addWidget(label_14);

        verticalSpacer_5 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_8->addItem(verticalSpacer_5);


        horizontalLayout_5->addLayout(verticalLayout_8);

        verticalLayout_9 = new QVBoxLayout();
        verticalLayout_9->setObjectName(QStringLiteral("verticalLayout_9"));
        label_ETA = new QLabel(horizontalLayoutWidget_3);
        label_ETA->setObjectName(QStringLiteral("label_ETA"));

        verticalLayout_9->addWidget(label_ETA);

        label_missinLenght = new QLabel(horizontalLayoutWidget_3);
        label_missinLenght->setObjectName(QStringLiteral("label_missinLenght"));

        verticalLayout_9->addWidget(label_missinLenght);

        label_nextLatitude = new QLabel(horizontalLayoutWidget_3);
        label_nextLatitude->setObjectName(QStringLiteral("label_nextLatitude"));

        verticalLayout_9->addWidget(label_nextLatitude);

        label_nextLongitude = new QLabel(horizontalLayoutWidget_3);
        label_nextLongitude->setObjectName(QStringLiteral("label_nextLongitude"));

        verticalLayout_9->addWidget(label_nextLongitude);

        label_nextAltitude = new QLabel(horizontalLayoutWidget_3);
        label_nextAltitude->setObjectName(QStringLiteral("label_nextAltitude"));

        verticalLayout_9->addWidget(label_nextAltitude);

        verticalSpacer_6 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_9->addItem(verticalSpacer_6);


        horizontalLayout_5->addLayout(verticalLayout_9);


        horizontalLayout->addWidget(groupBox_3);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer);


        verticalLayout_2->addLayout(horizontalLayout);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        groupBox_4 = new QGroupBox(tab_status);
        groupBox_4->setObjectName(QStringLiteral("groupBox_4"));
        formLayoutWidget = new QWidget(groupBox_4);
        formLayoutWidget->setObjectName(QStringLiteral("formLayoutWidget"));
        formLayoutWidget->setGeometry(QRect(10, 30, 271, 131));
        formLayout = new QFormLayout(formLayoutWidget);
        formLayout->setObjectName(QStringLiteral("formLayout"));
        formLayout->setContentsMargins(0, 0, 0, 0);
        label_16 = new QLabel(formLayoutWidget);
        label_16->setObjectName(QStringLiteral("label_16"));

        formLayout->setWidget(0, QFormLayout::LabelRole, label_16);

        label_flightmode_main = new QLabel(formLayoutWidget);
        label_flightmode_main->setObjectName(QStringLiteral("label_flightmode_main"));

        formLayout->setWidget(0, QFormLayout::FieldRole, label_flightmode_main);

        label_18 = new QLabel(formLayoutWidget);
        label_18->setObjectName(QStringLiteral("label_18"));

        formLayout->setWidget(1, QFormLayout::LabelRole, label_18);

        label_filghtmode_sub = new QLabel(formLayoutWidget);
        label_filghtmode_sub->setObjectName(QStringLiteral("label_filghtmode_sub"));

        formLayout->setWidget(1, QFormLayout::FieldRole, label_filghtmode_sub);

        label_20 = new QLabel(formLayoutWidget);
        label_20->setObjectName(QStringLiteral("label_20"));

        formLayout->setWidget(2, QFormLayout::LabelRole, label_20);

        label_21 = new QLabel(formLayoutWidget);
        label_21->setObjectName(QStringLiteral("label_21"));

        formLayout->setWidget(3, QFormLayout::LabelRole, label_21);

        label_flightmode_other = new QLabel(formLayoutWidget);
        label_flightmode_other->setObjectName(QStringLiteral("label_flightmode_other"));

        formLayout->setWidget(2, QFormLayout::FieldRole, label_flightmode_other);

        label_autoPilot = new QLabel(formLayoutWidget);
        label_autoPilot->setObjectName(QStringLiteral("label_autoPilot"));

        formLayout->setWidget(3, QFormLayout::FieldRole, label_autoPilot);

        label_6 = new QLabel(formLayoutWidget);
        label_6->setObjectName(QStringLiteral("label_6"));

        formLayout->setWidget(4, QFormLayout::LabelRole, label_6);

        label_mavlinkType = new QLabel(formLayoutWidget);
        label_mavlinkType->setObjectName(QStringLiteral("label_mavlinkType"));

        formLayout->setWidget(4, QFormLayout::FieldRole, label_mavlinkType);

        label_19 = new QLabel(formLayoutWidget);
        label_19->setObjectName(QStringLiteral("label_19"));

        formLayout->setWidget(5, QFormLayout::LabelRole, label_19);

        label_mavlinkState = new QLabel(formLayoutWidget);
        label_mavlinkState->setObjectName(QStringLiteral("label_mavlinkState"));

        formLayout->setWidget(5, QFormLayout::FieldRole, label_mavlinkState);


        horizontalLayout_2->addWidget(groupBox_4);


        verticalLayout_2->addLayout(horizontalLayout_2);

        textBrowser = new QTextBrowser(tab_status);
        textBrowser->setObjectName(QStringLiteral("textBrowser"));
        textBrowser->setMaximumSize(QSize(16777215, 200));

        verticalLayout_2->addWidget(textBrowser);

        tab_manager->addTab(tab_status, QString());

        hboxLayout->addWidget(tab_manager);

        verticalLayout_10 = new QVBoxLayout();
        verticalLayout_10->setObjectName(QStringLiteral("verticalLayout_10"));
        groupBox = new QGroupBox(centralwidget);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        groupBox->setMinimumSize(QSize(200, 0));

        verticalLayout_10->addWidget(groupBox);

        groupBox_5 = new QGroupBox(centralwidget);
        groupBox_5->setObjectName(QStringLiteral("groupBox_5"));

        verticalLayout_10->addWidget(groupBox_5);


        hboxLayout->addLayout(verticalLayout_10);

        MainWindowDesign->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindowDesign);
        menubar->setObjectName(QStringLiteral("menubar"));
        menubar->setGeometry(QRect(0, 0, 944, 19));
        MainWindowDesign->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindowDesign);
        statusbar->setObjectName(QStringLiteral("statusbar"));
        MainWindowDesign->setStatusBar(statusbar);

        retranslateUi(MainWindowDesign);
        QObject::connect(action_Quit, SIGNAL(triggered()), MainWindowDesign, SLOT(close()));

        tab_manager->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(MainWindowDesign);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindowDesign)
    {
        MainWindowDesign->setWindowTitle(QApplication::translate("MainWindowDesign", "QRosApp", 0));
        action_Quit->setText(QApplication::translate("MainWindowDesign", "&Quit", 0));
        action_Quit->setShortcut(QApplication::translate("MainWindowDesign", "Ctrl+Q", 0));
        action_Preferences->setText(QApplication::translate("MainWindowDesign", "&Preferences", 0));
        actionAbout->setText(QApplication::translate("MainWindowDesign", "&About", 0));
        actionAbout_Qt->setText(QApplication::translate("MainWindowDesign", "About &Qt", 0));
        groupBox_position->setTitle(QApplication::translate("MainWindowDesign", "Positioning and heading", 0));
        label_5->setText(QApplication::translate("MainWindowDesign", "<html><head/><body><p><span style=\" font-weight:600;\">Longitude:</span></p></body></html>", 0));
        label->setText(QApplication::translate("MainWindowDesign", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Sans Serif'; font-size:9pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:12px; margin-bottom:12px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-weight:600;\">Latitude:</span></p></body></html>", 0));
        label_2->setText(QApplication::translate("MainWindowDesign", "<html><head/><body><p><span style=\" font-weight:600;\">Height:</span></p></body></html>", 0));
        label_3->setText(QApplication::translate("MainWindowDesign", "<html><head/><body><p><span style=\" font-weight:600;\">Velocity:</span></p></body></html>", 0));
        label_4->setText(QApplication::translate("MainWindowDesign", "<html><head/><body><p><span style=\" font-weight:600;\">Heading:</span></p></body></html>", 0));
        label_15->setText(QApplication::translate("MainWindowDesign", "<html><head/><body><p><span style=\" font-weight:600;\">RPY:</span></p></body></html>", 0));
        label_longitude->setText(QApplication::translate("MainWindowDesign", "N/A", 0));
        label_latitude->setText(QApplication::translate("MainWindowDesign", "N/A", 0));
        label_height->setText(QApplication::translate("MainWindowDesign", "N/A", 0));
        label_velocity->setText(QApplication::translate("MainWindowDesign", "N/A", 0));
        label_heading->setText(QApplication::translate("MainWindowDesign", "N/A", 0));
        label_RPY->setText(QApplication::translate("MainWindowDesign", "N/A", 0));
        groupBox_2->setTitle(QApplication::translate("MainWindowDesign", "Drone Status", 0));
        label_armed->setText(QApplication::translate("MainWindowDesign", "<html><head/><body><p><span style=\" font-size:12pt; font-weight:600; color:#ff0000;\">Not Armed</span></p></body></html>", 0));
        label_status->setText(QApplication::translate("MainWindowDesign", "<html><head/><body><p><span style=\" font-size:12pt; font-weight:600;\">ON GROUND</span></p></body></html>", 0));
        label_7->setText(QApplication::translate("MainWindowDesign", "<html><head/><body><p><span style=\" font-weight:600;\">Battery:</span></p></body></html>", 0));
        label_8->setText(QApplication::translate("MainWindowDesign", "<html><head/><body><p><span style=\" font-weight:600;\">Time:</span></p></body></html>", 0));
        label_9->setText(QApplication::translate("MainWindowDesign", "<html><head/><body><p><span style=\" font-weight:600;\">Up Time:</span></p></body></html>", 0));
        label_battery->setText(QApplication::translate("MainWindowDesign", "N/A", 0));
        label_time->setText(QApplication::translate("MainWindowDesign", "N/A", 0));
        label_uptime->setText(QApplication::translate("MainWindowDesign", "N/A", 0));
        groupBox_3->setTitle(QApplication::translate("MainWindowDesign", "Mission", 0));
        label_10->setText(QApplication::translate("MainWindowDesign", "<html><head/><body><p><span style=\" font-weight:600;\">ETA:</span></p></body></html>", 0));
        label_11->setText(QApplication::translate("MainWindowDesign", "<html><head/><body><p><span style=\" font-weight:600;\">Mission length:</span></p></body></html>", 0));
        label_12->setText(QApplication::translate("MainWindowDesign", "<html><head/><body><p><span style=\" font-weight:600;\">Next latitude:</span></p></body></html>", 0));
        label_13->setText(QApplication::translate("MainWindowDesign", "<html><head/><body><p><span style=\" font-weight:600;\">Next Longitude:</span></p></body></html>", 0));
        label_14->setText(QApplication::translate("MainWindowDesign", "<html><head/><body><p><span style=\" font-weight:600;\">Next Altitude:</span></p></body></html>", 0));
        label_ETA->setText(QApplication::translate("MainWindowDesign", "N/A", 0));
        label_missinLenght->setText(QApplication::translate("MainWindowDesign", "N/A", 0));
        label_nextLatitude->setText(QApplication::translate("MainWindowDesign", "N/A", 0));
        label_nextLongitude->setText(QApplication::translate("MainWindowDesign", "N/A", 0));
        label_nextAltitude->setText(QApplication::translate("MainWindowDesign", "N/A", 0));
        groupBox_4->setTitle(QApplication::translate("MainWindowDesign", "Drone Info:", 0));
        label_16->setText(QApplication::translate("MainWindowDesign", "<html><head/><body><p><span style=\" font-weight:600;\">Main Flightmode:</span></p></body></html>", 0));
        label_flightmode_main->setText(QApplication::translate("MainWindowDesign", "N/A", 0));
        label_18->setText(QApplication::translate("MainWindowDesign", "<html><head/><body><p><span style=\" font-weight:600;\">Sub flightmode:</span></p></body></html>", 0));
        label_filghtmode_sub->setText(QApplication::translate("MainWindowDesign", "N/A", 0));
        label_20->setText(QApplication::translate("MainWindowDesign", "<html><head/><body><p><span style=\" font-weight:600;\">Other modes:</span></p></body></html>", 0));
        label_21->setText(QApplication::translate("MainWindowDesign", "<html><head/><body><p><span style=\" font-weight:600;\">AutoPilot:</span></p></body></html>", 0));
        label_flightmode_other->setText(QApplication::translate("MainWindowDesign", "N/A", 0));
        label_autoPilot->setText(QApplication::translate("MainWindowDesign", "N/A", 0));
        label_6->setText(QApplication::translate("MainWindowDesign", "<html><head/><body><p><span style=\" font-weight:600;\">Mavlink Type:</span></p></body></html>", 0));
        label_mavlinkType->setText(QApplication::translate("MainWindowDesign", "N/A", 0));
        label_19->setText(QApplication::translate("MainWindowDesign", "<html><head/><body><p><span style=\" font-weight:600;\">Mavlink State:</span></p></body></html>", 0));
        label_mavlinkState->setText(QApplication::translate("MainWindowDesign", "N/A", 0));
        tab_manager->setTabText(tab_manager->indexOf(tab_status), QApplication::translate("MainWindowDesign", "Drone", 0));
        groupBox->setTitle(QApplication::translate("MainWindowDesign", "Active Nodes", 0));
        groupBox_5->setTitle(QApplication::translate("MainWindowDesign", "Docking Stations", 0));
    } // retranslateUi

};

namespace Ui {
    class MainWindowDesign: public Ui_MainWindowDesign {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAIN_WINDOW_H

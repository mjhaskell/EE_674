#ifndef MAINWINDOW_HPP
#define MAINWINDOW_HPP

#include <QMainWindow>
#include "uav_sim/dronenode.hpp"
//#include "autopilot/controllernode.hpp"
#include <QProcess>

#include "uav_msgs/State.h"

namespace Ui
{
class MainWindow;
}

class OSGWidget;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(int argc,char** argv,QWidget *parent = nullptr);
    ~MainWindow();

protected:
    void setupSignalsAndSlots();
    void closeEvent(QCloseEvent *);
    void readSettings();
    void writeSettings();
    void updateRosStatus();
    void startRosCore();
    void createToolbar();
    void setupStatusBar();
    QAction* createStartAction();
    QAction* createResetAction();
    void startOrPauseSim();
    bool startSimulation();
    void pauseSimulation();
    void resetSimulation();
    void disableOtherConnectionOptions();
    void onSuccessfulMasterConnection();
    QAction* createRosPanelAction();
    QAction* createControllerPanelAction();
    void populateTopicsComboBox();
    void onToolbarVisibilityChanged(bool visible);

    void updatePoseFromSliders();
    void updateWindFromSliders();

public slots:
    void closeWithWarning();

private slots:
    void on_start_triggered();
    void on_close_triggered();
    void on_roscore_button_clicked();
    void on_ros_check_box_clicked();
    void on_view_ros_settings_panel_triggered();
    void on_view_controller_panel_triggered();
    void on_view_ros_connection_status_triggered();
    void on_master_connect_button_clicked();
    void on_use_env_check_box_clicked(bool checked);
    void on_ip_button_clicked();
    void on_ros_dock_visibilityChanged(bool visible);
    void on_controller_dock_visibilityChanged(bool visible);
    void on_scan_button_clicked();
    void on_subscribe_button_clicked();
    void on_reset_triggered();
    void on_view_main_toolbar_triggered();
    void on_set_waypoint_button_clicked();
    void on_set_weights_button_clicked();
    void on_set_rates_button_clicked();
    void on_pause_triggered();
    void on_wn_spin_valueChanged(double value);
    void on_we_spin_valueChanged(double arg1);
    void on_gust_check_box_toggled(bool checked);
    void on_default_waypoints_button_clicked();
    void on_generate_map_button_clicked();

private:
    Ui::MainWindow *m_ui{nullptr};
    OSGWidget *m_osg_widget{nullptr};
    int m_argc;
    char** m_argv;
    uav::DroneNode m_drone_node;
    QToolBar *m_main_toolbar{nullptr};
    QProcess *m_process{nullptr};
    bool m_app_started_roscore{false};
    QIcon m_check_icon{QIcon{":myicons/check.png"}};
    QIcon m_x_icon{QIcon{":myicons/red_x.jpg"}};
    bool m_use_ros_ip{true};
    bool m_is_running{false};
    double m_de{0};
    double m_dt{0};
    double m_da{0};
    double m_dr{0};
    double m_wn{0};
    double m_we{0};
};

#endif // MAINWINDOW_HPP

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

struct eulerAngles {
    double theta_1;
    double psi_1;
    double phi_1;

    double theta_2;
    double psi_2;
    double phi_2;
};

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_calcButton_clicked();

    void on_showButton_clicked();

    void on_resetButton_clicked();

    void on_showButton2_clicked();

private:
    Ui::MainWindow *ui;
    void runCalc();
    void readPatchFile();
    void setCameraBox();
};

#endif // MAINWINDOW_H

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

struct eulerAngles {
    double theta_x;
    double theta_y;
    double theta_z;
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

    void on_saveButton_clicked();

    void on_loadButton_clicked();

    void on_interpolateButton_clicked();

    void on_matchButton_clicked();

private:
    Ui::MainWindow *ui;
    void runCalc();
    void readPatchFile();
    void setCameraBox();
    void showIImage();
};

#endif // MAINWINDOW_H

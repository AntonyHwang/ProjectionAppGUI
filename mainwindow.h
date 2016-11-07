#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

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

private:
    Ui::MainWindow *ui;
    void runCalc();
    void readPatchFile();
    void setCameraBox();
};

#endif // MAINWINDOW_H

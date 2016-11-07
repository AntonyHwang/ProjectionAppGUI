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

private:
    Ui::MainWindow *ui;
    void runCalc();
    void readPatchFile();
};

#endif // MAINWINDOW_H

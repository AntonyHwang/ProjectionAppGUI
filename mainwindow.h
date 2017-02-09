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

    void on_showButton2_clicked();

    void on_saveButton_clicked();

    void on_loadButton_clicked();

    void on_interpolateButton_clicked();

    void on_matchButton_clicked();

    void on_selectButton_clicked();

    void on_interpolateSlider_actionTriggered(int action);

    void on_sliderSpinner_valueChanged(int arg1);

    void on_getRGBValButton_clicked();

    void on_cam2Box_currentTextChanged(const QString &arg1);

    void on_getRGBValImpButton_clicked();


private:
    Ui::MainWindow *ui;
    void runCalc();
    void setCameraBox();
    void showIImage(int pointCount);
    void showRGBImg(int camIndex);
};

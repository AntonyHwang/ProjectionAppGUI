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

    void disable_all_ui();

    void enable_all_ui();

    void cluster_all_points (int numOfPoint);

    void interpolate_frames();

    void on_calcButton_clicked();

    void on_showButton_clicked();

    void on_resetButton_clicked();

    void on_showButton2_clicked();

    void on_matchButton_clicked();

    void on_selectButton_clicked();

    void on_interpolateSlider_valueChanged();

    void on_sliderSpinner_valueChanged();

    void on_cam2Box_currentTextChanged();

    void getRGBVal();

    void on_full_interpolation_clicked();

private:
    Ui::MainWindow *ui;
    void runCalc();
    void setCameraBox();
    void showIImage(int pointCount);
    void showRGBImg(int camIndex);
};

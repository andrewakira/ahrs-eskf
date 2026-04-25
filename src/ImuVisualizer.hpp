#pragma once

#include <pangolin/pangolin.h>
#include <pangolin/plot/plotter.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <mutex>
#include <memory>
#include <queue>

class ImuVisualizer {
public:
    ImuVisualizer() {
        pangolin::CreateWindowAndBind("IMU Viewer", 1280, 720);
        glEnable(GL_DEPTH_TEST);

        pangolin::CreatePanel("ui")
            .SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(180));

        btnAccel = std::make_unique<pangolin::Var<bool>>("ui.Accel", false, false);
        btnGyro  = std::make_unique<pangolin::Var<bool>>("ui.Gyro",  false, false);
        btnPose  = std::make_unique<pangolin::Var<bool>>("ui.Pose",  false, false);

        accelLog.SetLabels({"ax", "ay", "az"});
        gyroLog.SetLabels({"gx", "gy", "gz"});

        accelPlotter = std::make_unique<pangolin::Plotter>(
            &accelLog,
            0.0, 1000.0,        // X-axis window: display the latest 1000 IMU samples (~5 seconds at 200 Hz)
            -12.0, 12.0,        // Y-axis range: accelerometer values in m/s^2
            200.0, 1.0          // Grid spacing: X tick every 200 samples (~1 sec), Y tick every 1 m/s^2
        );

        gyroPlotter = std::make_unique<pangolin::Plotter>(
            &gyroLog,
            0.0, 1000.0,        // X-axis window: display the latest 1000 IMU samples (~5 seconds at 200 Hz)
            -4.5, 4.5,          // Y-axis range: gyroscope angular velocity in rad/s
            200.0, 0.5          // Grid spacing: X tick every 200 samples (~1 sec), Y tick every 0.5 rad/s
        );

        accelPlotter->Track("$i");
        gyroPlotter->Track("$i");

        pangolin::Display("main")
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(180), 1.0)
            .AddDisplay(*accelPlotter)
            .AddDisplay(*gyroPlotter);

        sCam = std::make_unique<pangolin::OpenGlRenderState>(
            pangolin::ProjectionMatrixOrthographic(
                -2.5, 2.5,       // left, right
                -2.5, 2.5,       // bottom, top
                -10.0, 10.0      // near, far
            ),
            pangolin::ModelViewLookAt(
                3, -3, 3,        // camera position
                0, 0, 0,         // look at origin
                pangolin::AxisZ
            )
        );

        handler3D = std::make_unique<pangolin::Handler3D>(*sCam);

        poseView = &pangolin::Display("pose")
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(180), 1.0)
            .SetHandler(handler3D.get());
    }

    void pushImu(double ax, double ay, double az,
        double gx, double gy, double gz,
        const Eigen::Quaterniond& q) {
        std::lock_guard<std::mutex> lock(mtx);

        imuQueue.push({
            ax, ay, az,
            gx, gy, gz,
            q.normalized()
        });
    }

    void renderOnce() {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        consumeQueue();
        updateModeFromButtons();

        accelPlotter->Show(currentMode == 0);
        gyroPlotter->Show(currentMode == 1);
        poseView->Show(currentMode == 2);

        if (currentMode == 2) {
            drawPose();
        }

        pangolin::FinishFrame();
    }

    bool shouldQuit() const {
        return pangolin::ShouldQuit();
    }

private:
    struct ImuSample {
        double ax, ay, az;
        double gx, gy, gz;
        Eigen::Quaterniond q;
    };

    void updateModeFromButtons() {
        if (*btnAccel) {
            currentMode = 0;
            *btnAccel = false;
        }

        if (*btnGyro) {
            currentMode = 1;
            *btnGyro = false;
        }

        if (*btnPose) {
            currentMode = 2;
            *btnPose = false;
        }
    }

    void consumeQueue() {
        while (true) {
            ImuSample sample;

            {
                std::lock_guard<std::mutex> lock(mtx);

                if (imuQueue.empty()) {
                    break;
                }

                sample = imuQueue.front();
                imuQueue.pop();
            }

            accelLog.Log(sample.ax, sample.ay, sample.az);
            gyroLog.Log(sample.gx, sample.gy, sample.gz);
            currentQ = sample.q;
        }
    }

    void drawPose() {
        poseView->Activate(*sCam);

        drawWorldAxes();

        Eigen::Matrix3d R = currentQ.normalized().toRotationMatrix();

        Eigen::Vector3d origin(0, 0, 0);
        Eigen::Vector3d xAxis = R * Eigen::Vector3d(1, 0, 0);
        Eigen::Vector3d yAxis = R * Eigen::Vector3d(0, 1, 0);
        Eigen::Vector3d zAxis = R * Eigen::Vector3d(0, 0, 1);

        glLineWidth(4.0f);

        glColor3f(1.0f, 0.0f, 0.0f);
        drawLine(origin, xAxis);

        glColor3f(0.0f, 1.0f, 0.0f);
        drawLine(origin, yAxis);

        glColor3f(0.0f, 0.3f, 1.0f);
        drawLine(origin, zAxis);

        glLineWidth(1.0f);
    }

    void drawWorldAxes() {
        glLineWidth(1.5f);

        glColor3f(0.5f, 0.0f, 0.0f);
        drawLine(Eigen::Vector3d::Zero(), Eigen::Vector3d(1.5, 0, 0));

        glColor3f(0.0f, 0.5f, 0.0f);
        drawLine(Eigen::Vector3d::Zero(), Eigen::Vector3d(0, 1.5, 0));

        glColor3f(0.0f, 0.0f, 0.5f);
        drawLine(Eigen::Vector3d::Zero(), Eigen::Vector3d(0, 0, 1.5));
    }

    void drawLine(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2) {
        glBegin(GL_LINES);
        glVertex3d(p1.x(), p1.y(), p1.z());
        glVertex3d(p2.x(), p2.y(), p2.z());
        glEnd();
    }

private:
    pangolin::DataLog accelLog;
    pangolin::DataLog gyroLog;

    std::unique_ptr<pangolin::Plotter> accelPlotter;
    std::unique_ptr<pangolin::Plotter> gyroPlotter;

    std::unique_ptr<pangolin::OpenGlRenderState> sCam;
    std::unique_ptr<pangolin::Handler3D> handler3D;
    pangolin::View* poseView = nullptr;

    std::unique_ptr<pangolin::Var<bool>> btnAccel;
    std::unique_ptr<pangolin::Var<bool>> btnGyro;
    std::unique_ptr<pangolin::Var<bool>> btnPose;

    int currentMode = 0; // 0: Accel, 1: Gyro, 2: Pose

    std::mutex mtx;
    std::queue<ImuSample> imuQueue;

    Eigen::Quaterniond currentQ = Eigen::Quaterniond::Identity();
};
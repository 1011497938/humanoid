#include "dvision/ipm.hpp"
using namespace cv;
using namespace std;
using namespace Eigen;

namespace dvision {
void
IPM::Init(std::vector<double> extrinsic_para, double fx, double fy, double cx, double cy)
{
    m_extrinsic_para = extrinsic_para;
    assert(m_extrinsic_para.size() == 16);

    m_cameraMatrix = MatrixXd(4, 4);
    m_cameraMatrix << fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0, 0, 0, 0, 1;

    // debug
    cout << "camera matrix: \n" << m_cameraMatrix << endl;
    cout << "extrinsic para: [" << endl;
    for_each(m_extrinsic_para.begin(), m_extrinsic_para.end(), [](double& x) { cout << x << endl; });
    cout << "]" << endl;
}

void
IPM::update(double pitch, double yaw)
{
    calc_extrinsic(pitch, yaw);
    m_A = m_cameraMatrix * m_extrinsic;
    m_invA = m_A.inverse();
}

void
IPM::updateDeg(double pitch, double yaw)
{
    update(pitch / 180.0 * M_PI, yaw / 180.0 * M_PI);
}

Point2d
IPM::project(double x_real, double y_real, double z_real)
{
    MatrixXd real(4, 1);
    real << x_real, y_real, z_real, 1;

    MatrixXd pointImage = m_A * real;
    double u = pointImage(0) / pointImage(2);
    double v = pointImage(1) / pointImage(2);

    return Point2d(u, v);
}

Point2d
IPM::inverseProject(double u, double v, double z_real)
{
    double c1 = m_invA(2, 0);
    double c2 = m_invA(2, 1);
    double c3 = m_invA(2, 2);
    double c4 = m_invA(2, 3);

    double s = (z_real - c4) / (u * c1 + v * c2 + c3);

    MatrixXd foo(4, 1);
    foo << s * u, s * v, s, 1;

    MatrixXd res(4, 1);
    res = m_invA * foo;

    double x = res(0);
    double y = res(1);
    return Point2d(x, y);
}

void
IPM::calc_extrinsic(double pitchRad, double yawRad)
{
    auto Xw2p = m_extrinsic_para[0];
    auto Yw2p = m_extrinsic_para[1];
    auto Zw2p = m_extrinsic_para[2];

    auto RXw2p = m_extrinsic_para[3];
    auto RYw2p = m_extrinsic_para[4];
    auto RZw2p = m_extrinsic_para[5];
    auto Xp2c = m_extrinsic_para[6];
    auto Yp2c = m_extrinsic_para[7];
    auto Zp2c = m_extrinsic_para[8];

    auto RXp2c = m_extrinsic_para[9];
    auto RYp2c = m_extrinsic_para[10];
    auto RZp2c = m_extrinsic_para[11];

    auto scaleYaw = m_extrinsic_para[12];
    auto scalePitch = m_extrinsic_para[13];
    auto biasYaw = m_extrinsic_para[14];
    auto biasPitch = m_extrinsic_para[15];

    double pitch = (pitchRad + biasPitch) * scalePitch;
    double yaw = (yawRad + biasYaw) * scaleYaw;

    MatrixXd w2p(4, 4);
    w2p = rotateZ(RZw2p) * rotateY(RYw2p) * rotateX(RXw2p) * dtranslate(-Xw2p, -Yw2p, -Zw2p);

    MatrixXd p2c(4, 4);
    p2c = rotateZ(RZp2c) * rotateY(RYp2c) * rotateX(RXp2c) * dtranslate(-Xp2c, -Yp2c, -Zp2c) * rotateY(-pitch) * rotateZ(-yaw);

    MatrixXd c2i(4, 4); // camera to image
    c2i << 0, -1, 0, 0, 0, 0, -1, 0, 1, 0, 0, 0, 0, 0, 0, 1;

    m_extrinsic = c2i * p2c * w2p;
}
} // namespace dvision

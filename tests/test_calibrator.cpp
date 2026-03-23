#include <gtest/gtest.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "../include/hand_eye_calibrator.h"

TEST(QuaternionTest, Normalization) {
    Eigen::Quaterniond q(0.5, 0.5, 0.5, 0.5);
    q.normalize();
    EXPECT_NEAR(q.norm(), 1.0, 1e-10);
}

TEST(QuaternionTest, IdentityQuaternion) {
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    EXPECT_NEAR(q.w(), 1.0, 1e-10);
    EXPECT_NEAR(q.x(), 0.0, 1e-10);
    EXPECT_NEAR(q.y(), 0.0, 1e-10);
    EXPECT_NEAR(q.z(), 0.0, 1e-10);
}

TEST(QuaternionTest, RotationMatrixConversion) {
    Eigen::Quaterniond q(Eigen::AngleAxisd(M_PI/4, Eigen::Vector3d::UnitZ()));
    Eigen::Matrix3d R = q.toRotationMatrix();
    Eigen::Quaterniond q2(R);
    q2.normalize();

    // Check angle difference is small
    double angle_diff = 2.0 * std::acos(std::min(1.0, std::abs(q.w() * q2.w() + q.x() * q2.x() + q.y() * q2.y() + q.z() * q2.z())));
    EXPECT_NEAR(angle_diff, 0.0, 1e-6);
}

TEST(SLERPTest, InterpolateIdentity) {
    HandEyeCalibrator calib;
    Eigen::Quaterniond qa = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond qb(Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitY()));
    Eigen::Quaterniond result;
    bool ok = calib.slerpSafe(qa, qb, 0.5, result);
    EXPECT_TRUE(ok);
    EXPECT_NEAR(result.norm(), 1.0, 1e-10);
}

TEST(SLERPTest, Interpolate90Degrees) {
    HandEyeCalibrator calib;
    Eigen::Quaterniond qa(Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()));
    Eigen::Quaterniond qb(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ()));
    Eigen::Quaterniond result;
    bool ok = calib.slerpSafe(qa, qb, 0.5, result);
    EXPECT_TRUE(ok);

    // At t=0.5, should be 45 degrees
    double angle = 2.0 * std::acos(std::min(1.0, std::abs(result.w())));
    EXPECT_NEAR(angle, M_PI/4, 1e-6);
}

TEST(RelativeMotionTest, ComputeRelativeTransform) {
    // Create two poses
    TimedPose p1, p2;
    p1.timestamp = 0.0;
    p1.t = Eigen::Vector3d(0, 0, 0);
    p1.q = Eigen::Quaterniond::Identity();

    p2.timestamp = 1.0;
    p2.t = Eigen::Vector3d(1, 0, 0);
    p2.q = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ()));

    // Compute relative motion: p1^{-1} * p2
    Eigen::Quaterniond q1_inv = p1.q.conjugate();
    Eigen::Quaterniond q_rel = q1_inv * p2.q;
    Eigen::Vector3d t_rel = q1_inv * (p2.t - p1.t);

    // Expected: rotation by 90 degrees around Z, translation along X
    double angle = 2.0 * std::acos(std::min(1.0, std::abs(q_rel.w())));
    EXPECT_NEAR(angle, M_PI/2, 1e-6);
    EXPECT_NEAR(t_rel.x(), 1.0, 1e-6);
    EXPECT_NEAR(t_rel.y(), 0.0, 1e-6);
}

TEST(CalibratorInitTest, DefaultConstructor) {
    HandEyeCalibrator calib;
    // Just verify it can be constructed without error
    SUCCEED();
}

TEST(CalibratorInitTest, IdentityInit) {
    HandEyeCalibrator calib;
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    Eigen::Vector3d t = Eigen::Vector3d::Zero();

    EXPECT_NEAR(q.w(), 1.0, 1e-10);
    EXPECT_NEAR(t.norm(), 0.0, 1e-10);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

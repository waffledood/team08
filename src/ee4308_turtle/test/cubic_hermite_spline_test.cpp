#include <gtest/gtest.h>

#include <cmath>

#include "common.hpp"
#include "cubic_hermite_spline.h"

class CubicHermiteSplineTest : public testing::Test
{
protected:
  static std::vector<Position> raw_path_;
  static std::vector<double> t_;
  static CubicHermiteSpline spline_;

  static void SetUpTestSuite()
  {
    std::vector<double> x{ 0, 1, 2, 3 };
    std::vector<double> y{ exp(0), exp(1), exp(2), exp(3) };

    for (int i = 0; i < x.size(); ++i)
    {
      raw_path_.push_back(Position(x.at(i), y.at(i)));
    }

    const double average_speed = 0.2;
    std::vector<double> t{ 0 };
    double total_duration = 0;

    for (int i = 1; i < raw_path_.size(); ++i)
    {
      double dx = raw_path_.at(i).x - raw_path_.at(i - 1).x;
      double dy = raw_path_.at(i).y - raw_path_.at(i - 1).y;
      double duration = sqrt(dx * dx + dy * dy) / average_speed;
      total_duration += duration;
      t.push_back(total_duration);
    }

    t_ = t;
    spline_ = CubicHermiteSpline(raw_path_, t);
  }
};

std::vector<Position> CubicHermiteSplineTest::raw_path_;
std::vector<double> CubicHermiteSplineTest::t_;
CubicHermiteSpline CubicHermiteSplineTest::spline_;

TEST_F(CubicHermiteSplineTest, evaluateBoundary)
{
  for (int i = 0; i < t_.size(); ++i)
  {
    Position pos = spline_.evaluate(t_.at(i));
    EXPECT_NEAR(pos.x, raw_path_.at(i).x, raw_path_.at(i).x * 1e-3);
    EXPECT_NEAR(pos.y, raw_path_.at(i).y, raw_path_.at(i).y * 1e-3);
  }
}

TEST_F(CubicHermiteSplineTest, cZeroContinuity)
{
  const double EPSILON_T = 1e-10;
  const double EPSILON_VAL = 1e-9;

  for (int i = 1; i < t_.size() - 1; ++i)
  {
    double boundary = t_.at(i);
    double before = boundary - EPSILON_T;
    double after = boundary + EPSILON_T;
    Position before_pos = spline_.evaluate(before);
    Position after_pos = spline_.evaluate(after);
    EXPECT_NEAR(before_pos.x, after_pos.x, EPSILON_VAL);
    EXPECT_NEAR(before_pos.y, after_pos.y, EPSILON_VAL);
  }
}

TEST_F(CubicHermiteSplineTest, cOneContinuity)
{
  const double EPSILON_T = 1e-10;
  const double EPSILON_GRAD = 1e-10;

  for (int i = 1; i < t_.size() - 1; ++i)
  {
    double boundary = t_.at(i);
    double before = boundary - EPSILON_T;
    double after = boundary + EPSILON_T;

    double before_grad = spline_.evaluateXVelocity(before);
    double after_grad = spline_.evaluateXVelocity(after);
    EXPECT_NEAR(before_grad, after_grad, EPSILON_GRAD);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

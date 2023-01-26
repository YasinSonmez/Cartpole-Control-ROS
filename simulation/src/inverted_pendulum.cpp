#include "inverted_pendulum.h"

#include "Eigen/Dense"

InvertedPendulum::InvertedPendulum(double M, double m, double J, double l,
                                   double c, double gamma, Eigen::VectorXd x_0)
    : M_(M),
      m_(m),
      J_(J),
      l_(l),
      c_(c),
      gamma_(gamma),
      g_(9.81),
      M_t_(M + m),
      J_t_(J + m * std::pow(l, 2)),
      x_(x_0),
      x_dot_(Eigen::VectorXd(4)),
      previous_time_(0)
{
  x_dot_ << 0, 0, 0, 0;
}

InvertedPendulum::InvertedPendulum(Eigen::VectorXd x_0)
    : InvertedPendulum(1.0, 1.0, 1.0, 1.0, 1.0, 1.0, x_0) {}

InvertedPendulum::InvertedPendulum()
    : InvertedPendulum(Eigen::VectorXd(4))
{
  x_ << 0, 0, 0, 0;
}

void InvertedPendulum::Update(double time, double u)
{
  // Recover state parameters
  double x = x_(0);     // position of the base
  double theta = x_(1); // angle of the pendulum
  double vx = x_(2);    // velocity of the base
  double omega = x_(3); // angular rate of the pendulum

  // Compute common terms
  double s_t = std::sin(theta);
  double c_t = std::cos(theta);
  double o_2 = std::pow(omega, 2);
  double l_2 = std::pow(l_, 2);

  // Calculate derivatives
  x_dot_(0) = vx;
  x_dot_(1) = omega;
  x_dot_(2) = (-m_ * l_ * s_t * o_2 + m_ * g_ * (m_ * l_2 / J_t_) * s_t * c_t -
               c_ * vx - (gamma_ / J_t_) * m_ * l_ * c_t * omega + u) /
              (M_t_ - m_ * (m_ * l_2 / J_t_) * c_t * c_t);
  x_dot_(3) =
      (-m_ * l_2 * s_t * c_t * o_2 + M_t_ * g_ * l_ * s_t - c_ * l_ * c_t * vx -
       gamma_ * (M_t_ / m_) * omega + l_ * c_t * u) /
      (J_t_ * (M_t_ / m_) - m_ * (l_ * c_t) * (l_ * c_t));

  // Apply Euler method to solve differential equations
  double dt = time - previous_time_;
  previous_time_ = time;
  x_ += x_dot_ * dt;
}

Eigen::VectorXd InvertedPendulum::GetState() const { return x_; }

void InvertedPendulum::Linearize()
{
  const double mu = M_t_ * J_t_ - std::pow((m_ * l_), 2);

  A_ = Eigen::MatrixXd::Zero(4, 4);
  A_(0, 2) = 1;
  A_(1, 3) = 1;
  A_(2, 1) = std::pow((m_ * l_), 2) * g_ / mu;
  A_(2, 2) = -c_ * J_t_ / mu;
  A_(2, 3) = -gamma_ * l_ * m_ / mu;
  A_(3, 1) = M_t_ * m_ * g_ * l_ / mu;
  A_(3, 2) = -c_ * l_ * m_ / mu;
  A_(3, 3) = -gamma_ * M_t_ / mu;

  B_ = Eigen::MatrixXd::Zero(4, 1);
  B_(2, 0) = J_t_ / mu;
  B_(3, 0) = l_ * m_ / mu;

  C_ = Eigen::MatrixXd::Identity(4, 4);
  D_ = Eigen::MatrixXd::Zero(4, 1);
}
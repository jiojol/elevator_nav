#include <array>
#include <Eigen/SparseCholesky>
#include <iostream>
#include <traj_utils/polynomial_traj.h>

PolynomialTraj PolynomialTraj::minSnapTraj(const Eigen::MatrixXd &Pos, const Eigen::Vector3d &start_vel,
                                           const Eigen::Vector3d &end_vel, const Eigen::Vector3d &start_acc,
                                           const Eigen::Vector3d &end_acc, const Eigen::VectorXd &Time)
{
  using Matrix6 = Eigen::Matrix<double, 6, 6>;
  using EndpointData = Eigen::Matrix<double, 6, 3>;
  const int segments = Time.size();
  if (segments == 0 || Pos.cols() != segments + 1 || !Pos.allFinite() ||
      !Time.allFinite() || (Time.array() <= 0.0).any())
    return {};
  if (segments == 1)
    return one_segment_traj_gen(Pos.col(0), start_vel, start_acc, Pos.col(1), end_vel, end_acc, Time(0));

  // Only velocity and acceleration at internal knots are free. Each segment
  // couples the two neighboring knots, so the minimum-jerk system is sparse.
  std::vector<Matrix6> coefficient_maps(segments);
  std::vector<EndpointData> endpoint_data(segments, EndpointData::Zero());
  std::vector<std::array<int, 6>> variable_rows(segments);
  const int free_count = 2 * (segments - 1);
  std::vector<Eigen::Triplet<double>> entries;
  entries.reserve(16 * segments);
  Eigen::MatrixXd rhs = Eigen::MatrixXd::Zero(free_count, 3);

  for (int seg = 0; seg < segments; ++seg)
  {
    const double time = Time(seg);
    auto &fixed = endpoint_data[seg];
    fixed.row(0) = Pos.col(seg).transpose();
    fixed.row(1) = Pos.col(seg + 1).transpose();
    auto &rows = variable_rows[seg];
    rows.fill(-1);
    if (seg == 0)
    {
      fixed.row(2) = start_vel.transpose();
      fixed.row(4) = start_acc.transpose();
    }
    else
    {
      rows[2] = 2 * (seg - 1);
      rows[4] = rows[2] + 1;
    }
    if (seg == segments - 1)
    {
      fixed.row(3) = end_vel.transpose();
      fixed.row(5) = end_acc.transpose();
    }
    else
    {
      rows[3] = 2 * seg;
      rows[5] = rows[3] + 1;
    }

    Matrix6 mapping = Matrix6::Zero(), jerk = Matrix6::Zero();
    for (int derivative = 0; derivative < 3; ++derivative)
    {
      mapping(2 * derivative, derivative) = derivative == 2 ? 2.0 : 1.0;
      for (int power = derivative; power < 6; ++power)
      {
        double factor = 1.0;
        for (int d = 0; d < derivative; ++d)
          factor *= power - d;
        mapping(2 * derivative + 1, power) = factor * std::pow(time, power - derivative);
      }
    }
    for (int a = 3; a < 6; ++a)
      for (int b = 3; b < 6; ++b)
        jerk(a, b) = a * (a - 1) * (a - 2) * b * (b - 1) * (b - 2) *
                     std::pow(time, a + b - 5) / (a + b - 5);

    auto &coefficients = coefficient_maps[seg];
    coefficients = mapping.inverse();
    const Matrix6 cost = coefficients.transpose() * jerk * coefficients;
    for (int a = 2; a < 6; ++a)
    {
      if (rows[a] < 0)
        continue;
      rhs.row(rows[a]) -= cost.row(a) * fixed;
      for (int b = 2; b < 6; ++b)
        if (rows[b] >= 0)
          entries.emplace_back(rows[a], rows[b], cost(a, b));
    }
  }

  Eigen::SparseMatrix<double> hessian(free_count, free_count);
  hessian.setFromTriplets(entries.begin(), entries.end());
  Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver(hessian);
  if (solver.info() != Eigen::Success)
    return {};
  const Eigen::MatrixXd solved = solver.solve(rhs);
  if (solver.info() != Eigen::Success || !solved.allFinite())
    return {};

  PolynomialTraj trajectory;
  for (int seg = 0; seg < segments; ++seg)
  {
    auto &data = endpoint_data[seg];
    for (int row = 2; row < 6; ++row)
      if (variable_rows[seg][row] >= 0)
        data.row(row) = solved.row(variable_rows[seg][row]);
    const EndpointData coeff = coefficient_maps[seg] * data;
    std::vector<double> x(6), y(6), z(6);
    for (int power = 0; power < 6; ++power)
    {
      x[5 - power] = coeff(power, 0);
      y[5 - power] = coeff(power, 1);
      z[5 - power] = coeff(power, 2);
    }
    trajectory.addSegment(x, y, z, Time(seg));
  }
  return trajectory;
}

PolynomialTraj PolynomialTraj::one_segment_traj_gen(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                                    const Eigen::Vector3d &end_pt, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc,
                                                    double t)
{
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(6, 6), Crow(1, 6);
  Eigen::VectorXd Bx(6), By(6), Bz(6);

  C(0, 5) = 1;
  C(1, 4) = 1;
  C(2, 3) = 2;
  Crow << pow(t, 5), pow(t, 4), pow(t, 3), pow(t, 2), t, 1;
  C.row(3) = Crow;
  Crow << 5 * pow(t, 4), 4 * pow(t, 3), 3 * pow(t, 2), 2 * t, 1, 0;
  C.row(4) = Crow;
  Crow << 20 * pow(t, 3), 12 * pow(t, 2), 6 * t, 2, 0, 0;
  C.row(5) = Crow;

  Bx << start_pt(0), start_vel(0), start_acc(0), end_pt(0), end_vel(0), end_acc(0);
  By << start_pt(1), start_vel(1), start_acc(1), end_pt(1), end_vel(1), end_acc(1);
  Bz << start_pt(2), start_vel(2), start_acc(2), end_pt(2), end_vel(2), end_acc(2);

  Eigen::VectorXd Cofx = C.colPivHouseholderQr().solve(Bx);
  Eigen::VectorXd Cofy = C.colPivHouseholderQr().solve(By);
  Eigen::VectorXd Cofz = C.colPivHouseholderQr().solve(Bz);

  vector<double> cx(6), cy(6), cz(6);
  for (int i = 0; i < 6; i++)
  {
    cx[i] = Cofx(i);
    cy[i] = Cofy(i);
    cz[i] = Cofz(i);
  }

  PolynomialTraj poly_traj;
  poly_traj.addSegment(cx, cy, cz, t);

  return poly_traj;
}

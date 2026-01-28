#include <optional>



#include "gimbal_optics_calc/gimbal_optics_calc.h"

GimbalOpticsCalc::GimbalOpticsCalc() {
  // 默认 zoom->alpha：alpha = k*zoom + b
  zoom2alpha_fn = [this](double z) { return k_zoom2alpha * z + b_zoom2alpha; };

  // 默认 normal_prob：用标准正态 CDF（把 0..100 映射到 μ=50, σ=15）
  normal_prob_fn = [](double x01) {
    const double x = clamp(x01, 0.0, 100.0);
    const double mu = 50.0;
    const double sigma = 15.0;
    const double z = (x - mu) / (sigma * std::sqrt(2.0));
    return 0.5 * (1.0 + std::erf(z));
  };
}

void GimbalOpticsCalc::bindNormalProbFn(std::function<double(double)> fn) {
  if (!fn) throw std::invalid_argument("normal_prob_fn is empty");
  normal_prob_fn = std::move(fn);
}

void GimbalOpticsCalc::bindZoom2AlphaFn(std::function<double(double)> fn) {
  if (!fn) throw std::invalid_argument("zoom2alpha_fn is empty");
  zoom2alpha_fn = std::move(fn);
}

void GimbalOpticsCalc::setZoomAlphaHeuristic(double k, double b) {
  k_zoom2alpha = k;
  b_zoom2alpha = b;
  zoom2alpha_fn = [this](double z) { return k_zoom2alpha * z + b_zoom2alpha; };
}

double GimbalOpticsCalc::alphaEffective(double a) const {
  return a * (1.0 + alpha_Distcoff);
}

GimbalOpticsCalc::SolveOneAxisResult
GimbalOpticsCalc::solveOneAxis(std::optional<double> frame,
                               std::optional<double> alpha,
                               std::optional<double> obj_distance) {
  const int known = (frame.has_value() ? 1 : 0) + (alpha.has_value() ? 1 : 0) +
                    (obj_distance.has_value() ? 1 : 0);
  if (known < 2) {
    throw std::invalid_argument("Need at least 2 known variables among frame/alpha/obj_distance.");
  }

  SolveOneAxisResult out{frame, alpha, obj_distance};

  // 你的新等式：
  // tan(0.5*alpha) = (0.5*frame) / obj_distance

  // 已知 alpha 与 obj_distance -> frame
  if (!out.frame.has_value() && out.alpha.has_value() && out.obj_distance.has_value()) {
    const double a = *out.alpha;
    const double d = *out.obj_distance;
    if (std::abs(d) < 1e-12) throw std::runtime_error("obj_distance too small; cannot solve frame.");
    out.frame = 2.0 * d * std::tan(0.5 * a);
    return out;
  }

  // 已知 frame 与 obj_distance -> alpha
  if (!out.alpha.has_value() && out.frame.has_value() && out.obj_distance.has_value()) {
    const double f = *out.frame;
    const double d = *out.obj_distance;
    if (std::abs(d) < 1e-12) throw std::runtime_error("obj_distance too small; cannot solve alpha.");
    const double t = (0.5 * f) / d;
    out.alpha = 2.0 * std::atan(t);
    return out;
  }

  // 已知 frame 与 alpha -> obj_distance
  if (!out.obj_distance.has_value() && out.frame.has_value() && out.alpha.has_value()) {
    const double f = *out.frame;
    const double a = *out.alpha;
    const double denom = std::tan(0.5 * a);
    if (std::abs(denom) < 1e-12) throw std::runtime_error("tan(alpha/2) too small; cannot solve obj_distance.");
    out.obj_distance = (0.5 * f) / denom;
    return out;
  }

  // 三个都给了：原样返回
  return out;
}

GimbalOpticsCalc::SolveTwoAxisResult
GimbalOpticsCalc::solveTwoAxis(std::optional<double> frame_x,
                               std::optional<double> frame_y,
                               std::optional<double> alpha_x,
                               std::optional<double> alpha_y,
                               std::optional<double> obj_distance) {
  SolveTwoAxisResult out;
  out.x = solveOneAxis(frame_x, alpha_x, obj_distance);
  out.y = solveOneAxis(frame_y, alpha_y, obj_distance);
  return out;
}

double GimbalOpticsCalc::alphaFromZoom(double z, bool apply_dist) {
  zoom = z;
  if (!zoom2alpha_fn) throw std::runtime_error("zoom2alpha_fn not bound.");

  const double a = zoom2alpha_fn(z);
  alpha = apply_dist ? alphaEffective(a) : a;
  return alpha;
}

double GimbalOpticsCalc::zoomFromAlpha(double alpha_in, bool alpha_has_dist) const {
  // 1) 先把 alpha 还原到“未畸变”的启发式 alpha（如果 alpha_in 是畸变后的）
  double a = alpha_in;
  if (alpha_has_dist) {
    const double s = (1.0 + alpha_Distcoff);
    if (std::abs(s) < 1e-12) throw std::runtime_error("Invalid alpha_Distcoff: 1+alpha_Distcoff too small.");
    a = a / s;
  }

  // 2) 线性启发式 alpha = k*zoom + b 的逆
  if (std::abs(k_zoom2alpha) < 1e-12) throw std::runtime_error("k_zoom2alpha is zero; cannot invert alpha->zoom.");
  return (a - b_zoom2alpha) / k_zoom2alpha;
}

double GimbalOpticsCalc::normalProb(double x01) const {
  if (!normal_prob_fn) throw std::runtime_error("normal_prob_fn not bound.");
  return clamp(normal_prob_fn(x01), 0.0, 1.0);
}


/** 概率 p 越大，返回的 picture_xy（画幅）反而越大（在同一个 target 下）*/
GimbalOpticsCalc::Pair
GimbalOpticsCalc::frameFromTargetAndProb(const Pair& target, double p) {
  target_xy = target;
  const double tx = target.first;
  const double ty = target.second;
  if (tx <= 0.0 || ty <= 0.0) throw std::invalid_argument("target size must be positive.");

  // p 是“区间概率”：P(|X-μ|<=nσ) = erf(n/sqrt(2))
  // => n = sqrt(2)*erfinv(p)
  const double pp = clamp(p, 0.0, 0.999999);  // 避免 erfinv(1) 发散
  const double n_sigma /* σ */ = std::sqrt(2.0) * erfinv_approx(pp)/* 把输入概率p反函数映射到-1到1*/;

  // 你给的启发：r=1 -> n=1; r=1/2 -> n=3，线性插值：
  // n = 1 + 4*(1-r)  => r = 1 - (n-1)/4
  double r = 1.0 - (n_sigma - 1.0) / 4.0;
  r = clamp(r, 1e-6, 1.0);

  const double target_area = tx * ty;
  const double frame_area = target_area / r;

  // 画幅保持 target 宽高比
  const double aspect = tx / ty;
  const double frame_y = std::sqrt(frame_area / aspect);
  const double frame_x = aspect * frame_y;

  picture_xy = {frame_x, frame_y};
  return picture_xy;
}

double GimbalOpticsCalc::clamp(double v, double lo, double hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

// Winitzki 近似 erfinv：工程够用；需要更准可换 boost::math::erf_inv
double GimbalOpticsCalc::erfinv_approx(double y) {
  y = clamp(y, -0.999999, 0.999999);

  const double a = 0.147;
  const double ln1 = std::log(1.0 - y * y);
  const double t = 2.0 / (M_PI * a) + ln1 / 2.0;
  const double inside = t * t - ln1 / a;
  const double sign = (y >= 0.0) ? 1.0 : -1.0;
  return sign * std::sqrt(std::sqrt(inside) - t);
}

/** 拟合*/

// 读 yaml：每条样本含 zoom/frame/obj_distance
// 先 solveOneAxis(frame, nullopt, obj_distance) 得 alpha
// 再拟合 alpha = k*zoom + b
std::pair<double, double> FitKBFromYaml(const std::string& yaml_path)
{
  YAML::Node root = YAML::LoadFile(yaml_path);
  if (!root) {
    throw std::runtime_error("Failed to load yaml: " + yaml_path);
  }
  if (!root.IsSequence()) {
    throw std::runtime_error("YAML root must be a sequence (list) of samples.");
  }
  if (root.size() < 2) {
    throw std::runtime_error("Need at least 2 samples to fit k,b.");
  }

  std::vector<double> zs;
  std::vector<double> as;
  zs.reserve(root.size());
  as.reserve(root.size());

  for (std::size_t i = 0; i < root.size(); ++i) {
    const YAML::Node& s = root[i];
    if (!s || !s.IsMap()) {
      throw std::runtime_error("Sample[" + std::to_string(i) + "] is not a map.");
    }
    if (!s["zoom"] || !s["frame"] || !s["obj_distance"]) {
      throw std::runtime_error("Sample[" + std::to_string(i) + "] missing keys: zoom/frame/obj_distance.");
    }

    const double zoom = s["zoom"].as<double>();
    const double frame = s["frame"].as<double>();
    const double d = s["obj_distance"].as<double>();

    // 由 frame 与 obj_distance 反解 alpha（你的新等式版本）
    auto r = GimbalOpticsCalc::solveOneAxis(frame, std::nullopt, d);
    if (!r.alpha) {
      throw std::runtime_error("Failed to solve alpha for sample[" + std::to_string(i) + "].");
    }
    const double alpha = *r.alpha;

    zs.push_back(zoom);
    as.push_back(alpha);
  }

  // ===== 线性回归：alpha = k*zoom + b =====
  const int n = static_cast<int>(zs.size());
  double sum_z = 0.0, sum_a = 0.0;
  for (int i = 0; i < n; ++i) {
    sum_z += zs[i];
    sum_a += as[i];
  }
  const double mean_z = sum_z / n;
  const double mean_a = sum_a / n;

  double var_z = 0.0, cov_za = 0.0;
  for (int i = 0; i < n; ++i) {
    const double dz = zs[i] - mean_z;
    const double da = as[i] - mean_a;
    var_z += dz * dz;
    cov_za += dz * da;
  }

  if (std::abs(var_z) < 1e-12) {
    throw std::runtime_error("Zoom variance is too small (all zoom nearly identical), cannot fit k.");
  }

  const double k = cov_za / var_z;
  const double b = mean_a - k * mean_z;
  return {k, b};
}


/** 三次多项式拟合*/
// 解 4x4 线性方程组 A x = b（高斯消元 + 部分主元）
static std::array<double, 4> Solve4x4(std::array<std::array<double, 4>, 4> A,
                                      std::array<double, 4> b) {
  for (int col = 0; col < 4; ++col) {
    // 选主元
    int pivot = col;
    double best = std::abs(A[col][col]);
    for (int r = col + 1; r < 4; ++r) {
      double v = std::abs(A[r][col]);
      if (v > best) { best = v; pivot = r; }
    }
    if (best < 1e-14) throw std::runtime_error("Singular/ill-conditioned normal matrix (pivot too small).");

    if (pivot != col) {
      std::swap(A[pivot], A[col]);
      std::swap(b[pivot], b[col]);
    }

    // 消元
    const double diag = A[col][col];
    for (int r = col + 1; r < 4; ++r) {
      const double f = A[r][col] / diag;
      if (std::abs(f) < 1e-20) continue;
      for (int c = col; c < 4; ++c) A[r][c] -= f * A[col][c];
      b[r] -= f * b[col];
    }
  }

  // 回代
  std::array<double, 4> x{};
  for (int i = 3; i >= 0; --i) {
    double s = b[i];
    for (int j = i + 1; j < 4; ++j) s -= A[i][j] * x[j];
    const double diag = A[i][i];
    if (std::abs(diag) < 1e-14) throw std::runtime_error("Singular/ill-conditioned normal matrix (diag too small).");
    x[i] = s / diag;
  }
  return x;
}

// 读 yaml，先解 alpha；然后拟合 alpha = c0 + c1 z + c2 z^2 + c3 z^3
// 返回 {c0, c1, c2, c3}
std::array<double, 4> FitCubicFromYaml(const std::string& yaml_path) {
  YAML::Node root = YAML::LoadFile(yaml_path);
  if (!root) throw std::runtime_error("Failed to load yaml: " + yaml_path);
  if (!root.IsSequence()) throw std::runtime_error("YAML root must be a sequence (list) of samples.");
  if (root.size() < 4) throw std::runtime_error("Need at least 4 samples to fit degree=3 polynomial.");

  // 构造法方程： (X^T X) c = X^T y
  // X 行向量为 [1, z, z^2, z^3], y=alpha
  std::array<std::array<double, 4>, 4> XtX{};
  std::array<double, 4> Xty{};
  for (int i = 0; i < 4; ++i) {
    Xty[i] = 0.0;
    for (int j = 0; j < 4; ++j) XtX[i][j] = 0.0;
  }

  for (std::size_t i = 0; i < root.size(); ++i) {
    const YAML::Node& s = root[i];
    if (!s || !s.IsMap()) throw std::runtime_error("Sample[" + std::to_string(i) + "] is not a map.");
    if (!s["zoom"] || !s["frame"] || !s["obj_distance"])
      throw std::runtime_error("Sample[" + std::to_string(i) + "] missing keys: zoom/frame/obj_distance.");

    const double z = s["zoom"].as<double>();
    const double frame = s["frame"].as<double>();
    const double d = s["obj_distance"].as<double>();

    // 由 frame 与 obj_distance 反解 alpha（你的新等式版本）
    auto r = GimbalOpticsCalc::solveOneAxis(frame, std::nullopt, d);
    if (!r.alpha) throw std::runtime_error("Failed to solve alpha for sample[" + std::to_string(i) + "].");
    const double a = *r.alpha;

    const double x0 = 1.0;
    const double x1 = z;
    const double x2 = z * z;
    const double x3 = x2 * z;
    const double x[4] = {x0, x1, x2, x3};

    for (int p = 0; p < 4; ++p) {
      Xty[p] += x[p] * a;
      for (int q = 0; q < 4; ++q) {
        XtX[p][q] += x[p] * x[q];
      }
    }
  }

  // 解出系数 c0..c3
  return Solve4x4(XtX, Xty);
}

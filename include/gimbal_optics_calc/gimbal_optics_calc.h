#pragma once
#include <cmath>
#include <functional>
#include <optional>
#include <stdexcept>
#include <utility>

#include <yaml-cpp/yaml.h>
#include <string>
#include <stdexcept>
#include <utility>
#include <vector>
#include <cmath>

// std::pair<double, double> FitZoomAlphaKBFromYaml(
//     const std::string& yaml_path = "/src/gimbal_optics_calc/data/kb.yaml",
//     const std::string& zoom_key,
//     const std::string& frame_key,
//     const std::string& obj_distance_key);

std::pair<double, double> FitKBFromYaml(const std::string& yaml_path = "/home/kilox/cloud_mapping/src/gimbal_optics_calc/data/kb.yaml");
std::array<double, 4> FitCubicFromYaml(const std::string& yaml_path = "/home/kilox/cloud_mapping/src/gimbal_optics_calc/data/kb.yaml");

class GimbalOpticsCalc {
public:
  using Pair = std::pair<double, double>;

  // =============== 成员变量 ===============
  Pair picture_xy{0.0, 0.0};      // 1) 画幅 (x, y) (单位自定，但要一致)
  Pair target_xy{0.0, 0.0};       // 8) 物体实际长宽
  double alpha{0.0};              // 2) 视场角 alpha（建议弧度）
  double zoom{1.0};               // 3) 放大倍率 zoom
  double alpha_Distcoff{0.0};     // 5) alpha 畸变系数
  double obj_distance{0.0};       // 6) 物距


  // 4) 正态分布启发函数：输入 x∈[0,100] -> 输出 p∈[0,1]（可替换）
  std::function<double(double)> normal_prob_fn;
  // 7) 倍率-视场角启发式：输入 zoom -> 输出 alpha（可替换）
  std::function<double(double)> zoom2alpha_fn;

  // 默认启发式参数：alpha = k*zoom + b
  double k_zoom2alpha{0.0};
  double b_zoom2alpha{0.0};

public:
  GimbalOpticsCalc();

  // =============== bind / setter ===============
  void bindNormalProbFn(std::function<double(double)> fn);
  void bindZoom2AlphaFn(std::function<double(double)> fn);

  // 设定默认线性启发式 alpha = k*zoom + b
  void setZoomAlphaHeuristic(double k, double b);

  // 你没规定畸变模型，这里按：alpha_eff = alpha * (1 + alpha_Distcoff)
  double alphaEffective(double a) const;

  // ============================================================
  // 成员函数 1：单轴求解
  // 等式：tan(0.5*alpha) = (0.5*frame) / obj_distance
  // 任意给 2 个求第 3 个
  // ============================================================
  struct SolveOneAxisResult {
    std::optional<double> frame;
    std::optional<double> alpha;
    std::optional<double> obj_distance;
  };

  static SolveOneAxisResult solveOneAxis(std::optional<double> frame,
                                        std::optional<double> alpha,
                                        std::optional<double> obj_distance);

  struct SolveTwoAxisResult {
    SolveOneAxisResult x;
    SolveOneAxisResult y;
  };

  static SolveTwoAxisResult solveTwoAxis(std::optional<double> frame_x,
                                        std::optional<double> frame_y,
                                        std::optional<double> alpha_x,
                                        std::optional<double> alpha_y,
                                        std::optional<double> obj_distance);

  // ============================================================
  // 成员函数 2：输入 zoom -> 求 alpha（调用启发式，可替换）
  // apply_dist=true 时会应用 alpha_Distcoff
  // ============================================================
  double alphaFromZoom(double zoom, bool apply_dist = false);
  double zoomFromAlpha(double alpha_in, bool alpha_has_dist = false) const;
  // ============================================================
  // 成员函数 4：正态分布概率接口（可替换）
  // 输入 0..100 -> 输出 0..1
  // ============================================================
  double normalProb(double x01) const;

  // ============================================================
  // 成员函数 3：target + prob -> frame
  // 规则见 cpp 中注释
  // ============================================================
  Pair frameFromTargetAndProb(const Pair& target, double p);

private:
  static double clamp(double v, double lo, double hi);
  static double erfinv_approx(double y);
};

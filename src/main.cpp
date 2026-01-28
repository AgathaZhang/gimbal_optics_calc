#include <ros/ros.h>
#include <optional>
#include "gimbal_optics_calc/gimbal_optics_calc.h"

#include <fstream>
#include <filesystem>
#include <iostream>
#include <string>

static double rad2deg(double rad) { return rad * 180.0 / M_PI; }


auto append_line_to_txt = [](const std::string& txt_path, const std::string& line) {
  namespace fs = std::filesystem;
  try {
    fs::path p(txt_path);
    if (p.has_parent_path()) {
      fs::create_directories(p.parent_path()); // 目录不存在就创建
    }
    std::ofstream ofs(txt_path, std::ios::out | std::ios::app); // app: 追加写；不存在会创建
    if (!ofs.is_open()) {
      std::cerr << "[append_line_to_txt] failed to open: " << txt_path << std::endl;
      return;
    }
    ofs << line << "\n";
  } catch (const std::exception& e) {
    std::cerr << "[append_line_to_txt] exception: " << e.what() << std::endl;
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "gimbal_optics_calc_node");
  ros::NodeHandle nh("~");

  auto [nk, nb] = FitKBFromYaml();
  std::cout << "拟合k=" << nk << " 拟合b=" << nb << std::endl;

  // auto c = FitCubicFromYaml();
  // std::cout << "拟合c0=" << c[0] << " 拟合c1=" << c[1] << " 拟合c2=" << c[2] << " 拟合c3=" << c[3] << std::endl;

  // double b = c[0];
  // double k = c[1];

  // ===== 参数（可 rosparam 覆盖）=====
  double k = nk;            // alpha = k*zoom + b
  double b = nb;             // 弧度
  double alpha_dist = 0.0;    // xy轴方向畸变

  double zoom = 25;         // 缩放10倍
  double obj_d = 6500;         // 2m物距

  double frame_x = 372.5;
  double frame_y = 0.24;

  double target_x = 100;
  double target_y = 100;
  // double prob = 0.80;         // 0..1
  double prob = 0.95;

  nh.param("k_zoom2alpha", k, k);
  nh.param("b_zoom2alpha", b, b);
  nh.param("alpha_distcoff", alpha_dist, alpha_dist);

  nh.param("zoom", zoom, zoom);
  nh.param("obj_distance", obj_d, obj_d);
  nh.param("frame_x", frame_x, frame_x);
  nh.param("frame_y", frame_y, frame_y);
  nh.param("target_x", target_x, target_x);
  nh.param("target_y", target_y, target_y);
  nh.param("prob", prob, prob);

  // ===== 构造计算器 =====
  GimbalOpticsCalc calc;
  calc.alpha_Distcoff = alpha_dist;        // 我可能是想体现 x轴方向的压缩倍率和y轴不一致
  calc.obj_distance = obj_d;               // 物距初始化
  calc.picture_xy = {frame_x, frame_y};
  calc.target_xy = {target_x, target_y};
  calc.setZoomAlphaHeuristic(k, b);        // zoom和alpha是一次关系

  // (2) zoom -> alpha
  const double alpha_rad = calc.alphaFromZoom(zoom, false);    // 给缩放得角度
  ROS_INFO_STREAM("zoom=" << zoom << " => alpha(rad)=" << alpha_rad << " alpha(deg)=" << rad2deg(alpha_rad));

  // (1) 任意两者求第三者：示例（用 x 轴）
  // {
  //   auto r = GimbalOpticsCalc::solveOneAxis(
  //       /*frame=*/std::nullopt ,
  //       /*alpha=*/alpha_rad,
  //       /*obj_distance=*/obj_d);
  //   ROS_INFO_STREAM("[solveOneAxis] known(alpha,obj_distance) => frame=" << (r.frame ? *r.frame : -1.0));
  // }

  // { 
  //   double zoom_result = -1.0;
  //   auto r = GimbalOpticsCalc::solveOneAxis(
  //       /*frame=*/frame_x,
  //       /*alpha=*/std::nullopt,
  //       /*obj_distance=*/obj_d);
  //   if (r.alpha) {
  //    zoom_result = calc.zoomFromAlpha(*r.alpha, /*alpha_has_dist=*/false);
  //   }
  //   ROS_INFO_STREAM("[solveOneAxis] known(frame,obj_distance) => zoom=" << zoom_result /*(zoom == 0? zoom: -1.0)*/);
  // }

  // {
  //   auto r = GimbalOpticsCalc::solveOneAxis(
  //       /*frame=*/frame_x,
  //       /*alpha=*/alpha_rad,
  //       /*obj_distance=*/std::nullopt);
  //   ROS_INFO_STREAM("[solveOneAxis] known(frame,alpha) => obj_distance=" << (r.obj_distance ? *r.obj_distance : -1.0));
  // }

  for (double big = 100; big < 2100; big+=100)    // 外层物体大小
  {
    for (double far = 1000; far < 31000; far+=1000)    // 内层物距
    {
      auto frame = calc.frameFromTargetAndProb({big, big}, prob);
      double zoom_result = -1.0;
      auto r = GimbalOpticsCalc::solveOneAxis(
          /*frame=*/frame.first,
          /*alpha=*/std::nullopt,
          /*obj_distance=*/far);
      if (r.alpha) {
      zoom_result = calc.zoomFromAlpha(*r.alpha, /*alpha_has_dist=*/false);
      // 如果 zoom_result 为负数，zoom_result=0
      if (zoom_result < 0) {
        zoom_result = 0;
      }
      std::cout << "big = " << big/1000 << " far = " << far/1000 << " zoom = " << zoom_result << std::endl;
          append_line_to_txt(
          "/home/kilox/cloud_mapping/src/gimbal_optics_calc/data/result.yaml",
          "big=" + std::to_string(big/1000) +
          " far=" + std::to_string(far/1000) +
          " zoom=" + std::to_string(zoom_result)
      );
    
      }
      // ROS_INFO_STREAM("[solveOneAxis] known(frame,obj_distance) => zoom=" << zoom_result /*(zoom == 0? zoom: -1.0)*/);
    }
  }
  






  // (3) target + prob -> frame
  {
    auto frame = calc.frameFromTargetAndProb({target_x, target_y}, prob);
    ROS_INFO_STREAM("[frameFromTargetAndProb] target=(" << target_x << "," << target_y << "), prob=" << prob << " => frame=(" << frame.first << "," << frame.second << ")");
  }




  
  // (4) normalProb：0..100 -> 0..1
  {
    double x01 = 70.0;
    double p = calc.normalProb(x01);
    ROS_INFO_STREAM("[normalProb] x=" << x01 << " => p=" << p);
  }

  ROS_INFO("Demo done, exiting.");
  return 0;
}

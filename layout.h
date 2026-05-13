#pragma once
#include "algorithm.h"
#include "hasher.h"
#include "lru_size.h"
#include "part.h"
#include "sheet.h"
#include "transformed_shape.h"
namespace nesting {

typedef typename geo::FT FT;
typedef typename geo::Point_2 Point_2;
typedef typename geo::Polygon_with_holes_2 Polygon_with_holes_2;
typedef typename geo::Transformation Transformation;
typedef typename geo::Vector_2 Vector_2;
typedef typename hash::NFPCacheKey NFPCacheKey;
typedef typename hash::NFPCacheValue NFPCacheValue;
typedef typename hash::NFPCacheKeyHasher NFPCacheKeyHasher;

// 算法运行时的布局情况，用于缓存已计算的NFP和已放置的多边形
struct BranchPoint {
  size_t idx;                    // 多边形索引
  FT before_pd;                  // 分支前的PD
  FT after_pd;                   // 分支后的PD
  Transformation trans;           // 分支前的变换
  Point_2 pos;                   // 分支前的位置
  std::shared_ptr<TransformedShape> saved_shape;  // 分支前的完整状态
  std::vector<FT> saved_pd;      // 分支前的PD数组
};

struct Layout {
  Layout(std::vector<Item> items, std::vector<Sheet> sheets, FT _area = -1);

 private:
  // 当前布局下的pd，压缩到一维。pd[i][j]映射到pd[n*i-i*(i+1)/2+(j-i-1)]
  std::vector<FT> glob_pd;
  std::vector<double> miu;

 public:
  // 超参数
  double rinc{0.01};
  double rdec{0.04};
  size_t maxIterations{50};
  uint32_t max_pd_cache{200000};
  // 模拟退火参数
  double saTemperature{1.0};        // 初始温度
  double saCoolingRate{0.95};       // 冷却率
  size_t saIterations{10};          // 分支迭代次数
  size_t saMaxIterations{100};      // 总退火迭代次数
  bool saEnabled{false};            // 是否启用模拟退火
  // 分支点存储
  std::vector<BranchPoint> branch_points;
  // 多边形总数
  size_t poly_num{0};
  size_t pd_count{0};
  size_t pd_miss{0};
  // 多板，但每个板都是相同的矩形
  std::vector<Sheet> sheets;
  // 已计算的nfp缓存
  std::unordered_map<NFPCacheKey, NFPCacheValue, NFPCacheKeyHasher> nfp_cache;
  // 已计算的规范多边形集合
  std::unordered_map<Polygon_with_holes_2,
                     std::shared_ptr<Polygon_with_holes_2>,
                     hash::PolygonHasher>
      canonical_polygons;
  // LRU PD缓存，线程不安全
  emlru_size::lru_cache<hash::PDCacheKey, FT, hash::PDCacheKeyHasher> pd_cache;
  // sheet_parts[i]代表sheet i上存储的变形shape
  std::vector<std::vector<TransformedShape>> sheet_parts;
  std::vector<std::vector<TransformedShape>> best_result;
  // 当前长度
  FT cur_length{-1};
  // 最短长度
  FT best_length{-1};
  // 最佳利用率
  double best_utilization{0};
  // 排样长度的下界
  FT lower_length{-1};
  // 总面积
  FT area{0};
  bool isRequestQuit{false};
  /// @brief 存放一个规范多边形，规范多边形的意思是其第一个顶点平移到(0, 0)处
  /// @param poly 需要存放的规范多边形
  /// @return 指向该多边形的一个指针
  Polygon_with_holes_2* get_canonical_polygon(const Polygon_with_holes_2& poly);

  // 设置polygon[i]和polygon[j]之间的pd
  inline void set_pd(size_t i, size_t j, FT pd) {
    if (i == j) {
      return;
    }
    if (i > j) {
      i ^= j;
      j ^= i;
      i ^= j;
    }
    size_t idx = poly_num * i - i * (i + 1) / 2 + (j - i - 1);
    glob_pd[idx] = pd;
  }

  // 获取polygon[i]和polygon[j]之间的加权pd
  inline FT get_pd(size_t i, size_t j) {
    if (i == j) {
      return 0;
    }
    if (i > j) {
      i ^= j;
      j ^= i;
      i ^= j;
    }
    size_t idx = poly_num * i - i * (i + 1) / 2 + (j - i - 1);
    return glob_pd[idx] * miu[idx];
  }

  // 初始化权重均为1
  inline void initialize_miu() {
    auto size = miu.size();
    for (size_t i = 0; i < size; i++) {
      miu[i] = 1;
    }
  }

  // 更新权重
  void update_miu();

  // 获取权重
  inline double get_miu(size_t i, size_t j) {
    if (i == j) {
      return 0;
    } else if (i > j) {
      i ^= j;
      j ^= i;
      i ^= j;
    }
    size_t idx = poly_num * i - i * (i + 1) / 2 + (j - i - 1);
    return miu[idx];
  }

  // 获取关于polygon[p]的加权pd之和
  inline FT get_one_polygon_pd(size_t p) {
    FT t = 0;
    for (size_t i = 0; i < poly_num; i++) {
      t += get_pd(p, i);
    }
    return t;
  }

  // 获取所有polygon的加权pd之和
  inline FT get_total_pd() {
    FT total_pd = 0;
    size_t size = glob_pd.size();
    for (size_t i = 0; i < size; i++) {
      total_pd += glob_pd[i] * miu[i];
    }
    return total_pd;
  }

  // 获取所有polygon的纯pd之和
  inline FT get_pure_total_pd() {
    FT total_pd = 0;
    size_t size = glob_pd.size();
    for (size_t i = 0; i < size; i++) {
      total_pd += glob_pd[i];
    }
    return total_pd;
  }

  // 更新现有长度
  void update_cur_length();

  // 分支点管理
  size_t save_branch_point(size_t idx, FT before_pd, FT after_pd,
                            Transformation trans, Point_2 pos);
  void rollback_to_branch(size_t branch_id);
  void clear_branch_points();

  void debug_glob_pd();
};
}  // namespace nesting
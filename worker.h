#pragma once
#include <QDebug>
#include <QThread>
#include "interface.h"

class Worker : public QThread {
  Q_OBJECT
 public:
  Worker(QObject* parent = nullptr) : QThread(parent) {}
  ~Worker() {
    qDebug() << "~Worker()";
    // emit sendMessage("~Worker()");
    if (p != nullptr) {
      delete p;
    }
  }
  void requestQuit() {
    //if (p != nullptr) {
    //  p->layout.isRequestQuit = true;
    //}
    isRequestQuit = true;
  }
  void set(const bool _need_simplify,
           const double _top_offset,
           const double _left_offset,
           const double _bottom_offset,
           const double _right_offset,
           const double _part_offset,
           const double _sheet_width,
           const double _sheet_height,
           const size_t _max_time,
           const std::vector<nesting::geo::Polygon_with_holes_2>& _polygons,
           const std::vector<uint32_t>& _items_rotations,
           const std::vector<uint32_t>& _items_quantity,
           const bool _sa_enabled = false,
           const double _sa_temperature = 1.0,
           const double _sa_cooling_rate = 0.95,
           const size_t _sa_iterations = 10) {
    need_simplify = _need_simplify;
    top_offset = _top_offset;
    left_offset = _left_offset;
    bottom_offset = _bottom_offset;
    right_offset = _right_offset;
    part_offset = _part_offset;
    sheet_width = _sheet_width;
    sheet_height = _sheet_height;
    max_time = _max_time;
    polygons = _polygons;
    items_rotations = _items_rotations;
    items_quantity = _items_quantity;
    sa_enabled = _sa_enabled;
    sa_temperature = _sa_temperature;
    sa_cooling_rate = _sa_cooling_rate;
    sa_iterations = _sa_iterations;
  }
  void run() override {
    // 在这里执行耗时的操作
    qDebug() << "CustomThread is running in thread:"
             << QThread::currentThreadId();

    doWork();
  }

 signals:
  void resultReady(Solution,
                   const std::vector<nesting::Item>&,
                   const std::vector<nesting::Item>&);
  void sendMessage(const QString&);

 private:
  volatile bool isRequestQuit{false};
  bool need_simplify{false};
  double top_offset{0};
  double left_offset{0};
  double bottom_offset{0};
  double right_offset{0};
  double part_offset{0};
  double sheet_width{0};
  double sheet_height{0};
  size_t max_time{0};
  // 模拟退火参数
  bool sa_enabled{false};
  double sa_temperature{1.0};
  double sa_cooling_rate{0.95};
  size_t sa_iterations{10};
  std::vector<nesting::geo::Polygon_with_holes_2> polygons;
  std::vector<uint32_t> items_rotations;
  std::vector<uint32_t> items_quantity;
  std::function<void(const Solution&)> h =
      std::bind(&Worker::hook, this, std::placeholders::_1);
  nesting::Preprocess* p{nullptr};
  void hook(const Solution& solution) {
    emit resultReady(Solution(solution), p->simplified_items,
                     p->original_items);
  }
  void doWork() {
    qDebug() << "doWork START";
    try {
      p = new nesting::Preprocess(nesting::preprocess(
          need_simplify, top_offset, left_offset, bottom_offset, right_offset,
          part_offset, sheet_width, sheet_height, polygons, items_rotations,
          items_quantity));
    } catch (const std::runtime_error& e) {
      emit sendMessage(e.what());
    }
    try {
      // 设置模拟退火参数
      p->layout.saEnabled = sa_enabled;
      p->layout.saTemperature = sa_temperature;
      p->layout.saCoolingRate = sa_cooling_rate;
      p->layout.saIterations = sa_iterations;
      nesting::start(max_time, p->layout, h, &isRequestQuit);
    } catch (const std::runtime_error& e) {
      emit sendMessage(e.what());
    }
    delete p;
    p = nullptr;
    qDebug() << "doWork DONE";
  }
};
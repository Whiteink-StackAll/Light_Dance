#include "light_solver/one_euro_filter.hpp"
#include <cmath>

namespace light_solver {

// 低通滤波器实现
OneEuroFilter::LowPassFilter::LowPassFilter(double alpha)
    : alpha_(alpha), y_(0.0), s_(0.0), initialized_(false) {}

// 基础滤波（使用内部 alpha）
double OneEuroFilter::LowPassFilter::filter(double value) {
    double result;
    if (initialized_) {
        result = alpha_ * value + (1.0 - alpha_) * s_;
    } else {
        result = value;  // 首次输入直接赋值
        initialized_ = true;
    }
    y_ = value;
    s_ = result;
    return result;
}

// 带外部 alpha 的滤波（动态调整时使用）
double OneEuroFilter::LowPassFilter::filter(double value, double alpha) {
    double result = alpha * value + (1.0 - alpha) * s_;
    y_ = value;
    s_ = result;
    return result;
}

// 计算 alpha 值（基于截止频率和时间差）
double OneEuroFilter::alphaFromCutoff(double cutoff, double dt) {
    // 使用实际时间差 dt 计算
    double tau = 1.0 / (2.0 * M_PI * cutoff);
    return 1.0 / (1.0 + tau / dt);
}

// 一欧元滤波器构造函数
OneEuroFilter::OneEuroFilter(double min_cutoff, double beta, double d_cutoff)
    : min_cutoff_(min_cutoff),
      beta_(beta),
      d_cutoff_(d_cutoff),
      x_filter_(0.5),  // 初始 alpha 临时值（首次滤波会更新）
      dx_filter_(0.5),
      last_value_(0.0),
      last_timestamp_(0.0),
      initialized_(false) {}

// 核心滤波逻辑
double OneEuroFilter::filter(double value, double timestamp) {
    std::lock_guard<std::mutex> lock(mutex_);  // 线程安全
    
    // 首次调用初始化
    if (!initialized_) {
        last_value_ = value;
        last_timestamp_ = timestamp;
        x_filter_.filter(value);       // 初始化位置滤波器
        dx_filter_.filter(0.0);        // 初始化速度滤波器（初始速度为 0）
        initialized_ = true;
        return value;
    }
    
    // 计算时间差（秒）
    double dt = timestamp - last_timestamp_;
    if (dt <= 1e-6) {  // 防止时间差为 0 或负数
        dt = 1e-6;
    }
    
    // 1. 计算当前速度（数值微分）
    double dx = (value - last_value_) / dt;
    // 2. 平滑速度（使用速度滤波器）
    double edx = dx_filter_.filter(dx, alphaFromCutoff(d_cutoff_, dt));
    
    // 3. 计算自适应截止频率（随速度动态调整）
    double cutoff = min_cutoff_ + beta_ * std::fabs(edx);
    
    // 4. 平滑位置（使用位置滤波器）
    double result = x_filter_.filter(value, alphaFromCutoff(cutoff, dt));
    
    // 更新状态
    last_value_ = value;
    last_timestamp_ = timestamp;
    
    return result;
}

// 参数设置函数
void OneEuroFilter::setMinCutoff(double min_cutoff) {
    std::lock_guard<std::mutex> lock(mutex_);
    min_cutoff_ = min_cutoff;
}

void OneEuroFilter::setBeta(double beta) {
    std::lock_guard<std::mutex> lock(mutex_);
    beta_ = beta;
}

void OneEuroFilter::setDCutoff(double d_cutoff) {
    std::lock_guard<std::mutex> lock(mutex_);
    d_cutoff_ = d_cutoff;
}

// 重置滤波器状态
void OneEuroFilter::reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    initialized_ = false;
    last_value_ = 0.0;
    last_timestamp_ = 0.0;
}

} // namespace light_solver
    
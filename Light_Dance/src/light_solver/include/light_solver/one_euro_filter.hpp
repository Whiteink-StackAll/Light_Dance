#ifndef LIGHT_SOLVER_ONE_EURO_FILTER_HPP
#define LIGHT_SOLVER_ONE_EURO_FILTER_HPP

#include <mutex>

// 命名空间统一为 light_solver
namespace light_solver {

class OneEuroFilter {
public:
    OneEuroFilter(double min_cutoff = 0.004, double beta = 0.04, double d_cutoff = 1.0);
    
    // 核心滤波函数
    double filter(double value, double timestamp);
    
    // 参数设置
    void setMinCutoff(double min_cutoff);
    void setBeta(double beta);
    void setDCutoff(double d_cutoff);
    
    // 重置滤波器状态
    void reset();

private:
    // 低通滤波器辅助类
    class LowPassFilter {
    public:
        LowPassFilter(double alpha);
        
        // 修正：添加参数 alpha 的重载（原代码调用时需要）
        double filter(double value);
        double filter(double value, double alpha);
        
        bool hasLastValue() const { return initialized_; }
        double lastValue() const { return y_; }
        
    private:
        double alpha_;
        double y_;       // 原始输入值
        double s_;       // 滤波后的值
        bool initialized_;
    };
    
    // 计算低通滤波器的 alpha 值（内部工具函数）
    double alphaFromCutoff(double cutoff, double dt);
    
    double min_cutoff_;  // 最小截止频率
    double beta_;        // 速度增益
    double d_cutoff_;    // 导数截止频率
    
    LowPassFilter x_filter_;   // 位置滤波器
    LowPassFilter dx_filter_;  // 速度滤波器
    
    double last_value_;       // 上一时刻原始值
    double last_timestamp_;   // 上一时刻时间戳
    bool initialized_;        // 初始化标志
    std::mutex mutex_;        // 线程安全锁
};

} // namespace light_solver

#endif // LIGHT_SOLVER_ONE_EURO_FILTER_HPP
    
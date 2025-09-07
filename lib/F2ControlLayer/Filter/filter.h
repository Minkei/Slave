#ifndef FILTER_H
#define FILTER_H

# include <Arduino.h>


// Base class for all filters
class Filter {
public:
    virtual ~Filter() {};
    virtual float update(float input) = 0;
    virtual void reset() = 0;
};

// Moving Average Filter
class MovingAverageFilter : public Filter {
private:
    float* _buffer;
    int _size;
    int _index;
    bool _filled;

public:
    MovingAverageFilter(int size);
    ~MovingAverageFilter();
    float update(float input) override;
    void reset() override;
    void setSize(int size);
    int getSize() const {return _size;}
};

// Exponential Moving Average Filter
class EMAFilter : public Filter {
private:
    float _alpha;
    float _output;
    bool _initialized;
    
public:
    EMAFilter(float alpha = 0.2f);
    
    float update(float input) override;
    void reset() override;
    void setAlpha(float alpha);
    float getAlpha() const { return _alpha; }
};

// Low Pass Filter (1st order IIR)
class LowPassFilter : public Filter {
private:
    float _alpha;
    float _output;
    bool _initialized;
    
public:
    LowPassFilter(float cutoffFreq, float sampleRate);
    LowPassFilter(float alpha);
    
    float update(float input) override;
    void reset() override;
    void setCutoffFreq(float cutoffFreq, float sampleRate);
    void setAlpha(float alpha);
    float getAlpha() const { return _alpha; }
};

// Median Filter
class MedianFilter : public Filter {
private:
    float* _buffer;
    int _size;
    int _index;
    
    void sortBuffer(float* sorted);
    
public:
    MedianFilter(int size = 5);
    ~MedianFilter();
    
    float update(float input) override;
    void reset() override;
    void setSize(int size);
    int getSize() const { return _size; }
};

// Simple Kalman Filter
class SimpleKalmanFilter : public Filter {
private:
    float _q; // Process noise covariance
    float _r; // Measurement noise covariance
    float _x; // Estimated value
    float _p; // Estimation error covariance
    float _k; // Kalman gain
    bool _initialized;

public:
    SimpleKalmanFilter(float processNoise = 0.1f, float measurementNoise = 4.0f);
    float update(float input) override;
    void reset() override;
    void setProcessNoise(float q) { _q = q; }
    void setMeasurementNoise(float r) { _r = r; }
    float getEstimate() const { return _x; }
    float getErrorCovariance() const { return _p; }
};

// Multi-Stage Filter (combines multiple filters)
class MultiStageFilter : public Filter {
private:
    Filter** _filters;
    int _filterCount;
    
public:
    MultiStageFilter(Filter** filters, int count);
    ~MultiStageFilter();
    
    float update(float input) override;
    void reset() override;
    int getFilterCount() const { return _filterCount; }
};

// Adaptive Filter
class AdaptiveFilter : public Filter {
private:
    Filter* _currentFilter;
    enum FilterType {
        SMOOTH_FILTER,
        BALANCE_FILTER,
        FAST_FILTER
    };
    FilterType _currentFilterType;
    float _lowSpeedThreshold;
    float _highSpeedThreshold;
    float _lastInput;
    unsigned long _lastUpdateTime;

    void selectOptimalFilter(float input);

public:
    AdaptiveFilter (float lowSpeedThreshold = 20.0f, float highSpeedThreshold = 80.0f);
    ~AdaptiveFilter();

    float update(float input) override;
    void reset() override;
    void setThresholds(float lowThreshold, float highThreshold);

};

// Filter Factory for easy creation
class FilterFactory {
public:
    static MovingAverageFilter* createMovingAverage(int size = 5);
    static EMAFilter* createEMA(float alpha = 0.2f);
    static LowPassFilter* createLowPass(float cutoffFreq, float sampleRate);
    static MedianFilter* createMedian(int size = 5);
    static SimpleKalmanFilter* createKalman(float processNoise = 0.1f, float measurementNoise = 4.0f);
    
    // Pre-configured filters for RPM
    static Filter* createRPMFilter(); // Recommended RPM filter
    static Filter* createFastRPMFilter(); // For fast response
    static Filter* createSmoothRPMFilter(); // For maximum smoothness
    
    // Adaptive Filter
    static AdaptiveFilter* createAdaptiveFilter(float lowSpeedThreshold = 20.0f, float highSpeedThreshold = 80.0f);
};


#endif // FILTER_H
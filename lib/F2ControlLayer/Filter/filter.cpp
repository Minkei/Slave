#include "filter.h"
#include <string.h>
#include <math.h>

MovingAverageFilter::MovingAverageFilter(int size) : _size(size), _index(0), _filled(false)
{
    _buffer = new float[size];
    reset();
}

MovingAverageFilter::~MovingAverageFilter()
{
    delete[] _buffer;
}

float MovingAverageFilter::update(float input)
{
    _buffer[_index] = input;
    _index = (_index + 1) % _size;

    if (_index == 0)
        _filled = true;

    // Calculate average
    float sum = 0;
    int count = _filled ? _size : _index;
    for (int i = 0; i < count; i++)
    {
        sum += _buffer[i];
    }

    return sum / count;
}

void MovingAverageFilter::reset()
{
    for (int i = 0; i < _size; i++)
    {
        _buffer[i] = 0;
    }
    _index = 0;
    _filled = false;
}

void MovingAverageFilter::setSize(int size)
{
    if (size > 0 && size != _size)
    {
        delete[] _buffer;
        _size = size;
        _buffer = new float[size];
        reset();
    }
}

// =============== EMAFilter ===============

EMAFilter::EMAFilter(float alpha) : _alpha(alpha), _output(0), _initialized(false)
{
    if (_alpha < 0)
        _alpha = 0;
    if (_alpha > 1)
        _alpha = 1;
}

float EMAFilter::update(float input)
{
    if (!_initialized)
    {
        _output = input;
        _initialized = true;
        return _output;
    }

    _output = _alpha * input + (1.0f - _alpha) * _output;
    return _output;
}

void EMAFilter::reset()
{
    _output = 0;
    _initialized = false;
}

void EMAFilter::setAlpha(float alpha)
{
    if (alpha >= 0 && alpha <= 1)
    {
        _alpha = alpha;
    }
}

// =============== LowPassFilter ===============

LowPassFilter::LowPassFilter(float cutoffFreq, float sampleRate) : _output(0), _initialized(false)
{
    float dt = 1.0f / sampleRate;
    float RC = 1.0f / (2.0f * PI * cutoffFreq);
    _alpha = dt / (dt + RC);
}

LowPassFilter::LowPassFilter(float alpha) : _alpha(alpha), _output(0), _initialized(false)
{
    if (_alpha < 0)
        _alpha = 0;
    if (_alpha > 1)
        _alpha = 1;
}

float LowPassFilter::update(float input)
{
    if (!_initialized)
    {
        _output = input;
        _initialized = true;
        return _output;
    }

    _output = _alpha * input + (1.0f - _alpha) * _output;
    return _output;
}

void LowPassFilter::reset()
{
    _output = 0;
    _initialized = false;
}

void LowPassFilter::setCutoffFreq(float cutoffFreq, float sampleRate)
{
    float dt = 1.0f / sampleRate;
    float RC = 1.0f / (2.0f * PI * cutoffFreq);
    _alpha = dt / (dt + RC);
}

void LowPassFilter::setAlpha(float alpha)
{
    if (alpha >= 0 && alpha <= 1)
    {
        _alpha = alpha;
    }
}

// =============== MedianFilter ===============

MedianFilter::MedianFilter(int size) : _size(size), _index(0)
{
    _buffer = new float[size];
    reset();
}

MedianFilter::~MedianFilter()
{
    delete[] _buffer;
}

float MedianFilter::update(float input)
{
    _buffer[_index] = input;
    _index = (_index + 1) % _size;

    // Create sorted copy
    float *sorted = new float[_size];
    memcpy(sorted, _buffer, _size * sizeof(float));
    sortBuffer(sorted);

    float median = sorted[_size / 2];
    delete[] sorted;

    return median;
}

void MedianFilter::reset()
{
    for (int i = 0; i < _size; i++)
    {
        _buffer[i] = 0;
    }
    _index = 0;
}

void MedianFilter::sortBuffer(float *sorted)
{
    // Simple bubble sort (efficient for small arrays)
    for (int i = 0; i < _size - 1; i++)
    {
        for (int j = 0; j < _size - i - 1; j++)
        {
            if (sorted[j] > sorted[j + 1])
            {
                float temp = sorted[j];
                sorted[j] = sorted[j + 1];
                sorted[j + 1] = temp;
            }
        }
    }
}

void MedianFilter::setSize(int size)
{
    if (size > 0 && size != _size)
    {
        delete[] _buffer;
        _size = size;
        _buffer = new float[size];
        reset();
    }
}

// =============== SimpleKalmanFilter ===============

SimpleKalmanFilter::SimpleKalmanFilter(float processNoise, float measurementNoise)
    : _q(processNoise), _r(measurementNoise), _p(1.0f), _k(0), _x(0), _initialized(false)
{
}

float SimpleKalmanFilter::update(float input)
{
    if (!_initialized)
    {
        _x = input;
        _initialized = true;
        return _x;
    }

    // Prediction step
    _p = _p + _q;

    // Update step
    _k = _p / (_p + _r);
    _x = _x + _k * (input - _x);
    _p = (1 - _k) * _p;

    return _x;
}

void SimpleKalmanFilter::reset()
{
    _p = 1.0f;
    _k = 0;
    _x = 0;
    _initialized = false;
}

// =============== MultiStageFilter ===============

MultiStageFilter::MultiStageFilter(Filter **filters, int count)
    : _filterCount(count)
{
    _filters = new Filter *[count];
    for (int i = 0; i < count; i++)
    {
        _filters[i] = filters[i];
    }
}

MultiStageFilter::~MultiStageFilter()
{
    // Note: We don't delete individual filters as they may be managed elsewhere
    delete[] _filters;
}

float MultiStageFilter::update(float input)
{
    float output = input;
    for (int i = 0; i < _filterCount; i++)
    {
        output = _filters[i]->update(output);
    }
    return output;
}

void MultiStageFilter::reset()
{
    for (int i = 0; i < _filterCount; i++)
    {
        _filters[i]->reset();
    }
}

// =============== FilterFactory ===============

MovingAverageFilter *FilterFactory::createMovingAverage(int size)
{
    return new MovingAverageFilter(size);
}

EMAFilter *FilterFactory::createEMA(float alpha)
{
    return new EMAFilter(alpha);
}

LowPassFilter *FilterFactory::createLowPass(float cutoffFreq, float sampleRate)
{
    return new LowPassFilter(cutoffFreq, sampleRate);
}

MedianFilter *FilterFactory::createMedian(int size)
{
    return new MedianFilter(size);
}

SimpleKalmanFilter *FilterFactory::createKalman(float processNoise, float measurementNoise)
{
    return new SimpleKalmanFilter(processNoise, measurementNoise);
}

Filter *FilterFactory::createRPMFilter()
{
    // Recommended: EMA with moderate smoothing
    return new EMAFilter(0.25f);
}

Filter *FilterFactory::createFastRPMFilter()
{
    // For fast response: Higher alpha EMA
    return new EMAFilter(0.5f);
}

Filter *FilterFactory::createSmoothRPMFilter()
{
    // For maximum smoothness: Moving Average + EMA
    // Note: This would require MultiStageFilter implementation in practice
    return new MovingAverageFilter(7);
}

AdaptiveFilter *FilterFactory::createAdaptiveFilter(float lowSpeedThreshold, float highSpeedThreshold)
{
    return new AdaptiveFilter(lowSpeedThreshold, highSpeedThreshold);
}

void AdaptiveFilter::selectOptimalFilter(float input)
{
    float absInput = abs(input);
    unsigned long currentTime = millis();

    float rateOfChange = 0;
    if (_lastUpdateTime > 0)
    {
        float deltaTime = (currentTime - _lastUpdateTime) / 1000.0f; // Convert ms to seconds
        if (deltaTime > 0)
        {
            rateOfChange = abs(input - _lastInput) / deltaTime;
        }
    }

    FilterType newFilterType;
    if (absInput < _lowSpeedThreshold && rateOfChange < 10.0f)
    {
        newFilterType = SMOOTH_FILTER;
    }
    else if (absInput > _highSpeedThreshold || rateOfChange > 50.0f)
    {
        newFilterType = FAST_FILTER;
    }
    else
    {
        newFilterType = BALANCE_FILTER;
    }

    if (newFilterType != _currentFilterType)
    {
        Filter *_newFilter = nullptr;
        switch (newFilterType)
        {
        case SMOOTH_FILTER:
            _newFilter = new MovingAverageFilter(15);
            break;
        case FAST_FILTER:
            _newFilter = new EMAFilter(0.6f);
            break;
        case BALANCE_FILTER:
        default:
            _newFilter = new EMAFilter(0.25f);
            break;
        }
        if (_newFilter)
        {
            delete _currentFilter;
            _currentFilter = _newFilter;
            _currentFilterType = newFilterType;
        }
    }
    _lastInput = input;
    _lastUpdateTime = currentTime;
}

AdaptiveFilter::AdaptiveFilter(float lowSpeedThreshold, float highSpeedThreshold)
    : _lowSpeedThreshold(lowSpeedThreshold), _highSpeedThreshold(highSpeedThreshold), _currentFilterType(BALANCE_FILTER), _lastInput(0), _lastUpdateTime(0)
{
    _currentFilter = new EMAFilter(0.25f); // Start with balanced filter
}

AdaptiveFilter::~AdaptiveFilter()
{
    if (_currentFilter)
    {
        delete _currentFilter;
        _currentFilter = nullptr;
    }
}

float AdaptiveFilter::update(float input)
{
    selectOptimalFilter(input);
    return _currentFilter->update(input);
}

void AdaptiveFilter::reset()
{
    if (_currentFilter)
    {
        _currentFilter->reset();
    }
    _lastInput = 0;
    _lastUpdateTime = 0;
}

void AdaptiveFilter::setThresholds(float lowThreshold, float highThreshold)
{
    _lowSpeedThreshold = lowThreshold;
    _highSpeedThreshold = highThreshold;
}

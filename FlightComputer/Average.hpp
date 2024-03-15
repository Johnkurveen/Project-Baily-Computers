
#ifndef AVERAGE_HPP
#define AVERAGE_HPP

#include <cstdint>

template<typename T>
class Average {
  public:
    Average() = default;
    ~Average() = default;

    void insert(T entry) {
      _sum += entry;
      _count++;
    }

    T get() {
      if (_count == 0)
      {
        return static_cast<T>(0);
      }

      T result { _sum / static_cast<T>(_count) };
      _sum = static_cast<T>(0);
      _count = 0;
      return result;
    }

    uint32_t getCount() {
      return _count;
    }
  private:
    T _sum;
    uint32_t _count;
};

#endif // AVERAGE_HPP


#ifndef BUFFERED_FILE_HPP
#define BUFFERED_FILE_HPP

#include <SD.h>

template<size_t BUFFER_LEN>
class BufferedFile {
public:
  BufferedFile() = default;

  BufferedFile(char const* filename)
    : _file{ SD.open(filename, O_WRITE | O_CREAT) } {
  }
  
  operator bool() {
    return _file;
  }

  template<typename T>
  void push(T value) {
    memcpy(&_buffer[_bufferSize], &value, sizeof(T));
    _bufferSize += sizeof(T);
  }

  void doWrite() {
    postWrite();
  }

  void flush() {
    _file.flush();
  }

private:
  File _file;
  size_t _bufferSize { 0 };
  uint8_t _buffer[BUFFER_LEN]{ 0 };

  void postWrite() {
    size_t head {0};

    size_t toWrite { static_cast<size_t>(_file.availableForWrite()) };
    while (_bufferSize >= (toWrite + head)) {
      _file.write(&_buffer[head], toWrite);
      head += toWrite;
      toWrite = static_cast<size_t>(_file.availableForWrite()); // In case it changed
    }

    size_t copy { _bufferSize - head };
    for (size_t i { 0 }; i < copy; i++) {
      _buffer[i] = _buffer[i + head];
    }
    _bufferSize = copy;
  }
};

#endif  // BUFFERED_FILE_HPP

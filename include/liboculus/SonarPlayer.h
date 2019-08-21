#pragma once

#include <fstream>

#include "gpmf-parser/GPMF_parser.h"
#include "liboculus/SimplePingResult.h"

namespace liboculus {

class SonarPlayerBase {
public:
  SonarPlayerBase();
  virtual ~SonarPlayerBase();

  virtual bool open(const std::string &filename);
  virtual bool isOpen() const { return _input.is_open(); }

  virtual bool setStream(GPMF_stream *stream) { return false; }

  virtual bool eof() const { return _input.eof(); }
  virtual void rewind() { _input.seekg(0); }

  virtual std::shared_ptr<SimplePingResult> nextPing() = 0;

  static std::shared_ptr<SonarPlayerBase> OpenFile(const std::string &filename);
  static std::shared_ptr<SonarPlayerBase> createGPMFSonarPlayer();

protected:
  std::ifstream _input;
};

///
///
///
class RawSonarPlayer : public SonarPlayerBase {
public:
  RawSonarPlayer();
  virtual ~RawSonarPlayer();

  std::shared_ptr<MessageBuffer> nextPacket();
  virtual std::shared_ptr<SimplePingResult> nextPing();

private:
};

///
///
///
class OculusSonarPlayer : public SonarPlayerBase {
public:
  OculusSonarPlayer();
  virtual ~OculusSonarPlayer();

  // char *nextPacket();
  virtual std::shared_ptr<SimplePingResult> nextPing();

private:
};

///
///
///
class GPMFSonarPlayer : public SonarPlayerBase {
public:
  GPMFSonarPlayer();
  virtual ~GPMFSonarPlayer();

  virtual bool open(const std::string &filename);
  virtual bool setStream(GPMF_stream *stream);
  virtual bool isOpen() const { return _valid; }

  void close();

  virtual bool eof();
  virtual void rewind();

  // char *nextPacket();
  virtual std::shared_ptr<SimplePingResult> nextPing();

  void dumpGPMF(void);

private:
  GPMF_stream _stream;
  bool _valid;
  std::string _buffer;
};

} // namespace liboculus

#pragma once

#include <fstream>

#ifdef WITH_GPMF
  #include "gpmf-parser/GPMF_parser.h"
#endif

#include "liboculus/SimplePingResult.h"

namespace liboculus {

class SonarPlayerBase {
public:
  SonarPlayerBase();
  virtual ~SonarPlayerBase();

  virtual bool open(const std::string &filename);
  virtual bool isOpen() const { return _input.is_open(); }

  virtual bool eof() const { return _input.eof(); }
  virtual void rewind() { _input.seekg(0); }

  virtual std::shared_ptr<SimplePingResult> nextPing() = 0;

  static std::shared_ptr<SonarPlayerBase> OpenFile(const std::string &filename);

#ifdef WITH_GPMF
  virtual bool setStream(GPMF_stream *stream) { return false; }

  static std::shared_ptr<SonarPlayerBase> createGPMFSonarPlayer();
#endif

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

#ifdef WITH_GPMF
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
#endif

} // namespace liboculus

#pragma once

#include <thread>
#include <chrono>
#include <mutex>
#include <condition_variable>

#include <boost/asio.hpp>

#include "Oculus/Oculus.h"

namespace liboculus {


class SonarStatus
{
public:

  typedef std::chrono::time_point<std::chrono::system_clock> sys_time_point;

  SonarStatus();

  OculusStatusMsg operator()( void ) const;

  //std::mutex &mutex( void ) { return _writeLock; }

  bool valid() const { return _valid; }
  void dump() const;

  boost::asio::ip::address ipAddr() const;

  void update( const OculusStatusMsg &msg, sys_time_point msgTime = std::chrono::system_clock::now() );

protected:

  mutable std::mutex       _statusMutex;
  std::condition_variable  _statusUpdateCond;

private:

  bool                  _valid;
  OculusStatusMsg       _osm;             // The more recent status message
  sys_time_point        _msgTime;     // The time of the last message
};

}

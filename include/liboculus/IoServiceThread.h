#pragma once

#include <boost/asio.hpp>


namespace liboculus {


// Generic "worker thread" for boost::asio
class IoServiceThread {
public:
    IoServiceThread()
      : _service(),
        _thread() {}

    ~IoServiceThread() {}

    void fork()
    {
        if (_thread) return; // running

        _thread.reset(new std::thread(
            boost::bind(&boost::asio::io_service::run, &_service)
        ));
    }

    void stop()
    {
        if (!_thread) return; // stopped
        _service.stop();
    }

    void join()
    {
      if (!_thread) return; // stopped
      _thread->join();
      _service.reset();
      _thread.reset();
    }

    boost::asio::io_service &service()
    { return _service; }

private:
    boost::asio::io_service _service;
    std::unique_ptr<std::thread> _thread;
};

}

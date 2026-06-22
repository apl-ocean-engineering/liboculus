
# liboculus

(No, sadly, not that kind of [Oculus](https://www.oculus.com/))


> [!WARNING]
>
> January 2026
>
> This is the [main "v3" branch](https://github.com/apl-ocean-engineering/liboculus), in which G3Log has been replaced with [`spdlog`](https://github.com/gabime/spdlog).  It also includes other formatting and restructuring changes under the hood.   When in doubt, start here.  It builds in ROS1 or ROS2 workspaces, and in a plain CMake environment.
>
> The "v2" branch of this repo, along with the matching "v2" branch for [oculus_sonar_driver](https://gitlab.com/apl-ocean-engineering/oculus_sonar_driver/-/commits/v2) uses G3Log for logging.   It also includes the hybrid `CMakeLists.txt` for ROS1, ROS2 and CMake.
>
> The previous ROS1-only version has been archived as [`v1.2.0`](https://github.com/apl-ocean-engineering/liboculus/tree/v1.2.0).

This library can:

  - Communicate with a [Blueprint Subsea Oculus](https://www.blueprintsubsea.com/oculus/index.php) imaging sonar over
    its ethernet interface.
  - Request that the sonar start pinging.
  - Decode and parse fields from the resulting ping messages from the sonar.
  - Load and parse sonar data recorded as raw streams of binary packets.

The library contains no special provisions for *saving* sonar data,
but it's straightforward to write packets as a raw binary stream
(which the library can then read) -- see [`tools/oculus_client.cpp`](https://github.com/apl-ocean-engineering/liboculus/blob/main/tools/oculus_client.cpp) for an example.  

**liboculus cannot parse `.oculus` files saved from Blueprint's software.**


---
## Dependencies

In any of the environments, this package require `spdlog`, `fmt` and `boost::asio`

In ROS environments these can be installed automatically with `rosdep`.

On recent Ubuntu distros, the dependencies can be also installed with:

```
sudo apt install -y libfmt-dev libspdlog-dev libasio-dev libboost-system-dev
```

A recent `spdlog` is required, this package will not build with the version of spdlog in Ubuntu 20.04.

## Build/Installation

This is a hybrid repository which will build in either ROS1 or ROS2 workspaces, though there are no ROS dependencies in the code.

## Build with cmake

This package can also be built with the standard cmake process:

```
mkdir build && cd build
cmake ..
make
```

`CMakelists.txt` attempts to auto-detect ROS.  Cmake builds should be done in an environment where ROS has not been loaded (there are no `ROS_*` environment variables).

### Windows (MSVC)

See `BUILD_WINDOWS.md` for a repeatable Windows build flow that uses a clean
out-of-source directory and forces fetching fmt/spdlog to avoid version
conflicts.

## Logging

Internally the library uses [spdlog](https://github.com/gabime/spdlog).  The library's logger does not have any registered sinks by default and will not output to the console.

If the calling application uses `spdlog`, either the library's logger can be reset to the default logger:

```
liboculus::Logger::set_logger( spdlog::default_logger() );
```

or a sink can be added to the library's logger:

```
auto stdout_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt >();  liboculus::Logger::add_sink( stdout_sink );
```

### Fetching `spdlog` and `fmt` to avoid dependencies

Export the `FETCH_SPDLOG` variable to download and build static libraries for `spdlog` and `fmt`:

```
$ export FETCH_SPDLOG=1
$ cmake ../ # In build directory
[...]
-- {fmt} version: 12.1.0
-- Build type:
-- Build spdlog: 1.17.0
[...]
```

---
## oc_client binary

The repo contains one binary, `oc_client` which can read data either from a
real Oculus sonar via ethernet, or from a file containing raw Ethernet
data.

As noted above, it **cannot** read files saved by the proprietary Oculus GUI as that is based on a proprietary data format (independent from the `SimplePingResult` format used in this code).

Here's the help string for `oc_client`:

    Simple Oculus Sonar app
    Usage: oc_client [OPTIONS]

    Options:
      -h,--help                   Print this help message and exit
      -v,--verbose                Additional output (use -vv for even more!)
      --ip TEXT                   IP address of sonar or "auto" to automatically detect.
      -o,--output TEXT            Filename to save sonar data to.
      -i,--input TEXT             Filename to read sonar data from.
      -n,--frames INT             Stop after (n) frames.


The `--output` format works for both live data, and datafiles
specified with `--input`.  The generated files are raw binary
streams of sonar packets, and can be opened by `oc_client`.

## Library Design

See [oc_client](https://github.com/apl-ocean-engineering/liboculus/blob/main/tools/oculus_client.cpp) as a sample non-ROS client.   A typical client will have instances of two interface classes.  Both use Boost::Asio for network IO and must be given an [`boost::asio::io_context`](https://www.boost.org/doc/libs/1_79_0/doc/html/boost_asio/reference/io_context.html) on construction.

* [DataRx](https://github.com/apl-ocean-engineering/liboculus/blob/main/include/liboculus/DataRx.h) receives packets from the sonar, calling a callback function for each ping.
* [StatusRx](https://github.com/apl-ocean-engineering/liboculus/blob/main/include/liboculus/StatusRx.h) monitors the UDP broadcast-based protocol used to autodetect sonars on the network.   On receiving a good sonar status, it calls a callback.

The client must implement callbacks that will handle data from the sonar ([for example](https://github.com/apl-ocean-engineering/liboculus/blob/438f34a469eaf0d495ea515e86290b39cf965a20/tools/oculus_client.cpp#L131)) -- independent callbacks must be defined for the Oculus V1 and V2 packets.   DataRx also has a [callback on successful connection with a sonar](https://github.com/apl-ocean-engineering/liboculus/blob/438f34a469eaf0d495ea515e86290b39cf965a20/tools/oculus_client.cpp#L181) which can be used to send a configuration to the sonar (this will start the sonar pinging).

This library makes liberal use of overlay classes in order to provide
zero-copy accessor functions into the raw data chunks received from
the oculus.  These classes overlay the struct hierarchy defined in
[Oculus.h](include/liboculus/thirdparty/Oculus/Oculus.h), making it possible to directly cast between the types depending on which accessors you want to use:
* `OculusSimplePingResult` carries all image data from the oculus.
* Its first field is the `OculusSimpleFireMessage` that triggered data collection
* In turn, the first field of the `OculusSimpleFireMessage` is an `OculusMessageHeader`

So, in our code:
* `MessageHeader` ([SimplePingResult.h](include/liboculus/SimplePingResult.h))
  * Overlays `OculusMessageHeader` (there exists an accessor function that returns the original Oculus type)
  * However, it contains a buffer that will accept the full message payload, which is then used by other classes that provide accessors.
* `SimplePingResult` (SimplePingResult.{h,cpp}) overlays the `OculusSimplePingResult`.
  * `SimplePingResult` subclasses `MessageHeader`
  * Overlays both `OculusSimpleFireMessage` and `OculusSimplePingResult` (there are accessor functions that cast it to either)
  * It has instances of two other overlay classes, `BearingData` and `ImageData`.
* `BearingData` ([BearingData.h](include/liboculus/BearingData.h))
* `ImageData` ([ImageData.h](include/liboculus/ImageData.h)) overlays the buffer in a SimplePingResult, using OculusSimpleFireMessage.imageOffset to index into the buffer at the correct spot.


Other files/classes:
* [DataTypes.h](include/liboculus/DataTypes.h): Utility conversions for enums defined in [Oculus.h](include/liboculus/thirdparty/Oculus/Oculus.h)

* [StatusRx](include/liboculus/StatusRx.h): Connects to the broadcast status port; copies received messages into a SonarStatus and calls SonarClient's callback with the SonarStatus.
* [SonarStatus](include/liboculus/SonarStatus.h): Wrapper around OculusStatusMsg. Only used to dump it to LOG(DEBUG), so I'd like to see it disappear in favor of a log_status helper function.

* [IoServiceThread](include/liboculus/IoServiceThread.h): thin wrapper which runs a [`boost::asio::io_context`](https://www.boost.org/doc/libs/1_79_0/doc/html/boost_asio/reference/io_context.html) within a thread.  Used by both StatusRx and DataRx

----
# Related Packages

* [oculus_sonar_driver](https://gitlab.com/apl-ocean-engineering/oculus_sonar_driver) provides a ROS node for interfacing with the Oculus sonar.  It includes both ROS1 and ROS2 nodes on separate branches.

* [sonar_image_proc](https://github.com/apl-ocean-engineering/sonar_image_proc) contains code to postprocess sonar data, including drawing the sonar data to an OpenCV Mat (contains both a ROS node and non-ROS library).  **We are in the process of porting this to ROS2**

* [marine_acoustic_msgs](https://github.com/apl-ocean-engineering/marine_msgs/tree/main/marine_acoustic_msgs) defines the ROS [ProjectedSonarImage](https://github.com/apl-ocean-engineering/marine_msgs/blob/main/marine_acoustic_msgs/msg/ProjectedSonarImage.msg) message type published by [oculus_sonar_driver](https://gitlab.com/apl-ocean-engineering/oculus_sonar_driver).

* [rqt_sonar_image_view](https://github.com/apl-ocean-engineering/rqt_sonar_image_view) is an Rqt plugin for displaying sonar imagery (uses [sonar_image_proc](https://github.com/apl-ocean-engineering/sonar_image_proc)).   **This is ROS1-only.  We have no plans to port to ROS2.**

---
# License

This code is released under the [BSD 3-clause license](LICENSE).

This repository contains one file provided by Blueprint as part of their free "Oculus Viewer" sample application: ([include/liboculus/thirdparty/Oculus/Oculus.h](thirdpart/Oculus/Oculus.h)).  It describes their protocol and data formats.   This file is distributed under [GPLv3](https://www.gnu.org/licenses/gpl-3.0.en.html).

It includes the header-only version of [CLI11](https://github.com/CLIUtils/CLI11) at [`include/liboculus/thirdparty/CLI11/`](include/liboculus/thirdparty/CLI11/)

It includes the header-only version of TartanLlama's [`expected` implementation](https://github.com/TartanLlama/expected) at [`include/liboculus/thirdparty/expected.hpp`](include/liboculus/thirdparty/expected.hpp)

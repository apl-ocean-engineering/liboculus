
# liboculus

[![Build Status](https://github.drone.camhd.science/api/badges/apl-ocean-engineering/liboculus/status.svg)](https://github.drone.camhd.science/apl-ocean-engineering/liboculus)

(No, sadly, not that kind of [Oculus](https://www.oculus.com/))

This library contains code for:

  - Communicating with a [Blueprint Subsea Oculus](https://www.blueprintsubsea.com/oculus/index.php) imaging sonar over
    its ethernet interface.
  - Requesting that the sonar start pinging.
  - Decoding and parsing fields from the resulting ping messages from the sonar.
  - Loading and parsing sonar data recorded as either:
    - Raw streams of binary packets.
    - **Note:** This repo includes scaffold code for reading `.oculus` files saved from the Blueprint GUI, but that format is proprietary and undocumented **we cannot parse `.oculus` files.**

The library contains no special provisions for *saving* sonar data,
but it's straightforward to write packets as a raw binary stream
(which the library can read) -- see [`tools/oculus_client.cpp`](https://github.com/apl-ocean-engineering/liboculus/blob/main/tools/oculus_client.cpp) for an example.


---
## Build/Installation

This is a hybrid repository:

* We primarily build using catkin, though there are no ROS dependencies in the code. We hope the code is still useful for others looking to talk to the Oculus.

* Historically, the repo has also supported the [fips](http://floooh.github.io/fips/) C++ dependency management tool. To build with fips: `./fips build`

The primary dependency is on [g3log](https://github.com/KjellKod/g3log).
* If using catkin, there are two options:
  * clone [g3log_ros](https://gitlab.com/apl-ocean-engineering/g3log_ros) into your workspace's `src/` directory
  * use the provided `liboculus.rosinstall` file: `cd <catkin_ws>/src`; `vcs import --input liboculus/liboculus.repos`
* It will be handled automagically if using fips.

The (optional) test suite also requires Googletest and the (also optional)
binary `oc_client` requires [CLI11](https://github.com/CLIUtils/CLI11),
both of which are also handled by fips.

Internally, the ethernet interface uses
[Boost::asio](https://www.boost.org/doc/libs/1_66_0/doc/html/boost_asio.html).

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
thirdparty/Oculus/Oculus.h, making it possible to directly cast between the types depending on which accessors you want to use:
* OculusSimplePingResult carries all image data from the oculus.
* Its first field is the OculusSimpleFireMessage that triggered data collection
* In turn, the first field of the OculusSimpleFireMessage is an OculusMessageHeader

So, in our code:
* MessageHeader (SimplePingResult.h)
  * Overlays OculusMessageHeader (there exist an accessor function that returns the original Oculus type)
  * However, it contains a buffer that will accept the full message payload, which is then used by other classes that provide accessors.
* SimplePingResult (SimplePingResult.{h,cpp}) overlays the OculusSimplePingResult.
  * SimplePingResult subclasses MessageHeader
  * Overlays both OculusSimpleFireMessage and OculusSimplePingResult (there are accessor functions that cast it to either)
  * It has instances of two other overlay classes, BearingData and ImageData.
* BearingData (BearingData.h) TODO: I'm still confused by how this one actually gets the bearings.
* ImageData (ImageData.h) overlays the buffer in a SimplePingResult, using OculusSimpleFireMessage.imageOffset to index into the buffer at the correct spot.


Other files/classes:
* DataTypes.h: Utility conversions for enums defined in Oculus.h

* StatusRx: Connects to the fixed status port; stuffs messages into a SonarStatus and calls SonarClient's callback with the SonarStatus.
* SonarStatus: Wrapper around OculusStatusMsg. Only used to dump it to LOG(DEBUG), so I'd like to see it disappear in favor of a log_status helper function.

* IoServiceThread: thin wrapper which runs a [`boost::asio::io_context`](https://www.boost.org/doc/libs/1_79_0/doc/html/boost_asio/reference/io_context.html) within a thread.  Used by both StatusRx and DataRx

----
# Related Packages

* [sonar_image_proc](https://github.com/apl-ocean-engineering/sonar_image_proc) contains code to postprocess sonar data, including drawing the sonar data to an OpenCV Mat (contains both a ROS node and non-ROS library).
* [oculus_sonar_driver](https://gitlab.com/apl-ocean-engineering/oculus_sonar_driver) provides a ROS node for interfacing with the Oculus sonar.
* [acoustic_msgs](https://github.com/apl-ocean-engineering/hydrographic_msgs/tree/main/acoustic_msgs) defines the ROS [SonarImage](https://github.com/apl-ocean-engineering/hydrographic_msgs/blob/main/acoustic_msgs/msg/SonarImage.msg) message type published by [oculus_sonar_driver](https://gitlab.com/apl-ocean-engineering/oculus_sonar_driver).
* [rqt_sonar_image_view](https://github.com/apl-ocean-engineering/rqt_sonar_image_view) is an Rqt plugin for displaying sonar imagery (uses [sonar_image_proc](https://github.com/apl-ocean-engineering/sonar_image_proc))

---
# License

This code is released under the [BSD 3-clause license](LICENSE).

This repository contains one file provided by Blueprint as part of their free "Oculus Viewer" sample application ([thirdparty/Oculus/Oculus.h](thirdpart/Oculus/Oculus.h)), which describes their protocol and data formats.   This file is distributed under [GPLv3](https://www.gnu.org/licenses/gpl-3.0.en.html).

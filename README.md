
# liboculus

[![Build Status](https://github.drone.camhd.science/api/badges/apl-ocean-engineering/liboculus/status.svg)](https://github.drone.camhd.science/apl-ocean-engineering/liboculus)

(No, sadly, not that kind of [Oculus](https://www.oculus.com/))

This library contains code for:

  - Communicating with a [Blueprint Subsea Oculus](https://www.blueprintsubsea.com/oculus/index.php) imaging sonar over
    its ethernet interface.

  - Requesting that the sonar ping.

  - Decoding and parsing fields from the resulting ping messages from the sonar.

  - Loading and parsing sonar data recorded as either:
    - Raw streams of binary packets.
    - Data encoded in the GPMF format by `serdp_recorder`
    - **Note:** This repo includes scaffold code for reading `.oculus` files saved from the Blueprint GUI, but as that format is proprietary and undocumented **we cannot actually parse `.oculus` files.**

The library contains no special provisions for _saving_ sonar data,
but it's straightforward to write packets as a raw binary stream
(which the library can read) -- see `tools/oculus_client.cpp` for an example.

## Build/Installation

This is a hybrid repository:

* We primarily build using catkin, though there are no ROS dependencies in the code. We hope the code is still useful for others looking to talk to the Oculus.

* Historically, the repo has also supported the [fips](http://floooh.github.io/fips/) C++ dependency management tool. To build with fips: `./fips build`

The primary dependency is on [g3log](https://github.com/KjellKod/g3log).
* If using catkin, there are two options:
  * clone [g3log_ros](https://gitlab.com/apl-ocean-engineering/g3log_ros)
  * use the provided liboculus.repos: `cd <catkin_ws>/src`; `vcs import --input liboculus/liboculus.repos`
* It will be handled automagically if using fips.

The (optional) test suite also requires Googletest and the (also optional)
binary `oc_client` requires [CLI11](https://github.com/CLIUtils/CLI11),
both of which are also handled by fips.

Internally, the ethernet interface uses
[Boost::asio](https://www.boost.org/doc/libs/1_66_0/doc/html/boost_asio.html),
so Boost needs to be installed.



## oc_client binary

The repo contains one binary, `oc_client` which can read data either from a
real Oculus sonar via ethernet, or from a file containing raw Ethernet
data, or in our GPMF-based data format.   As above, it can't actually read
files saved by the Oculus client as it saves a ping message format for which
we don't have the data format.

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

The driver using this library is expected to instantiate a SonarClient, and hook up two interfaces:
1. Instantiate a SonarConfig that will be kept updated with desired parameters; the SonarClient takes this as an argument at construction and keeps a reference to it.
1. implement a callback that will handle data received from the sensor.

Thus, the data flow is:
* Updating configuration:
  - Driver decides configuration needs to be updated. In the case of `oculus_sonar_driver`, this is triggered by `dynamic_reconfigure`. Driver calls `SonarConfiguration::.set{Range,etc.}`. Driver *owns* an instance of SonarConfig.
  - When updated, `SonarConfig` has a pointer to a callback that it calls, passing a pointer to itself.
  - That configCallback is responsible for actually sending it to the instrument. This is done by DataRx::sendConfiguration.
* Receiving
  - DataRx listens on the specified port. The message is read in two chunks, by `DataRx::readHeader` and `DataRx::readSimplePingResult`. The second of those stuffs the data into a `SimplePingResult` (which in turn is a thin wrapper around the Oculus-defined struct) and calls the provided callback.
  - This callback was set by ...

* Receiving status
  - StatusRx

The SonarClient itself owns a StatusRx and a DataRx, each of which listens on a fixed port for the given messages.
* The StatusRx callback simply prints warnings, and attempts to connect the DataRx using the received address.
* The DataRx


The SonarConfiguration is really obnoxious:
* the oculus_driver owns it.
* SonarClient is constructed with a config and keeps a reference
* SonarClient passes the reference to DataRx
* DataRx sets the callback that causes ges from the oculus_driver to actually be sent to the instrument.



This library makes liberal use of overlay classes in order to provide
zero-copy accessor functions into the raw data chunks received from
the oculus.

These classes overlay the struct hierarchy defined in
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

* IoServiceThread: thin wrapper around boost::asio functions providing a simple worker thread; used by both StatusRx and DataRx


## License

This code is released under the [BSD 3-clause license](LICENSE).

This repository contains one file provided by Blueprint as part of their free "Oculus Viewer" sample application ([thirdparty/Oculus/Oculus.h](thirdpart/Oculus/Oculus.h)), which describes their protocol and data formats.   This file is distributed under [GPLv3](https://www.gnu.org/licenses/gpl-3.0.en.html).
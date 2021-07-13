
# liboculus

[![Build Status](https://github.drone.camhd.science/api/badges/apl-ocean-engineering/liboculus/status.svg)](https://github.drone.camhd.science/apl-ocean-engineering/liboculus)

(No, sadly, not that kind of [Oculus](https://www.oculus.com/))

This library contains code for:

  - Communicating with a [Blueprint Subsea Oculus](https://www.blueprintsubsea.com/oculus/index.php) imaging sonar over
    its ethernet interface.

  - Decoding and parsing fields from the resulting ping messages.

  - Loading and parsing sonar data stored as either:
    - Raw streams of binary packets.
    - Data encoded in the GPMF format by `serdp_recorder`
    - (A scaffold exists for reading files saved by in GUI provided by Oculus,
      but it receives sonar data in an undocumented ping format).

The library contains no special provisions for _saving_ sonar data,
but it's straightforward to write packets as a raw binary stream
(which the library can read) -- see `tools/oculus_client.cpp` for an example.

This is a hybrid repository:

* The native build mechanism is the
[fips](http://floooh.github.io/fips/) C++
dependency management tool,
however we hope the code is still useful for others looking to
talk to the Oculus.

* This repo will also build in ROS / Catkin.  The `.rosinstall` file at the top
level will work with `wstool` to install dependencies.

The code has a few dependencies:

  - [libactiveobject](https://github.com/apl-ocean-engineering/libbinlogger) is our internal collection of thread synchronization classes.
  - [libg3logger](https://github.com/apl-ocean-engineering/libg3logger) is our version of [g3log](https://github.com/apl-ocean-engineering/libg3logger) which is used for async logging.

Both of these dependencies are handled by fips / wstool.

The (optional) test suite also requires Googletest and the (also optional)
binary `oc_client` requires [CLI11](https://github.com/CLIUtils/CLI11),
both of which are also handled by fips.

Internally, the ethernet interface uses
[Boost::asio](https://www.boost.org/doc/libs/1_66_0/doc/html/boost_asio.html),
so Boost needs to be installed.

To build (with fips):

  ./fips build    // Will probably work


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


Other files:
* DataTypes.h: Utility conversions for enums defined in Oculus.h


## License

This code is released under the [BSD 3-clause license](LICENSE).

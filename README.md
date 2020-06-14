
# liboculus

[![Build Status](https://github.drone.camhd.science/api/badges/apl-ocean-engineering/liboculus/status.svg)](https://github.drone.camhd.science/apl-ocean-engineering/liboculus)

(No, sadly, not that kind of [Oculus](https://www.oculus.com/))

This library contains code for:

  - Communicating with a [Blueprint Subsea Oculus](https://www.blueprintsubsea.com/oculus/index.php) multibeam sonar over
    its ethernet interface.

  - Decoding and parsing fields from the resulting ping messages.

  - Loading and parsing sonar data stored as either:
    - Raw streams of binary packets.
    - Data encoded in the GPMF format by `serdp_recorder`
    - (A scaffold exists for reading files saved by the Oculus client, but it receives sonar data in an undocumented ping format).

The library contains no special provisions for _saving_ sonar data,
but it's straightforward to write packets as a raw binary stream
(which the library can read).

We use the [fips](http://floooh.github.io/fips/) C++
dependency management tool, so this library is based on fips,
however we hope the code is still useful for others looking to
talk to the Oculus.

The code has a few dependencies:

  - [libactiveobject](https://github.com/apl-ocean-engineering/libbinlogger) is our internal collection of thread synchronization classes.
  - [libg3logger](https://github.com/apl-ocean-engineering/libg3logger) is our version of [g3log](https://github.com/apl-ocean-engineering/libg3logger) which is used for async logging.

Both of these dependencies are handled by fips.

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
we don't have the data structure.

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

## License

This code is released under the [BSD 3-clause license](LICENSE).


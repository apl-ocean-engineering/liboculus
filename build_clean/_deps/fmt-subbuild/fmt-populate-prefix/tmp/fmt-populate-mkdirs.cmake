# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "F:/Projects/liboculus/build_clean/_deps/fmt-src"
  "F:/Projects/liboculus/build_clean/_deps/fmt-build"
  "F:/Projects/liboculus/build_clean/_deps/fmt-subbuild/fmt-populate-prefix"
  "F:/Projects/liboculus/build_clean/_deps/fmt-subbuild/fmt-populate-prefix/tmp"
  "F:/Projects/liboculus/build_clean/_deps/fmt-subbuild/fmt-populate-prefix/src/fmt-populate-stamp"
  "F:/Projects/liboculus/build_clean/_deps/fmt-subbuild/fmt-populate-prefix/src"
  "F:/Projects/liboculus/build_clean/_deps/fmt-subbuild/fmt-populate-prefix/src/fmt-populate-stamp"
)

set(configSubDirs Debug)
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "F:/Projects/liboculus/build_clean/_deps/fmt-subbuild/fmt-populate-prefix/src/fmt-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "F:/Projects/liboculus/build_clean/_deps/fmt-subbuild/fmt-populate-prefix/src/fmt-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()

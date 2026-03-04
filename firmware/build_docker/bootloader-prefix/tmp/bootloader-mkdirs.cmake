# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/opt/esp/idf/components/bootloader/subproject"
  "/work/build_docker/bootloader"
  "/work/build_docker/bootloader-prefix"
  "/work/build_docker/bootloader-prefix/tmp"
  "/work/build_docker/bootloader-prefix/src/bootloader-stamp"
  "/work/build_docker/bootloader-prefix/src"
  "/work/build_docker/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/work/build_docker/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/work/build_docker/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()

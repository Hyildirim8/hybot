# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/opt/esp/idf/components/bootloader/subproject"
  "/work/firmware/build_flash/bootloader"
  "/work/firmware/build_flash/bootloader-prefix"
  "/work/firmware/build_flash/bootloader-prefix/tmp"
  "/work/firmware/build_flash/bootloader-prefix/src/bootloader-stamp"
  "/work/firmware/build_flash/bootloader-prefix/src"
  "/work/firmware/build_flash/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/work/firmware/build_flash/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/work/firmware/build_flash/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()

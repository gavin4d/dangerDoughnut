# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/nix/store/b0d45nsqi08wyqvfqadgmxnw820fpfrz-esp-idf-v5.4/components/bootloader/subproject"
  "/home/gavin4d/projects/DangerDoughnut/firmware/build/bootloader"
  "/home/gavin4d/projects/DangerDoughnut/firmware/build/bootloader-prefix"
  "/home/gavin4d/projects/DangerDoughnut/firmware/build/bootloader-prefix/tmp"
  "/home/gavin4d/projects/DangerDoughnut/firmware/build/bootloader-prefix/src/bootloader-stamp"
  "/home/gavin4d/projects/DangerDoughnut/firmware/build/bootloader-prefix/src"
  "/home/gavin4d/projects/DangerDoughnut/firmware/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/gavin4d/projects/DangerDoughnut/firmware/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/gavin4d/projects/DangerDoughnut/firmware/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()

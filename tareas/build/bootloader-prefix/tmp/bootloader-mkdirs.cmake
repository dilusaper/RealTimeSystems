# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/dilusaper/esp/v5.4.1/esp-idf/components/bootloader/subproject"
  "/home/dilusaper/Escritorio/SistemasEnTiempoReal/RealTimeSystems/tareas/build/bootloader"
  "/home/dilusaper/Escritorio/SistemasEnTiempoReal/RealTimeSystems/tareas/build/bootloader-prefix"
  "/home/dilusaper/Escritorio/SistemasEnTiempoReal/RealTimeSystems/tareas/build/bootloader-prefix/tmp"
  "/home/dilusaper/Escritorio/SistemasEnTiempoReal/RealTimeSystems/tareas/build/bootloader-prefix/src/bootloader-stamp"
  "/home/dilusaper/Escritorio/SistemasEnTiempoReal/RealTimeSystems/tareas/build/bootloader-prefix/src"
  "/home/dilusaper/Escritorio/SistemasEnTiempoReal/RealTimeSystems/tareas/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/dilusaper/Escritorio/SistemasEnTiempoReal/RealTimeSystems/tareas/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/dilusaper/Escritorio/SistemasEnTiempoReal/RealTimeSystems/tareas/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()

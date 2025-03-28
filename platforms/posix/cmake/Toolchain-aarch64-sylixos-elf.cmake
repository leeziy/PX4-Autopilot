set(triple aarch64-sylixos-elf)

set(CMAKE_LIBRARY_ARCHITECTURE ${triple})
set(TOOLCHAIN_PREFIX ${triple})

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_SYSTEM_VERSION 1)

set(CMAKE_C_COMPILER ${TOOLCHAIN_PREFIX}-gcc.exe)
set(CMAKE_C_COMPILER_TARGET ${triple})

set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PREFIX}-g++.exe)
set(CMAKE_CXX_COMPILER_TARGET ${triple})

set(CMAKE_ASM_COMPILER ${TOOLCHAIN_PREFIX}-gcc.exe)

# compiler tools
find_program(CMAKE_AR ${TOOLCHAIN_PREFIX}-gcc-ar.exe)
find_program(CMAKE_GDB ${TOOLCHAIN_PREFIX}-gdb.exe)
find_program(CMAKE_LD ${TOOLCHAIN_PREFIX}-ld.exe)
find_program(CMAKE_LINKER ${TOOLCHAIN_PREFIX}-ld.exe)
find_program(CMAKE_NM ${TOOLCHAIN_PREFIX}-gcc-nm.exe)
find_program(CMAKE_OBJCOPY ${TOOLCHAIN_PREFIX}-objcopy.exe)
find_program(CMAKE_OBJDUMP ${TOOLCHAIN_PREFIX}-objdump.exe)
find_program(CMAKE_RANLIB ${TOOLCHAIN_PREFIX}-gcc-ranlib.exe)
find_program(CMAKE_STRIP ${TOOLCHAIN_PREFIX}-strip.exe)

set(CMAKE_FIND_ROOT_PATH get_file_component(${CMAKE_C_COMPILER} PATH))
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

# os tools
foreach(tool grep make)
	string(TOUPPER ${tool} TOOL)
	find_program(${TOOL} ${tool})
	if(NOT ${TOOL})
		message(FATAL_ERROR "could not find ${tool}")
	endif()
endforeach()

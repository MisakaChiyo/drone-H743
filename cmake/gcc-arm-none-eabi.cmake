set(CMAKE_SYSTEM_NAME               Generic)
set(CMAKE_SYSTEM_PROCESSOR          arm)

set(CMAKE_C_COMPILER_ID GNU)
set(CMAKE_CXX_COMPILER_ID GNU)

# Resolve the GNU Arm toolchain from either an STM32Cube bundle/CLT install or PATH.
unset(TOOLCHAIN_PREFIX)
unset(STM32CLT_ROOT)

if(DEFINED ENV{ARM_GCC_TOOLCHAIN_BIN} AND NOT "$ENV{ARM_GCC_TOOLCHAIN_BIN}" STREQUAL "")
    set(TOOLCHAIN_PREFIX "$ENV{ARM_GCC_TOOLCHAIN_BIN}/arm-none-eabi-")
elseif(DEFINED ENV{CUBE_BUNDLE_PATH} AND NOT "$ENV{CUBE_BUNDLE_PATH}" STREQUAL "")
    file(GLOB STM32_GNU_TOOLCHAIN_DIRS
        LIST_DIRECTORIES true
        "$ENV{CUBE_BUNDLE_PATH}/gnu-tools-for-stm32/*"
    )
    list(SORT STM32_GNU_TOOLCHAIN_DIRS COMPARE NATURAL ORDER DESCENDING)
    list(LENGTH STM32_GNU_TOOLCHAIN_DIRS STM32_GNU_TOOLCHAIN_COUNT)
    if(STM32_GNU_TOOLCHAIN_COUNT GREATER 0)
        list(GET STM32_GNU_TOOLCHAIN_DIRS 0 STM32_GNU_TOOLCHAIN_DIR)
        set(TOOLCHAIN_PREFIX "${STM32_GNU_TOOLCHAIN_DIR}/bin/arm-none-eabi-")
    endif()
elseif(WIN32)
    file(GLOB STM32CLT_DIRS
        LIST_DIRECTORIES true
        "E:/ST/STM32CubeCLT_*"
        "C:/ST/STM32CubeCLT_*"
        "$ENV{ProgramFiles}/STMicroelectronics/STM32Cube/STM32CubeCLT_*"
    )
    list(SORT STM32CLT_DIRS COMPARE NATURAL ORDER DESCENDING)
    foreach(STM32CLT_DIR IN LISTS STM32CLT_DIRS)
        if(EXISTS "${STM32CLT_DIR}/GNU-tools-for-STM32/bin/arm-none-eabi-gcc.exe")
            set(TOOLCHAIN_PREFIX "${STM32CLT_DIR}/GNU-tools-for-STM32/bin/arm-none-eabi-")
            set(STM32CLT_ROOT "${STM32CLT_DIR}")
            break()
        endif()
    endforeach()
else()
    file(GLOB STM32_GNU_TOOLCHAIN_DIRS
        LIST_DIRECTORIES true
        "$ENV{HOME}/Library/Application Support/stm32cube/bundles/gnu-tools-for-stm32/*"
    )
    list(SORT STM32_GNU_TOOLCHAIN_DIRS COMPARE NATURAL ORDER DESCENDING)
    list(LENGTH STM32_GNU_TOOLCHAIN_DIRS STM32_GNU_TOOLCHAIN_COUNT)
    if(STM32_GNU_TOOLCHAIN_COUNT GREATER 0)
        list(GET STM32_GNU_TOOLCHAIN_DIRS 0 STM32_GNU_TOOLCHAIN_DIR)
        set(TOOLCHAIN_PREFIX "${STM32_GNU_TOOLCHAIN_DIR}/bin/arm-none-eabi-")
    endif()
endif()

if(NOT TOOLCHAIN_PREFIX)
    find_program(ARM_NONE_EABI_GCC arm-none-eabi-gcc)
    if(ARM_NONE_EABI_GCC)
        get_filename_component(ARM_NONE_EABI_BIN_DIR "${ARM_NONE_EABI_GCC}" DIRECTORY)
        set(TOOLCHAIN_PREFIX "${ARM_NONE_EABI_BIN_DIR}/arm-none-eabi-")
    endif()
endif()

if(WIN32)
    set(ARM_GCC_SUFFIX ".exe")
else()
    set(ARM_GCC_SUFFIX "")
endif()

if(NOT TOOLCHAIN_PREFIX OR
   NOT EXISTS "${TOOLCHAIN_PREFIX}gcc${ARM_GCC_SUFFIX}")
    message(FATAL_ERROR
        "Could not locate arm-none-eabi-gcc. Set ARM_GCC_TOOLCHAIN_BIN, "
        "CUBE_BUNDLE_PATH, or add the GNU Arm toolchain to PATH."
    )
endif()

if(NOT STM32CLT_ROOT)
    get_filename_component(ARM_GCC_BIN_DIR "${TOOLCHAIN_PREFIX}gcc${ARM_GCC_SUFFIX}" DIRECTORY)
    get_filename_component(ARM_GCC_TOOLCHAIN_DIR "${ARM_GCC_BIN_DIR}" DIRECTORY)
    get_filename_component(STM32CLT_ROOT "${ARM_GCC_TOOLCHAIN_DIR}" DIRECTORY)
endif()

if(NOT CMAKE_MAKE_PROGRAM)
    if(WIN32 AND EXISTS "${STM32CLT_ROOT}/Ninja/bin/ninja.exe")
        set(CMAKE_MAKE_PROGRAM "${STM32CLT_ROOT}/Ninja/bin/ninja.exe" CACHE FILEPATH "" FORCE)
    else()
        find_program(NINJA_PROGRAM ninja)
        if(NINJA_PROGRAM)
            set(CMAKE_MAKE_PROGRAM "${NINJA_PROGRAM}" CACHE FILEPATH "" FORCE)
        endif()
    endif()
endif()

set(CMAKE_C_COMPILER                "${TOOLCHAIN_PREFIX}gcc${ARM_GCC_SUFFIX}" CACHE FILEPATH "" FORCE)
set(CMAKE_ASM_COMPILER              ${CMAKE_C_COMPILER})
set(CMAKE_CXX_COMPILER              "${TOOLCHAIN_PREFIX}g++${ARM_GCC_SUFFIX}" CACHE FILEPATH "" FORCE)
set(CMAKE_LINKER                    "${TOOLCHAIN_PREFIX}g++${ARM_GCC_SUFFIX}" CACHE FILEPATH "" FORCE)
set(CMAKE_OBJCOPY                   "${TOOLCHAIN_PREFIX}objcopy${ARM_GCC_SUFFIX}" CACHE FILEPATH "" FORCE)
set(CMAKE_SIZE                      "${TOOLCHAIN_PREFIX}size${ARM_GCC_SUFFIX}" CACHE FILEPATH "" FORCE)
set(CMAKE_AR                        "${TOOLCHAIN_PREFIX}ar${ARM_GCC_SUFFIX}" CACHE FILEPATH "" FORCE)
set(CMAKE_RANLIB                    "${TOOLCHAIN_PREFIX}ranlib${ARM_GCC_SUFFIX}" CACHE FILEPATH "" FORCE)
set(CMAKE_NM                        "${TOOLCHAIN_PREFIX}nm${ARM_GCC_SUFFIX}" CACHE FILEPATH "" FORCE)
set(CMAKE_OBJDUMP                   "${TOOLCHAIN_PREFIX}objdump${ARM_GCC_SUFFIX}" CACHE FILEPATH "" FORCE)
set(CMAKE_READELF                   "${TOOLCHAIN_PREFIX}readelf${ARM_GCC_SUFFIX}" CACHE FILEPATH "" FORCE)
set(CMAKE_ADDR2LINE                 "${TOOLCHAIN_PREFIX}addr2line${ARM_GCC_SUFFIX}" CACHE FILEPATH "" FORCE)

set(CMAKE_EXECUTABLE_SUFFIX_ASM     ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_C       ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX     ".elf")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# MCU specific flags
set(TARGET_FLAGS "-mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard ")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${TARGET_FLAGS}")
set(CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp -MMD -MP")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -fdata-sections -ffunction-sections -fstack-usage")

# The cyclomatic-complexity parameter must be defined for the Cyclomatic complexity feature in STM32CubeIDE to work.
# However, most GCC toolchains do not support this option, which causes a compilation error; for this reason, the feature is disabled by default.
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fcyclomatic-complexity")

set(CMAKE_C_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_C_FLAGS_RELEASE "-Os -g0")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_CXX_FLAGS_RELEASE "-Os -g0")

set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -fno-rtti -fno-exceptions -fno-threadsafe-statics")

set(CMAKE_EXE_LINKER_FLAGS "${TARGET_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T \"${CMAKE_SOURCE_DIR}/STM32H743XX_FLASH.ld\"")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --specs=nano.specs")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${CMAKE_PROJECT_NAME}.map -Wl,--gc-sections")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--print-memory-usage")
set(TOOLCHAIN_LINK_LIBRARIES "m")

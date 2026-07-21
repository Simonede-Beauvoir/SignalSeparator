# Prevent accidental repeated inclusion.
include_guard(GLOBAL)

# CMSIS-DSP repository root.
set(
        CMSIS_DSP_ROOT
        "${PROJECT_SOURCE_DIR}/CMSIS-DSP"
        CACHE PATH
        "Path to the CMSIS-DSP repository"
)

# CMSIS-Core root.
#
# CMSIS-DSP internally appends /Include to this path, therefore this must
# point to Drivers/CMSIS rather than Drivers/CMSIS/Include.
set(
        CMSISCORE
        "${PROJECT_SOURCE_DIR}/Drivers/CMSIS"
        CACHE PATH
        "Path to the CMSIS-Core root"
)

# Verify that the required sources are present.
if(NOT EXISTS "${CMSIS_DSP_ROOT}/Source/CMakeLists.txt")
    message(
            FATAL_ERROR
            "CMSIS-DSP was not found at:\n"
            "  ${CMSIS_DSP_ROOT}\n"
            "Expected file:\n"
            "  ${CMSIS_DSP_ROOT}/Source/CMakeLists.txt"
    )
endif()

if(NOT EXISTS "${CMSISCORE}/Include/core_cm7.h")
    message(
            FATAL_ERROR
            "CMSIS-Core was not found at:\n"
            "  ${CMSISCORE}\n"
            "Expected file:\n"
            "  ${CMSISCORE}/Include/core_cm7.h"
    )
endif()

#
# CMSIS-DSP build configuration.
#
# STM32H743 has no native half-precision floating-point acceleration.
# Disabling float16 also avoids compiler compatibility problems.
#
set(DISABLEFLOAT16 ON CACHE BOOL "Disable float16 CMSIS-DSP kernels" FORCE)

#
# Enable the modules needed by the signal separator.
#
# BasicMath:
#   Vector addition, subtraction, multiplication and scaling.
#
# ComplexMath:
#   Complex magnitude and related FFT post-processing.
#
# FastMath:
#   Fast sine, cosine, square root and atan2 functions.
#
# Statistics:
#   Mean, RMS, variance and related analysis.
#
# Support:
#   Data conversion and support functions.
#
# Transform:
#   CFFT, RFFT and other transforms.
#
set(BASICMATH ON CACHE BOOL "Build CMSIS-DSP basic math functions" FORCE)
set(COMPLEXMATH ON CACHE BOOL "Build CMSIS-DSP complex math functions" FORCE)
set(FASTMATH ON CACHE BOOL "Build CMSIS-DSP fast math functions" FORCE)
set(STATISTICS ON CACHE BOOL "Build CMSIS-DSP statistics functions" FORCE)
set(SUPPORT ON CACHE BOOL "Build CMSIS-DSP support functions" FORCE)
set(TRANSFORM ON CACHE BOOL "Build CMSIS-DSP transform functions" FORCE)

#
# Disable modules not currently needed.
#
set(CONTROLLER OFF CACHE BOOL "Build controller functions" FORCE)
set(FILTERING OFF CACHE BOOL "Build filtering functions" FORCE)
set(MATRIX OFF CACHE BOOL "Build matrix functions" FORCE)
set(SVM OFF CACHE BOOL "Build SVM functions" FORCE)
set(BAYES OFF CACHE BOOL "Build Bayesian functions" FORCE)
set(DISTANCE OFF CACHE BOOL "Build distance functions" FORCE)
set(INTERPOLATION OFF CACHE BOOL "Build interpolation functions" FORCE)
set(QUATERNIONMATH OFF CACHE BOOL "Build quaternion functions" FORCE)

#
# Do not use the old ARM_DSP_CONFIG_TABLES mechanism.
#
# Current CMSIS-DSP versions rely on the linker to remove unused FFT tables.
# Use size-specific initialization functions such as:
#
#   arm_rfft_fast_init_4096_f32()
#
# instead of the generic arm_rfft_fast_init_f32().
#

# Add CMSIS-DSP as an independent CMake subproject.
add_subdirectory(
        "${CMSIS_DSP_ROOT}/Source"
        "${CMAKE_BINARY_DIR}/cmsis_dsp"
)

if(NOT TARGET CMSISDSP)
    message(
            FATAL_ERROR
            "CMSIS-DSP did not create the expected CMSISDSP target."
    )
endif()

#
# Optimize only CMSIS-DSP itself.
#
# The main application can remain a Debug build while the DSP kernels are
# compiled with performance-oriented options.
#
set(
        CMSIS_DSP_OPTIMIZED_TARGETS
        CMSISDSPBasicMath
        CMSISDSPComplexMath
        CMSISDSPFastMath
        CMSISDSPStatistics
        CMSISDSPSupport
        CMSISDSPTransform
        CMSISDSPCommon
)

foreach(CMSIS_DSP_TARGET IN LISTS CMSIS_DSP_OPTIMIZED_TARGETS)
    if(TARGET "${CMSIS_DSP_TARGET}")
        target_compile_options(
                "${CMSIS_DSP_TARGET}"
                PRIVATE
                $<$<COMPILE_LANGUAGE:C>:-O3>
                $<$<COMPILE_LANGUAGE:C>:-ffast-math>
        )
    endif()
endforeach()

message(STATUS "CMSIS-DSP root: ${CMSIS_DSP_ROOT}")
message(STATUS "CMSIS-Core root: ${CMSISCORE}")
message(STATUS "CMSIS-DSP enabled for STM32H743 signal processing")
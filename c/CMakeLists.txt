cmake_minimum_required(VERSION 2.8.7)
project(libixcom)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake" "${CMAKE_CURRENT_LIST_DIR}/cmake/common")

include(GNUInstallDirs)
include(CCache)
include(SwiftCmakeOptions)
include(TestTargets)
include(LanguageStandards)
include(CompileOptions)

set(disable_tests FALSE)
if(MSVC)
  message(STATUS "MSVC detected, skipping unit tests")
  set(disable_tests TRUE)
endif()

swift_create_project_options(
        HAS_TESTS
        DISABLE_TEST_COMPONENTS ${disable_tests}
        TEST_PACKAGES "Check"
)

find_package(Swiftnav REQUIRED)

include(ClangFormat)
swift_setup_clang_format()

include(ClangTidy)
swift_setup_clang_tidy(PATTERNS "src/*.c")

add_subdirectory (src)

if(libixcom_BUILD_TESTS)
  add_subdirectory (test)
endif()


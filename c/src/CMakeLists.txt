cmake_minimum_required(VERSION 2.8.7)

set(libixcom_HEADERS
  )

add_library(ixcom
  )

target_link_libraries(ixcom m)
target_include_directories(ixcom PUBLIC ${PROJECT_SOURCE_DIR}/include)

target_compile_options(ixcom PRIVATE "-Wall")
target_compile_options(ixcom PRIVATE "-Wextra")
target_compile_options(ixcom PRIVATE "-Werror")
target_compile_options(ixcom PRIVATE "-Wimplicit")
target_compile_options(ixcom PRIVATE "-Wshadow")
target_compile_options(ixcom PRIVATE "-Wswitch-default")
target_compile_options(ixcom PRIVATE "-Wswitch-enum")
target_compile_options(ixcom PRIVATE "-Wundef")
target_compile_options(ixcom PRIVATE "-Wuninitialized")
target_compile_options(ixcom PRIVATE "-Wpointer-arith")
target_compile_options(ixcom PRIVATE "-Wcast-align")
target_compile_options(ixcom PRIVATE "-Wformat=2")
target_compile_options(ixcom PRIVATE "-Wimplicit-function-declaration")
target_compile_options(ixcom PRIVATE "-Wredundant-decls")
target_compile_options(ixcom PRIVATE "-Wformat-security")
target_compile_options(ixcom PRIVATE "-fno-unwind-tables")
target_compile_options(ixcom PRIVATE "-fno-asynchronous-unwind-tables")
target_compile_options(ixcom PRIVATE "-std=gnu99")
target_compile_options(ixcom PRIVATE "-fPIC")
# require at least gcc 5.0
if (NOT "${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU" OR CMAKE_C_COMPILER_VERSION VERSION_GREATER 5.0)
   target_compile_options(ixcom PRIVATE "-Wfloat-conversion")
endif()

install(TARGETS ixcom DESTINATION ${CMAKE_INSTALL_FULL_LIBDIR})
install(FILES ${libixcom_HEADERS} DESTINATION ${CMAKE_INSTALL_FULL_INCLUDEDIR}/ixcom)


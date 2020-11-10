cmake_minimum_required(VERSION 2.8.7)

set(libixcom_HEADERS
  ${PROJECT_SOURCE_DIR}/include/ixcom/messages.h
  ${PROJECT_SOURCE_DIR}/include/ixcom/XCOMdat.h
  ${PROJECT_SOURCE_DIR}/include/ixcom/decode.h
  ${PROJECT_SOURCE_DIR}/include/ixcom/encode.h
  )

add_library(ixcom
  decode.c
  encode.c
  messages.c
  )

target_link_libraries(ixcom m swiftnav)
target_include_directories(ixcom PUBLIC ${PROJECT_SOURCE_DIR}/include)
swift_set_language_standards(ixcom)
swift_set_compile_options(ixcom ADD -pedantic)


install(TARGETS ixcom DESTINATION ${CMAKE_INSTALL_FULL_LIBDIR})
install(FILES ${libixcom_HEADERS} DESTINATION ${CMAKE_INSTALL_FULL_INCLUDEDIR}/ixcom)


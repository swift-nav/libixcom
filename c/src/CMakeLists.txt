cmake_minimum_required(VERSION 2.8.7)

set(libixcom_HEADERS
  ${PROJECT_SOURCE_DIR}/include/ixcom/messages.h
  ${PROJECT_SOURCE_DIR}/include/ixcom/XCOMdat.h
  ${PROJECT_SOURCE_DIR}/include/ixcom/decode.h
  ${PROJECT_SOURCE_DIR}/include/ixcom/encode.h
  )

swift_add_library(ixcom
  SOURCES
    decode.c
    encode.c
    messages.c
)
add_library(swiftnav::ixcom ALIAS ixcom)

target_link_libraries(ixcom PUBLIC swiftnav)
target_include_directories(ixcom PUBLIC ${PROJECT_SOURCE_DIR}/include)

install(TARGETS ixcom DESTINATION ${CMAKE_INSTALL_FULL_LIBDIR})
install(FILES ${libixcom_HEADERS} DESTINATION ${CMAKE_INSTALL_FULL_INCLUDEDIR}/ixcom)


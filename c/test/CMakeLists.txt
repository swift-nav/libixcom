cmake_minimum_required(VERSION 2.8.7)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/")

find_package(Threads)

set(TEST_SOURCE_FILES
    check_main.c
    check_ixcom.c
    )

swift_add_test(test_libixcom
        POST_BUILD
        SRCS ${TEST_SOURCE_FILES}
        LINK ixcom check Threads::Threads
        )

target_compile_options(test_libixcom PRIVATE "-std=gnu99")

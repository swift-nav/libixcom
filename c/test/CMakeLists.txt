cmake_minimum_required(VERSION 2.8.7)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/")

find_package(Threads)

set(TEST_SOURCE_FILES
    check_main.c
    check_ixcom.c
    )

swift_add_test(test_libixcom
        UNIT_TEST
        POST_BUILD
        SRCS ${TEST_SOURCE_FILES}
        LINK ixcom check Threads::Threads
        )
swift_set_language_standards(test_libixcom)
swift_set_compile_options(test_libixcom)

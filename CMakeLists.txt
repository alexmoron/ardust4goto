########### Tutorial one ##############
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

set(BIN_INSTALL_DIR "/usr/bin")
include_directories("/usr/include/libindi/")
add_executable(arduino_st4go arduino_st4go.cpp ardust4driver.cpp)

target_link_libraries(arduino_st4go indidriver "/usr/lib/i386-linux-gnu/libnova-0.14.so.0")

install(TARGETS arduino_st4go RUNTIME DESTINATION bin )
cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(RoastDegreeAnalyzer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-DUNICODE)
add_definitions(-D_UNICODE)

add_executable(RoastDegreeAnalyzer Server.cpp)

if(UNIX)
	set(THREADS_PREFER_PTHREAD_FLAG ON)
	find_package(Threads REQUIRED)
	target_link_libraries(RoastDegreeAnalyzer PRIVATE Threads::Threads)

	find_package(OpenSSL REQUIRED)
	include_directories(${OPENSSL_INCLUDE_DIR})
	target_link_libraries(RoastDegreeAnalyzer PRIVATE ${OPENSSL_CRYPTO_LIBRARY})
	target_link_libraries(RoastDegreeAnalyzer PRIVATE ${OPENSSL_SSL_LIBRARY})

	find_package(PkgConfig REQUIRED)

	pkg_check_modules(LIBCAMERA REQUIRED libcamera)
	message(STATUS "libcamera library found:")
	message(STATUS "    version: ${LIBCAMERA_VERSION}")
	message(STATUS "    libraries: ${LIBCAMERA_LINK_LIBRARIES}")
	message(STATUS "    include path: ${LIBCAMERA_INCLUDE_DIRS}")
	include_directories(${CMAKE_SOURCE_DIR} ${LIBCAMERA_INCLUDE_DIRS})
	target_link_libraries(RoastDegreeAnalyzer PRIVATE ${LIBCAMERA_LINK_LIBRARIES})
endif(UNIX)

cmake_minimum_required(VERSION 2.8)


set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)


set(CMAKE_SYSROOT  /home/tgjuranec/download_source/rpiSysRoot)
set(tooldir /opt/cross-pi-gcc/bin)
set(SECOND_INCLUDE_SYSROOT /home/tgjuranec/download_source/rpiSysRoot)

######SET COMPILERS - BEFORE project()!!!!!!
set(CMAKE_C_COMPILER ${tooldir}/arm-linux-gnueabihf-gcc)
set(CMAKE_CXX_COMPILER ${tooldir}/arm-linux-gnueabihf-g++)
set(CMAKE_AR                    ${tooldir}/arm-linux-gnueabihf-ar)
set(CMAKE_ASM_COMPILER          ${tooldir}/arm-linux-gnueabihf-as)
set(CMAKE_LINKER                ${tooldir}/arm-linux-gnueabihf-gcc)
set(CMAKE_OBJCOPY               ${tooldir}/arm-linux-gnueabihf-objcopy)
set(CMAKE_RANLIB                ${tooldir}/arm-linux-gnueabihf-ranlib)
set(CMAKE_SIZE                  ${tooldir}/arm-linux-gnueabihf-size)
set(CMAKE_STRIP                 ${tooldir}/arm-linux-gnueabihf-strip)

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR arm)

#BUILD TYPE RELWITHDEBINFO | MINSIZEREL | DEBUG | RELEASE
set(CMAKE_BUILD_TYPE ${CMAKE_C_FLAGS_DEBUG})

set(CMAKE_C_FLAGS "-ggdb -march=armv6 -mfloat-abi=hard -mfpu=vfp")
set(CMAKE_CXX_FLAGS ${CMAKE_C_FLAGS})
#https://newbedev.com/force-gcc-to-notify-about-undefined-references-in-shared-libraries
#https://stackoverflow.com/questions/44617703/how-to-resolve-undefined-reference-when-they-are-not-used-with-gcc-cmake
#https://stackoverflow.com/questions/11783932/how-do-i-add-a-linker-or-compile-flag-in-a-cmake-file
set(CMAKE_EXE_LINKER_FLAGS  "-Wl,--allow-shlib-undefined")


project(scrcpy C CXX ASM)

set(SCRCPY_SOURCES 
	adb.c
	cli.c
	clock.c
	compat.c
	controller.c
	control_msg.c
	decoder.c
	device_msg.c
	event_converter.c
	file_handler.c
	fps_counter.c
	frame_buffer.c
	input_manager.c
	main.c
	opengl.c
	receiver.c
	recorder.c
	scrcpy.c
	screen.c
	server.c
	stream.c
	tiny_xpm.c
	v4l2_sink.c
	video_buffer.c
	sys/unix/process.c
	util/log.c
	util/net.c
	util/process.c
	util/str_util.c
	util/thread.c
	util/tick.c )

link_directories(${CMAKE_SYSROOT}/usr/lib/arm-linux-gnueabihf)
link_directories(${CMAKE_SYSROOT}/usr/lib)
link_directories(${CMAKE_SYSROOT}/lib/arm-linux-gnueabihf)
link_directories(${CMAKE_SYSROOT}/lib)

include_directories(${SECOND_INCLUDE_SYSROOT}/usr/include)
include_directories(${SECOND_INCLUDE_SYSROOT}/usr/include/arm-linux-gnueabihf 
android 
sys/unix 
trait 
util)

set(AVFORMAT_LIBRARY avformat)
set(AVCODEC_LIBRARY avcodec)
set(AVDEVICE_LIBRARY avdevice)
set(AVUTIL_LIBRARY avutil)
set(SDL2_LIBRARY SDL2)
set(AVFILTER_LIBRARY avfilter)

get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
foreach(dir ${dirs})
  message(STATUS "include dir ='${dir}'")
endforeach()

get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY LINK_DIRECTORIES)
foreach(dir ${dirs})
  message(STATUS "link dir ='${dir}'")
endforeach()
message(STATUS ${CMAKE_EXE_LINKER_FLAGS})

add_executable(scrcpy ${SCRCPY_SOURCES})

target_include_directories(scrcpy PUBLIC INCLUDE_DIRECTORIES)

target_link_libraries(scrcpy PUBLIC 
	${AVFORMAT_LIBRARY} 
	${AVCODEC_LIBRARY}
	${AVDEVICE_LIBRARY} 
	${AVUTIL_LIBRARY}
	${SDL2_LIBRARY}
	${AVFILTER_LIBRARY}
	)


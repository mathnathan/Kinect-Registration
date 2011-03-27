CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

project(KinReg C CXX)

set(Freenect_DIR ${KinReg_SOURCE_DIR}/include/libfreenect)

set(DEPENDENCIES OpenCV GLUT OpenGL)

message("\n")
foreach(DEP ${DEPENDENCIES})
	message("Looking for ${DEP}...")
	find_package(${DEP} REQUIRED)
	if( ${DEP} STREQUAL OpenGL )
		set( DEP OPENGL )
	endif()
		if( ${DEP}_FOUND )
			message("${DEP}_Found = TRUE\n")
		else()
			message("${DEP}_Found = FALSE\n")
		endif()
endforeach()

# Build the libfreenect libraries
message("---- Preparing to Build Dependencies ---- \n")

	# ------------------- TO DO ---------------------
	# Eventually package OpenCV in here as well...

	if(EXISTS ${CMAKE_BINARY_DIR}/lib/libfreenect.so AND ${LIBFREENECT_BUILT})	
		message("\nLIBFREENECT_ALREADY_BUILT = " ${LIBFREENECT_BUILT})
		message("\n")
		set(Freenect_LIBS 
			
			${CMAKE_BINARY_DIR}/lib/libfreenect.so
			${CMAKE_BINARY_DIR}/lib/libfreenect_cv.so
			${CMAKE_BINARY_DIR}/lib/libfreenect_sync.so 
			)
	else()
		add_subdirectory(${Freenect_DIR})
		set(Freenect_LIBS 

			freenect 
			freenect_cv 
			freenect_sync 
			)
	endif()

set( Freenect_INCLUDE_DIRS

    ${Freenect_DIR}/include
    ${Freenect_DIR}/wrappers/c_sync
    ${Freenect_DIR}/wrappers/opencv
	)

include_directories( 

	${Freenect_INCLUDE_DIRS}
	${GLUT_INCLUDE_DIR}
	${OPENGL_INCLUDE_DIR}
	${LIBUSB_1_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
	${freenect_INCLUDE_DIRS}
	)


# ------------- TO DO ---------------
# Chop out some of the code from kinReg.cpp 
# and put it into libraries.
# set( KinReg_SOURCES headers... libraries... )
# add_library(kinreg SHARED ${KinReg_SOURCES})
#
# Then install the libraries and header files

add_executable(kinect_reg kinReg.cpp) 

target_link_libraries(kinect_reg 

	${GLUT_LIBRARY} 
	${OPENGL_LIBRARIES} 
	${OpenCV_LIBS}
	${Freenect_LIBS} 
	)


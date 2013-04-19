CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

project(KinReg C CXX)

set(DEPENDENCIES OpenCV GLUT OpenGL Freenect)

message("\n")
foreach( DEP ${DEPENDENCIES} )
    
    find_package( ${DEP} )

    string( TOUPPER ${DEP} UDEP ) # Capitalize
    if( ${DEP}_FOUND OR ${UDEP}_FOUND )
        message("\n${DEP}_Found = TRUE\n")
    else()
        message("Find${DEP}.cmake not found... Looking for ${DEP} 
        ourselves.\n")
        include( "cmake/find/Find${DEP}.cmake" ) # Try our Find files
        if( ${DEP}_FOUND OR ${UDEP}_FOUND )
            message("\n${DEP}_Found = TRUE\n")
        else()
            message("\n${DEP}_Found = FALSE\n")
        endif()
    endif() 

endforeach()


include_directories( 

    ${FREENECT_INCLUDE_DIR}
	${GLUT_INCLUDE_DIR}
	${OPENGL_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(kinect_reg kinReg.cpp) 

target_link_libraries(kinect_reg 

    ${FREENECT_LIBRARIES} 
	${GLUT_LIBRARY} 
	${OPENGL_LIBRARIES} 
	${OpenCV_LIBS}
)

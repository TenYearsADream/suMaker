cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# add our macro directory to cmake search path
set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake)

##使用libgl和nanoGUI作为编辑界面
set(APP_NAME suMaker)
project(${APP_NAME})

######################External Lib Setting Begin#####################
###保留libgl的设置(from libgl tutorial)
### libIGL options: choose between header only and compiled static library
option(LIBIGL_USE_STATIC_LIBRARY "Use LibIGL as static library" OFF)
option(LIBIGL_WITH_VIEWER      "Use OpenGL viewer"  ON)
option(LIBIGL_WITH_NANOGUI     "Use Nanogui menu"   ON)

### libIGL options: choose your dependencies (by default everything is OFF, in this example we need the viewer) ###
option(LIBIGL_WITH_BBW              "Use BBW"            ON)
find_package(CGAL QUIET)
option(LIBIGL_WITH_CGAL             "Use CGAL"           "${CGAL_FOUND}")
option(LIBIGL_WITH_COMISO           "Use CoMiso"         ON)
option(LIBIGL_WITH_CORK             "Use CORK"           OFF)
option(LIBIGL_WITH_EMBREE           "Use Embree"         ON)
option(LIBIGL_WITH_LIM              "Use LIM"            ON)
find_package(MATLAB QUIET)
option(LIBIGL_WITH_MATLAB           "Use Matlab"         "${MATLAB_FOUND}")
option(LIBIGL_WITH_MOSEK            "Use MOSEK"          "${MOSEK_FOUND}")
option(LIBIGL_WITH_OPENGL           "Use OpenGL"         ON)
option(LIBIGL_WITH_PNG              "Use PNG"            ON)
option(LIBIGL_WITH_TETGEN           "Use Tetgen"         ON)
option(LIBIGL_WITH_TRIANGLE         "Use Triangle"       ON)
option(LIBIGL_WITH_XML              "Use XML"            ON)
### End   to be tested ----

### libIGL options: decide if you want to use the functionalities that depends on cgal
if(LIBIGL_WITH_CGAL) # Do not remove or move this block, cgal strange build system fails without it
  find_package(CGAL REQUIRED)
  set(CGAL_DONT_OVERRIDE_CMAKE_FLAGS TRUE CACHE BOOL "CGAL's CMAKE Setup is super annoying ")
  include(${CGAL_USE_FILE})
endif()


### Adding libIGL: choose the path to your local copy libIGL ###
### This is going to compile everything you requested ###
add_subdirectory("${PROJECT_SOURCE_DIR}/external/libigl/shared/cmake" "libigl")


### Output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

### Compilation flags: adapt to your needs ###
if(MSVC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP /bigobj") ### Enable parallel compilation
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR} )
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR} )
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /w") # disable all warnings (not ideal but...)
  
  list(APPEND LIBIGL_DEFINITIONS "-DEIGEN_DONT_ALIGN")     #remove eigen error
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11") #### Libigl requires a modern C++ compiler that supports c++11
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "../" )
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w") # disable all warnings (not ideal but...)
endif()

# Enable openMP if possible
#find_package(OpenMP)
#if (OPENMP_FOUND AND NOT WIN32)
#  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
#  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
#endif()


### Prepare the build environment

include_directories(${LIBIGL_INCLUDE_DIRS})
##add_definitions(${LIBIGL_DEFINITIONS})


## OpenMesh
find_package (OpenMesh)
if (NOT OPENMESH_FOUND)
    message (FATAL_ERROR "OpenMesh not found!")
endif ()

## OpenCV
FIND_PACKAGE(OpenCV REQUIRED)

#######################External Lib Setting End###############

#######################Use bin2c converting resource##########
# Run bin2c on resource files
set(PNG_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/resources/comb.png
  ${CMAKE_CURRENT_SOURCE_DIR}/resources/delete_stroke.png
  ${CMAKE_CURRENT_SOURCE_DIR}/resources/loadmesh.png
  ${CMAKE_CURRENT_SOURCE_DIR}/resources/scare.png
  ${CMAKE_CURRENT_SOURCE_DIR}/resources/sing_dir.png
  ${CMAKE_CURRENT_SOURCE_DIR}/resources/sing_pos.png
  ${CMAKE_CURRENT_SOURCE_DIR}/resources/instantmeshes.png)
file(GLOB FRAG_FILES "${CMAKE_CURRENT_SOURCE_DIR}/resources/*.frag")
file(GLOB VERT_FILES "${CMAKE_CURRENT_SOURCE_DIR}/resources/*.vert")
file(GLOB GEO_FILES  "${CMAKE_CURRENT_SOURCE_DIR}/resources/*.geo")
set(IM_RESOURCES ${PNG_FILES} ${FRAG_FILES} ${VERT_FILES} ${GEO_FILES})

add_custom_command(
  OUTPUT resources.cpp
  COMMAND bin2c ARGS resources.cpp resources.h ${IM_RESOURCES}
  DEPENDS bin2c ${IM_RESOURCES}
  COMMENT "Running bin2c"
  PRE_BUILD VERBATIM)

# Resource file (icons etc.)
set(EXTRA_SOURCE "")
if (APPLE)
  set(EXTRA_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/resources/im.icns")
elseif(WIN32)
  set(EXTRA_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/resources/im.rc")
endif()

#######################End resource###########################


#######################Project Setting########################
FILE(GLOB SRC_SOURCES	App/${APP_NAME}/*.cpp App/${APP_NAME}/*.c) 
FILE(GLOB SRC_HEADERS	App/${APP_NAME}/*.h App/${APP_NAME}/*.hpp)
set(SRC_LIST ${SRC_SOURCES} 
             ${SRC_HEADERS} 
			 resources.cpp
 ) 


set(PROJECT_INCLUDE_DIRS ${${APP_NAME}_SOURCE_DIR}/include)
set(PROJECT_LIBRARY_DIRS ${${APP_NAME}_SOURCE_DIR}/lib)

include_directories(
  ${OPENCV2_INCLUDE_PATH}
  ${OPENMESH_INCLUDE_DIRS}
  ${PROJECT_INCLUDE_DIRS}
  # Resource file
  ${CMAKE_CURRENT_BINARY_DIR}
)
  
link_directories(
  #${OPENCV2_LIB_PATH}  
  ${OPENMESH_LIBRARIES}
  ${PROJECT_LIBRARY_DIRS}  
)

add_executable (${APP_NAME}  
                ${SRC_LIST}
				${EXTRA_SOURCE}
				)

target_compile_definitions(${APP_NAME}  PRIVATE ${LIBIGL_DEFINITIONS} 
                                                ENABLE_IO    #Enable IO button
												)
target_link_libraries (${APP_NAME}                        
                       ${LIBIGL_LIBRARIES}
		       ${LIBIGL_EXTRA_LIBRARIES}     
		       ${OPENMESH_LIBRARIES}    
			   ${OpenCV_LIBS}
                      
)

message(STATUS "nanoGUI libs: "  ${NANOGUI_EXTRA_LIBS})



cmake_minimum_required( VERSION 3.0 )

set( ProjectName "canalboat" )

project( ${ProjectName} CXX )

set(CompilerFlags
        CMAKE_CXX_FLAGS
        CMAKE_CXX_FLAGS_DEBUG
        CMAKE_CXX_FLAGS_RELEASE
		CMAKE_CXX_FLAGS_MINSIZEREL
		CMAKE_CXX_FLAGS_RELWITHDEBINFO
        CMAKE_C_FLAGS
        CMAKE_C_FLAGS_DEBUG
        CMAKE_C_FLAGS_RELEASE
		CMAKE_C_FLAGS_MINSIZEREL
		CMAKE_C_FLAGS_RELWITHDEBINFO
        )
foreach(CompilerFlag ${CompilerFlags})
  string(REPLACE "/MD" "/MT" ${CompilerFlag} "${${CompilerFlag}}")
endforeach()

# Definitons
ADD_DEFINITIONS( -DUNICODE )
ADD_DEFINITIONS( -D_UNICODE )

# Include and Source Files
file( GLOB Project_INC "${PROJECT_SOURCE_DIR}/include/*.h" )

file( GLOB Project_SRC "${PROJECT_SOURCE_DIR}/src/*.cpp" )

include_directories( "${PROJECT_SOURCE_DIR}/include" )

# Boost Headers
set( BOOST_INCLUDE_DIR "NOTBOOST_INCLUDE_DIR-NOTFOUND" CACHE PATH "" )

if( EXISTS ${BOOST_INCLUDE_DIR} )
	include_directories( ${BOOST_INCLUDE_DIR} )
else()
	message( FATAL_ERROR "Invalid BOOST_INCLUDE_DIR." )
endif()

add_executable( ${ProjectName}  
	${Project_INC}
	${Project_SRC} )

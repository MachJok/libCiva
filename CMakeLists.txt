# Boilerplate cmake version check
cmake_minimum_required(VERSION 3.0)


# Define your project name here. You can build multiple projects from
# a single cmake file, or you can created nested project hierarchies,
# each in its own folder.
project(CIVA_IRU)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_definitions(-DDEBUG)
# set(name value...) is just setting a variable, nothing more.
# There's nothing magical about the 'SRC' and 'HDR' names, we're
# just using them to keep things organized.
set(SRC
	datarefs.cpp
	functions.cpp
	plugin.cpp
	structs.cpp
	dme_functions.cpp
	)
set(HDR
    datarefs.h
	functions.h
	GeographicLib.h
	sources.h
	structs.h
	variables.h
	dme_functions.h)
# To recall a variable use ${VARNAME}
set(ALL_SRC ${SRC} ${HDR})

# You can also set build-environment variables from the command line like so:
#	cmake -DFOO=BAR .
# this can then be referenced in the cmake file as ${FOO}

# This just gives you nice alphabetical build order, not necessary really
list(SORT ALL_SRC)

# This configures "my_test_project" as building a shared library, which is
# what X-Plane needs (DLL on Windows, dylib on Mac, .so on Linux) and
# associates all our source files with it. Notice that CMake doesn't mind
# if you list all sources and headers together in one big blob. It
# understands what is C and what is C++ (based on file extension and
# automatically picks the correct compiler & linker invocations.
add_library(CIVA_IRU SHARED ${ALL_SRC})

# Here we configure where the compilers will look for extra include files.
# Again, feel free to use the ${VARNAME} expansions in these strings if you
# need more environment-specific control over where stuff is (otherwise the
# paths are relative to where cmake is invoked).
include_directories(CIVA_IRU PUBLIC
    "XPLSDK303/CHeaders/XPLM"
    "XPLSDK303/CHeaders/Widgets"
    "XPLSDK303/CHeaders/Wrappers"
    "libacfutils/include"
    "libacfutils/lin64/include"
#change the following line to the local path for GeographicLib on your system
    "/usr/local/include/GeographicLib"
    )

# You can add language-specific options by manipulating CMAKE_C_FLAGS for
# the C language and CMAKE_CXX_FLAGS for the C++ language. To add common
# options for both, use add_compile_options("blah blah").
# Since compiler options tend to be quite compiler specific, you should
# probably enclose these in a conditional such as:
#	if (MSVC)
#		# MSVC: warning level 4 and all warnings as errors
#		add_compile_options(/W4 /WX)
#	else()
#		# GCC/Clang: lots of warnings and all warnings as errors
#		add_compile_options(-Wall -Wextra -pedantic -Werror)
#	endif()

# Compiler flag specification
# Always recommend maximum warnings and warnings-to-errors on,
# but of course, customize to your needs.
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -W -Wall -Wextra -Werror -pedantic --std=c11 \
    -Wno-unused-local-typedefs -Wno-missing-field-initializers")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=c++14" )

# A nifty little canned method that automatically converts between
# compiler define flags based on the target compiler:
add_definitions(-DXPLM200=1 -DXPLM210=1 -DXPLM300=1 -DXPLM301=1)
if(WIN32)
	add_definitions(-DAPL=0 -DIBM=1 -DLIN=0)
elseif(APPLE)
	add_definitions(-DAPL=1 -DIBM=0 -DLIN=0)
else()
	add_definitions(-DAPL=0 -DIBM=0 -DLIN=1)
endif()

# Linking

# This looks for a library named "foo" (foo.dll, foo.dylib, libfoo.so)
# in the system's standard library search paths. If found, "MY_LIB_NAME"
# will be defined how the linking commands to link that library into the
# resulting binary.
#find_library(LIBACFUTILS "libacfutils")


# You can also manually provide a search path
#find_library(BAR_LIBRARY "bar" "/search/for/it/here")
#find_package(GeographicLib REQUIRED)

# Alternatively, when using a static library, you can literally just use
# the exact library file as a linker input:
if(WIN32)
    set(ARCH "mingw64")
elseif(APPLE)
    set(ARCH "mac64")
else()
    set(ARCH "lin64")
endif()
# here ${LIBACFUTILS} points at the top-level directory
# of the libacfutils-redist package

set(LIBACFUTILS "${CMAKE_SOURCE_DIR}/libacfutils")
#set(GEOGRAPHIC_LIB "${CMAKE_SOURCE_DIR}/geographic_lib")
find_library(LIBACFUTILS_LIBS acfutils "${LIBACFUTILS}/${ARCH}/lib")
find_library(GEOGRAPHIC_LIB Geographic "${CMAKE_SOURCE_DIR}/geographic_lib")

#file(GLOB LIBACFUTILS_LIBS ${LIBACFUTILS}/${ARCH}/lib/*.a)
#file(GLOB GEOGRAPHIC_LIB ${CMAKE_SOURCE_DIR}/geographic_lib/*.a)

target_link_libraries(CIVA_IRU
    ${LIBACFUTILS_LIBS}
	${GEOGRAPHIC_LIB}      
)

# This avoids prepending or appending any `lib' or `-<version>' extensions
# to our build product. We want to control the output filename exactly.
set_target_properties(CIVA_IRU PROPERTIES PREFIX "")
set_target_properties(CIVA_IRU PROPERTIES SUFFIX "")
set_target_properties(CIVA_IRU PROPERTIES OUTPUT_NAME
    "CIVA_IRU.xpl")

# Where does our product go? Place it one level above our source directory
# into a directory name win_x64, mac_x64 or lin_x64, depending on the
# target host platform.
if(WIN32)
	set_target_properties(CIVA_IRU PROPERTIES
	    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/output/CIVA_IRU/win_x64" )
	set_target_properties(CIVA_IRU PROPERTIES
	    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/output/CIVA_IRU/win_x64" )
elseif(APPLE)
	set_target_properties(CIVA_IRU PROPERTIES
	    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/output/CIVA_IRU/mac_x64" )
	set_target_properties(CIVA_IRU PROPERTIES
	    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/output/CIVA_IRU/mac_x64" )
else()
	set_target_properties(CIVA_IRU PROPERTIES
	    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/output/CIVA_IRU/lin_x64" )
	set_target_properties(CIVA_IRU PROPERTIES
	    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/output/CIVA_IRU/lin_x64" )
endif()

# Any extra last-minute linker flags:
#set_target_properties(my_test_project PROPERTIES LINK_FLAGS
#    "${CMAKE_SHARED_LINKER_FLAGS} -rdynamic -nodefaultlibs \
#    -undefined_warning -fPIC -fvisibility=hidden")

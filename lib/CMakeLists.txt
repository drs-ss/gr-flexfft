# Copyright 2011,2012 Free Software Foundation, Inc.
#
# This file is part of GNU Radio
#
# GNU Radio is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
#
# GNU Radio is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with GNU Radio; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.

########################################################################
# Setup library
########################################################################
include(GrPlatform) #define LIB_SUFFIX
include(GrMiscUtils)
include(GrPython)

include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})

########################################################################
# Set QT generation stuff
########################################################################
set(display_moc_hdrs
    ../include/flexfft/displayform.h
    ../include/flexfft/timedisplayform.h
    ../include/flexfft/timecontrolpanel.h
    ../include/flexfft/form_menus.h
    ../include/flexfft/DisplayPlot.h
    ../include/flexfft/TimeDomainDisplayPlot.h
)

QT4_WRAP_CPP(display_moc_srcs ${display_moc_hdrs})
QT4_WRAP_UI(display_ui_hdrs)

########################################################################

list(APPEND flexfft_sources
    ${display_moc_srcs}
    ${display_ui_hdrs}
    flex_fft_parser.cpp
    displayform.cc
    spectrumUpdateEvents.cc
    timecontrolpanel.cc
    timedisplayform.cc
    DisplayPlot.cc
    TimeDomainDisplayPlot.cc
    flex_fft_sink_impl.cc
)

set(flexfft_sources "${flexfft_sources}" PARENT_SCOPE)
if(NOT flexfft_sources)
	MESSAGE(STATUS "No C++ sources... skipping lib/")
	return()
endif(NOT flexfft_sources)

add_definitions(-DQWT_DLL) #setup QWT library linkage
add_library(gnuradio-flexfft SHARED ${flexfft_sources})
target_link_libraries(gnuradio-flexfft ${Boost_LIBRARIES} ${GNURADIO_ALL_LIBRARIES} ${QWT_LIBRARIES} ${QT_LIBRARIES} ${PYTHON_LIBRARIES})
set_target_properties(gnuradio-flexfft PROPERTIES DEFINE_SYMBOL "gnuradio_flexfft_EXPORTS")

if(APPLE)
    set_target_properties(gnuradio-flexfft PROPERTIES
        INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib"
    )
endif(APPLE)

########################################################################
# Install built library files
########################################################################
install(TARGETS gnuradio-flexfft
    LIBRARY DESTINATION lib${LIB_SUFFIX} # .so/.dylib file
    ARCHIVE DESTINATION lib${LIB_SUFFIX} # .lib file
    RUNTIME DESTINATION bin              # .dll file
)

########################################################################
# Build and register unit test
########################################################################
include(GrTest)

include_directories(${CPPUNIT_INCLUDE_DIRS})

list(APPEND test_flexfft_sources
    ${CMAKE_CURRENT_SOURCE_DIR}/test_flexfft.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/qa_flexfft.cc
)

add_executable(test-flexfft ${test_flexfft_sources})

target_link_libraries(
  test-flexfft
  ${GNURADIO_RUNTIME_LIBRARIES}
  ${Boost_LIBRARIES}
  ${CPPUNIT_LIBRARIES}
  gnuradio-flexfft
)

GR_ADD_TEST(test_flexfft test-flexfft)

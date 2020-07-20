# FindWiringpi.cmake
# ----------- usage ----------
# Locate libraries and headers
#find_package(WiringPi REQUIRED)
#
# Include headers
#include_directories(${WIRINGPI_INCLUDE_DIRS})
#
# Link against libraries
#target_link_libraries(<yourProjectName> ${WIRINGPI_LIBRARIES} -lrt)


FIND_PATH(WIRINGPI_INCLUDE_DIR NAMES wiringPi.h)

FIND_LIBRARY(WIRINGPI_LIBRARIES NAMES wiringPi)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(wiringPi DEFAULT_MSG WIRINGPI_LIBRARIES WIRINGPI_INCLUDE_DIRS)
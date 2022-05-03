find_package(OpenMP REQUIRED)
include_directories(${OpenMP_CXX_INCLUDE_DIRS} )
list(APPEND ALL_TARGET_LIBRARIES ${OpenMP_CXX_LIBRARIES} )
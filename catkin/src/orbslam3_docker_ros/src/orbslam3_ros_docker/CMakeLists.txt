target_sources(
  mono_imu__containerized PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/mono_imu.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/config.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/config.hpp
)
add_subdirectory(callback)
add_subdirectory(publish)

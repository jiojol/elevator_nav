# Some ROS Noetic installations still export absolute Boost 1.71 paths after
# the system has upgraded Boost.  Replace only missing Boost paths with the
# installed library; leave every valid catkin link entry untouched.
set(_dog_nav_catkin_libraries "")
foreach(_dog_nav_library IN LISTS catkin_LIBRARIES)
  if(IS_ABSOLUTE "${_dog_nav_library}" AND NOT EXISTS "${_dog_nav_library}"
      AND "${_dog_nav_library}" MATCHES "/libboost_([^/]+)\\.so(\\.[0-9.]+)?$")
    set(_dog_nav_boost_name "boost_${CMAKE_MATCH_1}")
    unset(_dog_nav_boost_replacement CACHE)
    find_library(_dog_nav_boost_replacement NAMES "${_dog_nav_boost_name}")
    if(NOT _dog_nav_boost_replacement)
      message(FATAL_ERROR "Missing ${_dog_nav_library}; install ${_dog_nav_boost_name}")
    endif()
    message(STATUS "Replacing stale ROS link path ${_dog_nav_library} with ${_dog_nav_boost_replacement}")
    list(APPEND _dog_nav_catkin_libraries "${_dog_nav_boost_replacement}")
  else()
    list(APPEND _dog_nav_catkin_libraries "${_dog_nav_library}")
  endif()
endforeach()
set(catkin_LIBRARIES "${_dog_nav_catkin_libraries}")

cmake_minimum_required(VERSION 3.8)
project(search_bringup)

find_package(ament_cmake REQUIRED)
find_package(rmf_demos_maps REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

# rmf_demos worlds to prepare for Clearpath robots (see scripts/prepare_world.py).
set(SEARCH_WORLDS hotel office clinic)

set(rmf_maps_share "${rmf_demos_maps_DIR}/..")
set(prepared_worlds)
foreach(world ${SEARCH_WORLDS})
  set(source_world "${rmf_maps_share}/maps/${world}/${world}.world")
  set(output_world "${CMAKE_CURRENT_BINARY_DIR}/maps/${world}/${world}.world")
  set(world_config)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config/worlds/${world}.yaml")
    set(world_config "${CMAKE_CURRENT_SOURCE_DIR}/config/worlds/${world}.yaml")
  endif()
  add_custom_command(
    OUTPUT ${output_world}
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/prepare_world.py
      ${source_world} ${output_world} ${world_config}
    DEPENDS ${source_world} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/prepare_world.py ${world_config}
    COMMENT "Preparing rmf_demos ${world} world for Clearpath sensors"
  )
  list(APPEND prepared_worlds ${output_world})
endforeach()
add_custom_target(prepare_worlds ALL DEPENDS ${prepared_worlds})

install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/maps/ DESTINATION share/${PROJECT_NAME}/maps)
install(DIRECTORY config launch robots rviz DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/prepare_world.py DESTINATION lib/${PROJECT_NAME})

ament_package()

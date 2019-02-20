if (USE_QPOASES_IN_SEQUENCER)
  add_subdirectory(${CMAKE_CURRENT_BINARY_DIR}/../../3rdparty/qpOASES/qpOASES-3.0 ${CMAKE_CURRENT_BINARY_DIR}/../../3rdparty/qpOASES/qpOASES-3.0)
  link_directories(${CMAKE_CURRENT_BINARY_DIR}/../../3rdparty/qpOASES/qpOASES-3.0/bin)
  include_directories(${CMAKE_CURRENT_BINARY_DIR}/../../3rdparty/qpOASES/qpOASES-3.0/include)
  add_definitions(-DUSE_QPOASES_IN_SEQUENCER)
endif()
set(comp_sources bspline.cpp interpolator.cpp timeUtil.cpp seqplay.cpp SequencePlayer.cpp SequencePlayerService_impl.cpp qpOASESWrapper.cpp ../ImpedanceController/JointPathEx.cpp)
if (USE_QPOASES_IN_SEQUENCER)
  set(libs hrpModel-3.1 hrpCollision-3.1 hrpUtil-3.1 hrpsysBaseStub ${Boost_THREAD_LIBRARY} boost_system qpOASES)
else()
  set(libs hrpModel-3.1 hrpCollision-3.1 hrpUtil-3.1 hrpsysBaseStub ${Boost_THREAD_LIBRARY} boost_system)
endif()

add_library(SequencePlayer SHARED ${comp_sources})
target_link_libraries(SequencePlayer ${libs})
set_target_properties(SequencePlayer PROPERTIES PREFIX "")

add_executable(SequencePlayerComp SequencePlayerComp.cpp ${comp_sources})
target_link_libraries(SequencePlayerComp ${libs})

set(target SequencePlayer SequencePlayerComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)


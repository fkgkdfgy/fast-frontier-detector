SET (maps_SRCS
  gridtraversal.cpp
  mapconfig.cpp
)


ADD_LIBRARY( maps ${maps_SRCS})

TARGET_LINK_LIBRARIES(
  maps
)

# ADD_EXECUTABLE(test test.cpp)
# TARGET_LINK_LIBRARIES(test octomap)




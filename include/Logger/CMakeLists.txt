add_library(logger
            Logger.h
            Logger.cpp
            )

set_target_properties(logger PROPERTIES LINKER_LANGUAGE CXX)

target_include_directories(logger PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

#target_link_libraries(logger
#                      )

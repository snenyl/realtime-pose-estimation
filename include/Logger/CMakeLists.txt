add_library(logger
            Logger/Logger.h
            Logger/Logger.cc
            )

set_target_properties(logger PROPERTIES LINKER_LANGUAGE CXX)

target_include_directories(logger PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

#target_link_libraries(logger
#                      )

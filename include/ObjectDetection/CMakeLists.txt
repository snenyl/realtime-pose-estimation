add_library(object_detection
            ObjectDetection.h
            ObjectDetection.cc
            )

set_target_properties(object_detection PROPERTIES LINKER_LANGUAGE CXX)

target_include_directories(object_detection PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

target_link_libraries(object_detection
                      ${InferenceEngine_LIBRARIES}
                      ${NGRAPH_LIBRARIES}
                      ${OpenCV_LIBS}
                      )

#target_link_libraries(object_detection ${OpenCV_LIBS})
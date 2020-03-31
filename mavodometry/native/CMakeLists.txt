
file(GLOB detect_c_wrapperSources *.c *.cpp)
file(GLOB detect_c_wrapperIncludes *.h )

cuda_add_executable(detect_c_wrapper ${detect_c_wrapperSources})
cuda_add_library(jetsonNano SHARED ${detect_c_wrapperSources} )

target_link_libraries(detect_c_wrapper jetson-inference)
target_link_libraries(jetsonNano jetson-inference)

install(TARGETS detect_c_wrapper DESTINATION bin)
install(TARGETS jetsonNano DESTINATION lib)
install(FILES detect_c_wrapper.h DESTINATION include)

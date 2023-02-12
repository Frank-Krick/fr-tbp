add_library(fr_tbp
  ../include/tbp/tbp.h
  tbp/tbp.cpp)

target_include_directories(fr_tbp PUBLIC ../include)

target_link_libraries(fr_tbp PUBLIC Boost::boost)

target_compile_options(fr_tbp PUBLIC /std:c++latest)

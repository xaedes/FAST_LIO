cmake_minimum_required(VERSION 3.8)

# set(CMAKE_CXX_EXTENSIONS OFF)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_14)
target_compile_options(${PROJECT_NAME} PRIVATE -fexceptions)
# target_compile_options(${PROJECT_NAME} PRIVATE -O3)
target_compile_definitions(${PROJECT_NAME} PRIVATE -DFAST_LIO_ROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

# fix "number of sections exceeded object file format limit"
if (MSVC)
  target_compile_options(${PROJECT_NAME} PRIVATE /bigobj)
else ()
  # -Wa,option: Pass option as an option to the assembler. If option contains commas, it is split into multiple options at the commas.
  target_compile_options(${PROJECT_NAME} PRIVATE -Wa,-mbig-obj)
endif ()

message("Current CPU architecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processer number:  ${N}")
  if(N GREATER 4)
    target_compile_definitions(${PROJECT_NAME} PRIVATE -DMP_EN)
    target_compile_definitions(${PROJECT_NAME} PRIVATE -DMP_PROC_NUM=3)
    message("core for MP: 3")
  elseif(N GREATER 3)
    target_compile_definitions(${PROJECT_NAME} PRIVATE -DMP_EN)
    target_compile_definitions(${PROJECT_NAME} PRIVATE -DMP_PROC_NUM=2)
    message("core for MP: 2")
  else()
    target_compile_definitions(${PROJECT_NAME} PRIVATE -DMP_PROC_NUM=1)
  endif()
else()
  target_compile_definitions(${PROJECT_NAME} PRIVATE -DMP_PROC_NUM=1)
endif()



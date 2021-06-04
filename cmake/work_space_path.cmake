set(WORK_SPACE_PATH ${PROJECT_SOURCE_DIR})
configure_file (
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/models/utils/work_space_path.h.in
  ${PROJECT_BINARY_DIR}/include/${PROJECT_NAME}/models/utils/work_space_path.h)
include_directories(${PROJECT_BINARY_DIR}/include)

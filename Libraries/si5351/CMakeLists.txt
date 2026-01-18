cmake_minimum_required(VERSION 3.20)

add_library(si5351 STATIC
  Src/si5351.c
)

target_include_directories(si5351 PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}/Inc
)
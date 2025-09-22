cmake_minimum_required(VERSION 3.20)
project(TransferGame CXX)

set(CMAKE_CXX_STANDARD 17)

# Collect source files
file(GLOB SRC_FILES
    "${CMAKE_SOURCE_DIR}/Transfer/*.cpp"
    "${CMAKE_SOURCE_DIR}/Transfer/*.h"
)

# Create the executable
add_executable(${PROJECT_NAME} ${SRC_FILES})

# Find SDL3
find_package(SDL3 REQUIRED CONFIG)

# Link SDL3
target_link_libraries(${PROJECT_NAME} PRIVATE SDL3::SDL3)


# Platform-specific post-build steps
# Copy SDL3.dll next to the exe after build (Windows only)
if (WIN32)
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${SDL3_DIR}/../lib/x64/SDL3.dll"
            $<TARGET_FILE_DIR:${PROJECT_NAME}>
    )
elseif(APPLE)
    # On macOS, copy SDL3 dylib or framework if needed
    find_library(SDL3_LIBRARY SDL3)
    if(SDL3_LIBRARY)
        add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${SDL3_LIBRARY}"
                $<TARGET_FILE_DIR:${PROJECT_NAME}>
        )
    endif()
    # Optionally, set up .app bundle properties here
endif()

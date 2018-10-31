###########################################
# Generate VS Code Environment
###########################################

message(STATUS "Instrumenting for Visual Studio Code")

if(CMAKE_GENERATOR MATCHES "(MSYS|Unix) Makefiles")
    find_program (GDB_EXECUTABLE gdb HINTS ENV MINGW_PREFIX MSYS2_PATH)
    find_program(BASH_EXECUTABLE bash HINTS ENV MINGW_PREFIX MSYS2_PATH)
    find_program(GLOBAL_EXECUTABLE NAMES global global.exe)
    find_program(CLANG_EXECUTABLE NAMES clang clang.exe)
    find_program(CLANG_TIDY_EXECUTABLE NAMES clang-tidy clang-tidy.exe)

    set(DEBUG_PATH "${CMAKE_BINARY_DIR}")
    if(CMAKE_GENERATOR MATCHES "MSYS Makefiles")
        set(DEBUG_PATH "${DEBUG_PATH};$ENV{MINGW_PREFIX}/bin")
    else()
        set(DEBUG_PATH "$ENV{PATH}:${DEBUG_PATH}")
    endif()

    set(DEBUG_CONFIGURATIONS "")
    get_directory_property(TARGETS DIRECTORY "${CMAKE_SOURCE_DIR}" BUILDSYSTEM_TARGETS)
    foreach(TARGET ${TARGETS})
        if(${TARGET} MATCHES "demo_")
            message(STATUS "Generating debug configuration for ${TARGET}")
            set(DEBUG_CONFIGURATIONS "${DEBUG_CONFIGURATIONS}
            {
            \"name\": \"${TARGET}\",
            \"type\": \"cppdbg\",
            \"request\": \"launch\",
            \"args\": [],
            \"stopAtEntry\": false,
            \"cwd\": \"${CMAKE_BINARY_DIR}\",
            \"environment\": [
                {
                    \"name\":\"PATH\",
                    \"value\":\"${DEBUG_PATH}\"
                }                    
            ],
            \"externalConsole\": true,
                    \"program\": \"${CMAKE_BINARY_DIR}/${TARGET}${CMAKE_EXECUTABLE_SUFFIX}\",
                    \"miDebuggerPath\": \"${GDB_EXECUTABLE}\",
                    \"MIMode\": \"gdb\",
                    \"setupCommands\": [
                        {
                            \"description\": \"Enable pretty-printing for gdb\",
                            \"text\": \"-enable-pretty-printing\",
                            \"ignoreFailures\": true
                        }
                        ]
            },\n")
        endif()
    endforeach(TARGET)

    configure_file("${CMAKE_SOURCE_DIR}/cmake/tasks.json.in" "${CMAKE_SOURCE_DIR}/.vscode/tasks.json")
    configure_file("${CMAKE_SOURCE_DIR}/cmake/settings.json.in" "${CMAKE_SOURCE_DIR}/.vscode/settings.json")
    configure_file("${CMAKE_SOURCE_DIR}/cmake/launch.json.in" "${CMAKE_SOURCE_DIR}/.vscode/launch.json")
    configure_file("${CMAKE_SOURCE_DIR}/cmake/c_cpp_properties.json.in" "${CMAKE_SOURCE_DIR}/.vscode/c_cpp_properties.json" @ONLY)
endif()

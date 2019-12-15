
#if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU" OR
#    "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
#    set(warnings "-Wall -Wextra -Werror")
#elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
#    set(warnings "/W4 /WX /EHsc")
#endif()
#set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} ${warnings}")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${warnings}")

#enable_language(CXX)


PROJECT (EvolutionBoard)

cmake_minimum_required (VERSION 3.16)
#set(CMAKE_C_COMPILER   "C:/Program Files (x86)/Atollic/TrueSTUDIO for STM32 9.3.0/ARMTools/bin/arm-atollic-eabi-gcc")
#set(CMAKE_CXX_COMPILER "C:/Program Files (x86)/Atollic/TrueSTUDIO for STM32 9.3.0/ARMTools/bin/arm-atollic-eabi-g++")
#set(CMAKE_ASM_COMPILER "C:/Program Files (x86)/Atollic/TrueSTUDIO for STM32 9.3.0/ARMTools/bin/arm-atollic-eabi-gcc")
#set(CMAKE_MAKE_PROGRAM "C:/Program Files (x86)/Atollic/TrueSTUDIO for STM32 9.3.0/Tools/make")
#message("Parameters")
#message(${CMAKE_C_COMPILER})
#message(${CMAKE_CXX_COMPILER})
#message(${CMAKE_ASM_COMPILER})
#message(${CMAKE_MAKE_PROGRAM})

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
	set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -std=gnu11")
	SET(CMAKE_ASM_FLAGS "${CFLAGS} -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -g -Wa,--warn -x assembler-with-cpp -specs=nano.specs")
endif()

set (CurrentPath ${CMAKE_CURRENT_SOURCE_DIR})
#set (BuildPath ${CurrentPath}/Build)
set (BuildPath ${CurrentPath})

if (EXISTS ${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
    file (COPY ${CurrentPath}/conan/conanbuildinfo.cmake DESTINATION ${BuildPath})
	include (${BuildPath}/conanbuildinfo.cmake)
	conan_basic_setup ()
endif()

set  (ProjectIncludePath                  ${CurrentPath}/Project)
FILE (GLOB_RECURSE ProjectFilesInclude    ${ProjectIncludePath}/*.h ${ProjectIncludePath}/*.hxx)
FILE (GLOB_RECURSE ProjectFilesAndFolders ${ProjectIncludePath}/*)

#set  (TestIncludePath                     ${CurrentPath}/Test)
#FILE (GLOB_RECURSE TestFilesInclude       ${TestIncludePath}/*.h ${TestIncludePath}/*.hxx)
#FILE (GLOB_RECURSE TestFilesAndFolders    ${TestIncludePath}/*)

MACRO(GET_SUBDIRECTORIES_PATHS_LISTS main_subdirectory_path subdirectories_paths_lists)
    SET(directoryPathsLists "")
    FOREACH(subdirectoryPath ${main_subdirectory_path})
        GET_FILENAME_COMPONENT(directoryPath ${subdirectoryPath} PATH)
        SET(directoryPathsLists ${directoryPathsLists} ${directoryPath})
    ENDFOREACH()
    LIST(REMOVE_DUPLICATES directoryPathsLists)
    SET(${subdirectories_paths_lists} ${directoryPathsLists})
ENDMACRO()

set(ProjectSubdirectoriesPathsLists "")
GET_SUBDIRECTORIES_PATHS_LISTS("${ProjectFilesAndFolders}" ProjectSubdirectoriesPathsLists)

#set(TestSubdirectoriesPathsLists "")
#GET_SUBDIRECTORIES_PATHS_LISTS("${TestFilesAndFolders}" TestSubdirectoriesPathsLists)

#MACRO(REMOVE_FILES directory)
#	File (GLOB removeLists ${directory}/*)
#	foreach (file ${removeLists})
#		if (EXISTS ${file})
#			file(REMOVE ${file})
#		endif()
#	endforeach(file)
#ENDMACRO()

#REMOVE_FILES("${BuildPath}")

source_group               (TREE ${ProjectIncludePath} PREFIX "" FILES ${ProjectFilesAndFolders})
add_executable             (Project ${ProjectFilesAndFolders})
target_include_directories (Project PUBLIC ${ProjectSubdirectoriesPathsLists})
target_link_libraries      (Project ${CONAN_LIBS})
#target_compile_features    (Project PUBLIC cxx_std_11)
target_compile_definitions (Project PRIVATE STM32F401xC USE_HAL_DRIVER)

#source_group               (TREE ${TestIncludePath} PREFIX "" FILES ${TestFilesAndFolders})
#link_directories           (Test ${BuildPath}/lib)
#add_executable             (Test ${TestFilesAndFolders})
#target_include_directories (Test PUBLIC ${ProjectSubdirectoriesPathsLists} ${TestSubdirectoriesPathsLists})
#target_link_libraries      (Test ProjectLib ${CONAN_LIBS})
#target_compile_features    (Test PUBLIC cxx_std_17)

#add_library                (ProjectLib STATIC ${ProjectFilesAndFolders})
#target_include_directories (ProjectLib PUBLIC ${ProjectSubdirectoriesPathsLists})
#target_compile_features    (ProjectLib PUBLIC cxx_std_17)







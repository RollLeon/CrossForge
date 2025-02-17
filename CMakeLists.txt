cmake_minimum_required(VERSION 3.16)

SET(VCPKG_BOOTSTRAP_OPTIONS "-disableMetrics")    # Disable telemetry for vcpkg.
SET(X_VCPKG_APPLOCAL_DEPS_INSTALL ON)             # Install vcpkg dependencies automatically(experimental - might be changed or removed later; see: https://github.com/microsoft/vcpkg/issues/1653). 

# Forbid in-source builds.
IF("${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_BINARY_DIR}")
	MESSAGE(SEND_ERROR "In-source builds are not allowed. Use a different build directory.")
ENDIF("${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_BINARY_DIR}")

# builds the crossforge library
include("crossforge/CrossForgeBuild.cmake")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# create required directories if non existent 
file(MAKE_DIRECTORY "MyAssets")
file(MAKE_DIRECTORY "Prototypes")
file(MAKE_DIRECTORY "Prototypes/Shader")

#################################################################
########### Project - CrossForge Sandbox ########################
#################################################################
project (CForgeSandbox)

if(EMSCRIPTEN)
	include(FetchContent)

	### Eigen3TerrainMesh
	FetchContent_Declare(
		tinyxml2
		GIT_REPOSITORY https://github.com/leethomason/tinyxml2.git
		GIT_TAG 9.0.0
	)
	FetchContent_MakeAvailable(tinyxml2)

	# tinygltf
	FetchContent_Declare(
		tinygltf
		URL https://github.com/syoyo/tinygltf/archive/refs/tags/v2.8.2.zip
	)
	FetchContent_MakeAvailable(tinygltf)
	include_directories(${tinygltf_SOURCE_DIR}/)

	FetchContent_Declare(
		imgui
		URL https://github.com/ocornut/imgui/archive/refs/tags/v1.89.4.zip
	)
	FetchContent_MakeAvailable(imgui)
	include_directories(${imgui_SOURCE_DIR}/)
	include_directories(${imgui_SOURCE_DIR}/backends)


	set(CMAKE_EXECUTABLE_SUFFIX .html)
else()
# Additional packages not alreday included in the core library
FIND_PACKAGE(tinyxml2 CONFIG REQUIRED)	# support for xml loading, storing, and processing
FIND_PACKAGE(imgui CONFIG REQUIRED) # dear-imgui
FIND_PACKAGE(flecs CONFIG REQUIRED) # dear-imgui
FIND_PACKAGE(jsoncpp CONFIG REQUIRED)
FIND_PACKAGE(recastnavigation CONFIG REQUIRED)
FIND_PACKAGE(Bullet CONFIG REQUIRED)
FIND_PACKAGE(behaviortree_cpp CONFIG REQUIRED)
endif()


# enable OpenCV support
if(USE_OPENCV)
	FIND_PACKAGE(OpenCV CONFIG REQUIRED)	# Open computer vision library
	include_directories(
		"${OpenCV_INCLUDE_DIRS}"
	)
	add_compile_definitions(USE_OPENCV)
endif(USE_OPENCV)

add_executable(CForgeSandbox
	SandboxMain.cpp

           Examples/edt/Components.h Examples/edt/PathSystem.h Examples/edt/PhysicsSystem.h Examples/edt/RobotActionNodes.h Examples/edt/StateMachine.h Examples/edt/SteeringSystem.h Examples/edt/Systems.h Examples/levelloading/LevelLoader.h
		Examples/edt/DebugDraw.h
		Examples/edt/PlayerSystem.h


)

if(EMSCRIPTEN)
	target_link_Libraries(CForgeSandbox
		tinyxml2
		crossforge
	)
	set_target_properties(CForgeSandbox PROPERTIES LINK_FLAGS "${LINK_FLAGS} ${Optimization_Flag} -sEXIT_RUNTIME=1 -sALLOW_MEMORY_GROWTH=1 -sWASM=1 -sUSE_WEBGL2=1 -fwasm-exceptions -sUSE_GLFW=3 -sUSE_ZLIB=1 -sUSE_LIBPNG=1 -sUSE_LIBJPEG=1 --preload-file Assets --preload-file Shader --preload-file MyAssets")

elseif(WIN32)
	target_link_libraries(CForgeSandbox
	PRIVATE crossforge
	PRIVATE glfw
	PRIVATE glad::glad
	ws2_32
	PRIVATE tinyxml2::tinyxml2
	PRIVATE imgui::imgui
	PRIVATE flecs::flecs
	PRIVATE JsonCpp::JsonCpp
	PRIVATE ${BULLET_LIBRARIES}
	PRIVATE BT::behaviortree_cpp
	${FREETYPE_LIBRARIES}
	RecastNavigation::Detour RecastNavigation::Recast RecastNavigation::DebugUtils RecastNavigation::DetourCrowd
	)

elseif(__arm__)
	target_link_libraries(CForgeSandbox
		PRIVATE crossforge
		PRIVATE glfw
		PRIVATE glad::glad
		PRIVATE tinyxml2::tinyxml2
		PRIVATE imgui::imgui
		PRIVATE JsonCpp::JsonCpp
#		PRIVATE assimp
		PRIVATE gpiod
		PRIVATE BT::behaviortree_cpp
		PRIVATE ${BULLETerrainMeshT_LIBRARIES}
		PRIVATE dl
		RecastNavigation::Detour RecastNavigation::Recast RecastNavigation::DebugUtils RecastNavigation::DetourCrowd
		)
elseif(UNIX)
	target_link_libraries(CForgeSandbox
		PRIVATE crossforge
		PRIVATE glfw
		PRIVATE glad::glad
		PRIVATE tinyxml2::tinyxml2
		PRIVATE imgui::imgui
#		PRIVATE assimp
		PRIVATE flecs::flecs
		PRIVATE JsonCpp::JsonCpp
		PRIVATE dl
		RecastNavigation::Detour RecastNavigation::Recast RecastNavigation::DebugUtils RecastNavigation::DetourCrowd
		PRIVATE ${BULLET_LIBRARIES}
		PRIVATE BT::behaviortree_cpp
		PRIVATE dl
		)
endif()

#copy shaders
add_custom_command(
	TARGET CForgeSandbox PRE_BUILD
	COMMAND ${CMAKE_COMMAND} -E copy_directory
	${CMAKE_SOURCE_DIR}/crossforge/Shader
	${CMAKE_CURRENT_BINARY_DIR}/Shader
)
add_custom_command(
	TARGET CForgeSandbox PRE_BUILD
	COMMAND ${CMAKE_COMMAND} -E copy_directory
	${CMAKE_SOURCE_DIR}/Prototypes/Shader
	${CMAKE_CURRENT_BINARY_DIR}/Shader
)

#copy assets
add_custom_command(
	TARGET CForgeSandbox PRE_BUILD
	COMMAND ${CMAKE_COMMAND} -E copy_directory
	${CMAKE_SOURCE_DIR}/Assets
	${CMAKE_CURRENT_BINARY_DIR}/Assets
)

add_custom_command(
	TARGET CForgeSandbox PRE_BUILD
	COMMAND ${CMAKE_COMMAND} -E copy_directory
	${CMAKE_SOURCE_DIR}/MyAssets
	${CMAKE_CURRENT_BINARY_DIR}/MyAssets
)


if(EMSCRIPTEN)
set(APP_FILES CForgeSandbox.html CForgeSandbox.js CForgeSandbox.wasm CForgeSandbox.data)
foreach(i ${APP_FILES})
add_custom_command(
	TARGET CForgeSandbox POST_BUILD
	COMMAND ${CMAKE_COMMAND} -E copy
	${CMAKE_CURRENT_BINARY_DIR}/${i}
	${CMAKE_CURRENT_BINARY_DIR}/App/${i}
)
endforeach(i)
endif(EMSCRIPTEN)
cmake_minimum_required(VERSION 3.28)
project(GeometryKit)

set(CMAKE_CXX_STANDARD 17)
add_definitions(-DNOMINMAX)


file(GLOB imgui_cpp ./include/imgui/*.cpp)
file(GLOB imguizmo_cpp ./include/ImGuizmo/*.cpp)
file(GLOB imguibeckends_cpp ./include/imgui/backends/*.cpp)

add_executable(GeometryKit src/main.cpp
        src/glad.c
        src/dmSimSvrUnity.pb.cc
        ${imgui_cpp}
        ${imguizmo_cpp}
        ${imguibeckends_cpp}
)

target_link_libraries(GeometryKit "${CMAKE_SOURCE_DIR}/libs/glfw3.lib")

set(RELEASE_LIBS
        ${CMAKE_SOURCE_DIR}/libs/lib_release/ecal_core.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/ecal_proto.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/libprotobuf.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_bad_any_cast_impl.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_bad_optional_access.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_bad_variant_access.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_base.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_city.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_civil_time.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_cord.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_cord_internal.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_cordz_functions.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_cordz_handle.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_cordz_info.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_cordz_sample_token.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_crc_cord_state.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_crc_cpu_detect.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_crc_internal.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_crc32c.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_debugging_internal.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_demangle_internal.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_die_if_null.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_examine_stack.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_exponential_biased.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_failure_signal_handler.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_flags.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_flags_commandlineflag.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_flags_commandlineflag_internal.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_flags_config.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_flags_internal.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_flags_marshalling.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_flags_parse.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_flags_private_handle_accessor.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_flags_program_name.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_flags_reflection.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_flags_usage.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_flags_usage_internal.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_graphcycles_internal.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_hash.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_hashtablez_sampler.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_int128.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_leak_check.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_log_entry.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_log_flags.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_log_globals.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_log_initialize.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_log_internal_check_op.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_log_internal_conditions.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_log_internal_format.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_log_internal_globals.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_log_internal_log_sink_set.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_log_internal_message.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_log_internal_nullguard.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_log_internal_proto.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_log_severity.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_log_sink.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_low_level_hash.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_malloc_internal.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_periodic_sampler.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_random_distributions.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_random_internal_distribution_test_util.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_random_internal_platform.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_random_internal_pool_urbg.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_random_internal_randen.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_random_internal_randen_hwaes.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_random_internal_randen_hwaes_impl.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_random_internal_randen_slow.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_random_internal_seed_material.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_random_seed_gen_exception.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_random_seed_sequences.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_raw_hash_set.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_raw_logging_internal.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_scoped_set_env.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_spinlock_wait.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_stacktrace.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_status.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_statusor.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_str_format_internal.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_strerror.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_strings.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_strings_internal.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_symbolize.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_synchronization.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_throw_delegate.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_time.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/absl_time_zone.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/utf8_range.lib
        ${CMAKE_SOURCE_DIR}/libs/lib_release/utf8_validity.lib
)
target_link_libraries(GeometryKit ${RELEASE_LIBS})

target_include_directories(GeometryKit PUBLIC include)
target_include_directories(GeometryKit PUBLIC include/igl)
target_include_directories(GeometryKit PUBLIC include/Eigen)
target_include_directories(GeometryKit PUBLIC include/glad)
target_include_directories(GeometryKit PUBLIC include/GLFW)
target_include_directories(GeometryKit PUBLIC include/imgui)
target_include_directories(GeometryKit PUBLIC include/ImGuizmo)
target_include_directories(GeometryKit PUBLIC include/ecal)
target_include_directories(GeometryKit PUBLIC include/google)
target_include_directories(GeometryKit PUBLIC include/absl)

if (MSVC)
    add_compile_options(/bigobj)
    set_target_properties(GeometryKit PROPERTIES
            LINK_FLAGS "/SUBSYSTEM:WINDOWS /ENTRY:mainCRTStartup"
    )
endif ()
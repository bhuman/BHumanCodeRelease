set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

set(CMAKE_CXX_COMPILER clang++)
set(CMAKE_CXX_COMPILER_TARGET aarch64-linux-gnu)
set(CMAKE_CXX_FLAGS_INIT "-nostdinc -nostdinc++")
set(CMAKE_SKIP_RPATH ON)
# From the documentation I would have thought that CMAKE_SKIP_RPATH implies CMAKE_SKIP_BUILD_RPATH.
# But apparently, this is not the case as with only CMAKE_SKIP_RPATH set, an rpath to libcl.so is added to the linker command line.
set(CMAKE_SKIP_BUILD_RPATH ON)

set(_buildchain_dir "${CMAKE_CURRENT_LIST_DIR}/../../Util/Buildchain")

if(MACOS OR WINDOWS)
  message(FATAL_ERROR "This file should not be included on macOS or Windows.")
else()
  set(_linker "ld.mold")
endif()

set(_t1_lib_dir "${_buildchain_dir}/T1/lib")
set(_t1_include_dir "${_buildchain_dir}/T1/include")
execute_process(COMMAND bash -c "for d in $(clang++ -c -v -E - </dev/null 2>&1 | sed -n -e '/here:/,/^End/{/here:/d; /^End/d; p}'); \
do [ -f $d/xmmintrin.h ] && echo $d; done | head -1" OUTPUT_VARIABLE _clang_system_include_dir)

set(CMAKE_CXX_LINK_EXECUTABLE "${_linker} <LINK_FLAGS> -nostdlib --hash-style=gnu --eh-frame-hdr -m aarch64linux \
-dynamic-linker=/lib/ld-linux-aarch64.so.1 -o <TARGET> \
\"${_t1_lib_dir}/crt1.o\" \"${_t1_lib_dir}/crti.o\" \"${_t1_lib_dir}/crtbegin.o\" \
-L\"${_t1_lib_dir}\" <OBJECTS> <LINK_LIBRARIES> \
\"${_t1_lib_dir}\"/libstdc++.so.6.0.30 \
--start-group \"${_t1_lib_dir}/libm.so.6\" --end-group \
--start-group \"${_t1_lib_dir}/libgcc_s.so.1\" \"${_t1_lib_dir}/libgcc.a\" --end-group \
\"${_t1_lib_dir}/libgcc.a\" \
--start-group \"${_t1_lib_dir}/libc.so.6\" \"${_t1_lib_dir}/libc_nonshared.a\" --as-needed \"${_t1_lib_dir}/ld-linux-aarch64.so.1\" --no-as-needed --end-group \
--start-group \"${_t1_lib_dir}/libgcc_s.so.1\" \"${_t1_lib_dir}/libgcc.a\" --end-group \
\"${_t1_lib_dir}/libgcc.a\" \
\"${_t1_lib_dir}/crtend.o\" \"${_t1_lib_dir}/crtn.o\"")

include_directories(SYSTEM "${_t1_include_dir}/c++/11")
include_directories(SYSTEM "${_t1_include_dir}/aarch64-linux-gnu/c++/11")
include_directories(SYSTEM "${_t1_include_dir}/c++/11/backward")
include_directories(SYSTEM "${_clang_system_include_dir}")
include_directories(SYSTEM "${_t1_include_dir}/aarch64-linux-gnu")
include_directories(SYSTEM "${_t1_include_dir}")

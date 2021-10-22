set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR x86_64)

set(CMAKE_CXX_COMPILER clang++)
set(CMAKE_CXX_COMPILER_TARGET x86_64-linux-gnu)
set(CMAKE_CXX_FLAGS_INIT "-nostdinc -nostdinc++ -march=silvermont -mtune=silvermont -mrdrnd")
set(CMAKE_SKIP_RPATH ON)
# From the documentation I would have thought that CMAKE_SKIP_RPATH implies CMAKE_SKIP_BUILD_RPATH.
# But apparently, this is not the case as with only CMAKE_SKIP_RPATH set, an rpath to libcl.so is added to the linker command line.
set(CMAKE_SKIP_BUILD_RPATH ON)

set(_buildchain_dir "${CMAKE_CURRENT_LIST_DIR}/../../Util/Buildchain")

if(CMAKE_HOST_APPLE OR CMAKE_HOST_WIN32)
  message(FATAL_ERROR "This file should not be included on macOS or Windows.")
else()
  set(_linker "ld.lld")
endif()

set(_v6_lib_dir "${_buildchain_dir}/V6/gcc/lib")
set(_v6_include_dir "${_buildchain_dir}/V6/gcc/include")
execute_process(COMMAND bash -c "for d in $(clang++ -c -v -E - </dev/null 2>&1 | sed -n -e '/here:/,/^End/{/here:/d; /^End/d; p}'); \
do [ -f $d/xmmintrin.h ] && echo $d; done | head -1" OUTPUT_VARIABLE _clang_system_include_dir)

set(CMAKE_CXX_LINK_EXECUTABLE "${_linker} <LINK_FLAGS> -nostdlib --hash-style=gnu --eh-frame-hdr -m elf_x86_64 \
-dynamic-linker=/lib64/ld-linux-x86-64.so.2 -o <TARGET> \
\"${_v6_lib_dir}/crt1.o\" \"${_v6_lib_dir}/crti.o\" \"${_v6_lib_dir}/crtbegin.o\" \
-L\"${_v6_lib_dir}\" <OBJECTS> <LINK_LIBRARIES> \
\"${_v6_lib_dir}\"/libstdc++.so.6.0.28 \
--start-group \"${_v6_lib_dir}/libm-2.31.so\" --as-needed \"${_v6_lib_dir}/libmvec-2.31.so\" --no-as-needed --end-group \
--start-group \"${_v6_lib_dir}/libgcc_s.so.1\" \"${_v6_lib_dir}/libgcc.a\" --end-group \
\"${_v6_lib_dir}/libgcc.a\" \
--start-group \"${_v6_lib_dir}/libc-2.31.so\" \"${_v6_lib_dir}/libc_nonshared.a\" --as-needed \"${_v6_lib_dir}/ld-2.31.so\" --no-as-needed --end-group \
--start-group \"${_v6_lib_dir}/libgcc_s.so.1\" \"${_v6_lib_dir}/libgcc.a\" --end-group \
\"${_v6_lib_dir}/libgcc.a\" \
\"${_v6_lib_dir}/crtend.o\" \"${_v6_lib_dir}/crtn.o\"")

include_directories(SYSTEM "${_v6_include_dir}/c++/9")
include_directories(SYSTEM "${_v6_include_dir}/x86_64-linux-gnu/c++/9")
include_directories(SYSTEM "${_v6_include_dir}/c++/9/backward")
include_directories(SYSTEM "${_clang_system_include_dir}")
include_directories(SYSTEM "${_v6_include_dir}/x86_64-linux-gnu")
include_directories(SYSTEM "${_v6_include_dir}")

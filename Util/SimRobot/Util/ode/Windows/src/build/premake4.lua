----------------------------------------------------------------------
-- Premake4 configuration script for OpenDE
-- Contributed by Jason Perkins (starkos@industriousone.com)
-- For more information on Premake: http://industriousone.com/premake
----------------------------------------------------------------------

  ode_version = "0.13.1"

----------------------------------------------------------------------
-- Demo list: add/remove demos from here and the rest of the build
-- should just work.
----------------------------------------------------------------------

  local demos = {
    "boxstack",
    "buggy",
    "cards",
    "chain1",
    "chain2",
    "collision",
    "crash",
    "cylvssphere",
    "dball",
    "dhinge",
    "feedback",
    "friction",
    "gyroscopic",
    "gyro2",
    "heightfield",
    "hinge",
    "I",
    "jointPR",
    "jointPU",
    "joints",
    "kinematic",
    "motion",
    "motor",
    "ode",
    "piston",
    "plane2d",
    "rfriction",
    "slider",
    "space",
    "space_stress",
    "step",
    "transmission"
  }

  local trimesh_demos = {
    "basket",
    "cyl",
    "moving_convex",
    "moving_trimesh",
    "tracks",
    "trimesh"
  }
  
  if not _OPTIONS["no-trimesh"] then
    demos = table.join(demos, trimesh_demos)
  end



----------------------------------------------------------------------
-- Configuration options
----------------------------------------------------------------------

  newoption {
    trigger     = "with-demos",
    description = "Builds the demo applications and DrawStuff library"
  }
  
  newoption {
    trigger     = "with-tests",
    description = "Builds the unit test application"
  }
  
  newoption {
    trigger     = "with-gimpact",
    description = "Use GIMPACT for trimesh collisions (experimental)"
  }
  
  newoption {
    trigger     = "all-collis-libs",
    description = "Include sources of all collision libraries into the project"
  }
  
  newoption {
    trigger     = "with-libccd",
    description = "Uses libccd for handling some collision tests absent in ODE."
  }
  
  newoption {
    trigger     = "no-dif",
    description = "Exclude DIF (Dynamics Interchange Format) exports"
  }
  
  newoption {
    trigger     = "no-trimesh",
    description = "Exclude trimesh collision geometry"
  }
  
  newoption {
    trigger     = "with-ou",
    description = "Use TLS for global caches (allows threaded collision checks for separated spaces)"
  }

  newoption {
    trigger     = "with-builtin-threading-impl",
    description = "Include built-in multithreaded threading implementation (still must be created and assigned to be used)"
  }

  newoption {
    trigger     = "no-threading-intf",
    description = "Disable threading interface support (external implementations may not be assigned; overrides with-builtin-threading-impl)"
  }

  newoption {
    trigger     = "16bit-indices",
    description = "Use 16-bit indices for trimeshes (default is 32-bit)"
  }

  newoption {
    trigger     = "old-trimesh",
    description = "Use old OPCODE trimesh-trimesh collider"
  }
  
  newoption {
    trigger     = "to",
    value       = "path",
    description = "Set the output location for the generated project files"
  }
  
  newoption {
    trigger     = "only-shared",
	description = "Only build shared (DLL) version of the library"
  }
  
  newoption {
    trigger     = "only-static",
	description = "Only build static versions of the library"
  }

  newoption {
    trigger     = "only-single",
	description = "Only use single-precision math"
  }
  
  newoption {
    trigger     = "only-double",
	description = "Only use double-precision math"
  }
  
  -- always clean all of the optional components and toolsets
  if _ACTION == "clean" then
    _OPTIONS["with-demos"] = ""
    _OPTIONS["with-tests"] = ""
    for action in premake.action.each() do
      os.rmdir(action.trigger)
    end
    os.remove("../ode/src/config.h")
    os.remove("../include/ode/version.h")
    os.remove("../include/ode/precision.h")
    os.remove("../libccd/src/ccd/precision.h")
  end
  
  -- special validation for Xcode  
  if _ACTION == "xcode3" and (not _OPTIONS["only-static"] and not _OPTIONS["only-shared"]) then
    error(
	  "Xcode does not support different library types in a single project.\n" ..
	  "Please use one of the flags: --only-static or --only-shared", 0)
  end

  -- build the list of configurations, based on the flags. Ends up
  -- with configurations like "Debug", "DebugSingle" or "DebugSingleShared"
  local configs = { "Debug", "Release" }
  
  local function addconfigs(...)
	local newconfigs = { }
	for _, root in ipairs(configs) do
	  for _, suffix in ipairs(arg) do
		table.insert(newconfigs, root .. suffix)
	  end
	end
	configs = newconfigs
  end
  
  
  if not _OPTIONS["only-single"] and not _OPTIONS["only-double"] then
    addconfigs("Single", "Double")
  end
  
  if not _OPTIONS["only-shared"] and not _OPTIONS["only-static"] then
    addconfigs("DLL", "Lib")
  end

  
----------------------------------------------------------------------
-- The solution, and solution-wide settings
----------------------------------------------------------------------

  solution "ode"

    language "C++"
    uuid     "4DA77C12-15E5-497B-B1BB-5100D5161E15"
    location ( _OPTIONS["to"] or _ACTION )

    includedirs {
      "../include",
      "../ode/src"
    }
    
    defines { "_MT" }
    
    -- apply the configuration list built above
    configurations (configs)
    
    configuration { "Debug*" }
      defines { "_DEBUG" }
      flags   { "Symbols" }
      
    configuration { "Release*" }
      defines { "NDEBUG", "dNODEBUG" }
      flags   { "OptimizeSpeed", "NoFramePointer" }

    configuration { "*Single*" }
      defines { "dIDESINGLE", "CCD_IDESINGLE" }
      
    configuration { "*Double*" }
      defines { "dIDEDOUBLE", "CCD_IDEDOUBLE" }
    
    configuration { "Windows" }
      defines { "WIN32" }

    configuration { "MacOSX" }
      linkoptions { "-framework Carbon" }
      
    -- give each configuration a unique output directory
    for _, name in ipairs(configurations()) do
      configuration { name }
        targetdir ( "../lib/" .. name )
    end
      
    -- disable Visual Studio security warnings
    configuration { "vs*" }
      defines { "_CRT_SECURE_NO_DEPRECATE" }

    -- enable M_* macros from math.h
    configuration { "vs*" }
      defines { "_USE_MATH_DEFINES" }

    -- don't remember why we had to do this	
    configuration { "vs2002 or vs2003", "*Lib" }
      flags  { "StaticRuntime" }



----------------------------------------------------------------------
-- The demo projects, automated from list above. These go first so
-- they will be selected as the active project automatically in IDEs
----------------------------------------------------------------------

  if _OPTIONS["with-demos"] then
    for _, name in ipairs(demos) do
    
      project ( "demo_" .. name )
      
        kind      "ConsoleApp"
        location  ( _OPTIONS["to"] or _ACTION )
        files     { "../ode/demo/demo_" .. name .. ".*" }
		links     { "ode", "drawstuff" }        
        
        configuration { "Windows" }
          files   { "../drawstuff/src/resources.rc" }
          links   { "user32", "winmm", "gdi32", "opengl32", "glu32" }

        configuration { "MacOSX" }
          linkoptions { "-framework Carbon -framework OpenGL -framework AGL" }

        configuration { "not Windows", "not MacOSX" }
          links   { "GL", "GLU" }
        
    end
  end
  


----------------------------------------------------------------------
-- The ODE library project
----------------------------------------------------------------------

  project "ode"

    -- kind     "StaticLib"
    location ( _OPTIONS["to"] or _ACTION )

    includedirs {
      "../ode/src/joints",
      "../OPCODE",
      "../GIMPACT/include",
      "../libccd/src"
    }

    files {
      "../include/ode/*.h",
      "../ode/src/joints/*.h", 
      "../ode/src/joints/*.cpp",
      "../ode/src/*.h", 
      "../ode/src/*.c", 
      "../ode/src/*.cpp",
    }

    excludes {
      "../ode/src/collision_std.cpp",
    }

    if _OPTIONS["with-ou"] or not _OPTIONS["no-threading-intf"] then
      includedirs { "../ou/include" }
      files   { "../ou/**.h", "../ou/**.cpp" }
      defines { "_OU_NAMESPACE=odeou" }

      if _ACTION == "gmake" and ( os.get() == "linux" or os.get() == "bsd" ) then
        buildoptions { "-pthread" }
        linkoptions { "-pthread" }
      end

      if _ACTION == "gmake" and os.get() == "windows" then
        buildoptions { "-mthreads" }
        linkoptions { "-mthreads" }
      end

      -- TODO: MacOSX probably needs something too
    end

      
    configuration { "no-dif" }
      excludes { "../ode/src/export-dif.cpp" }

    configuration { "no-trimesh" }
      excludes {
        "../ode/src/collision_trimesh_colliders.h",
        "../ode/src/collision_trimesh_internal.h",
        "../ode/src/collision_trimesh_opcode.cpp",
        "../ode/src/collision_trimesh_gimpact.cpp",
        "../ode/src/collision_trimesh_box.cpp",
        "../ode/src/collision_trimesh_ccylinder.cpp",
        "../ode/src/collision_cylinder_trimesh.cpp",
        "../ode/src/collision_trimesh_distance.cpp",
        "../ode/src/collision_trimesh_ray.cpp",
        "../ode/src/collision_trimesh_sphere.cpp",
        "../ode/src/collision_trimesh_trimesh.cpp",
        "../ode/src/collision_trimesh_plane.cpp"
      }

    configuration { "not no-trimesh", "with-gimpact or all-collis-libs" }
      files   { "../GIMPACT/**.h", "../GIMPACT/**.cpp" }
    
    configuration { "not no-trimesh", "not with-gimpact" }
      files   { "../OPCODE/**.h", "../OPCODE/**.cpp" }
 
    configuration { "with-libccd" }
      files   { "../libccd/src/ccd/*.h", "../libccd/src/*.c" }
      defines { "dLIBCCD_ENABLED", "dLIBCCD_CYL_CYL" }

    configuration { "not with-libccd" }
      excludes { "../ode/src/collision_libccd.cpp", "../ode/src/collision_libccd.h" }

    configuration { "windows" }
      links   { "user32" }
            
    configuration { "only-static or *Lib" }
      kind    "StaticLib"
      defines "ODE_LIB"
      
    configuration { "only-shared or *DLL" }
      kind    "SharedLib"
      defines "ODE_DLL"

    configuration { "*DLL" }
      defines "_DLL"

    configuration { "Debug" }
      targetname "oded"
	  
    configuration { "Release" }
      targetname "ode"
	  
    configuration { "DebugSingle*" }
      targetname "ode_singled"
      
    configuration { "ReleaseSingle*" }
      targetname "ode_single"
      
    configuration { "DebugDouble*" }
      targetname "ode_doubled"
      
    configuration { "ReleaseDouble*" }
      targetname "ode_double"


----------------------------------------------------------------------
-- Write a custom <config.h> to build, based on the supplied flags
----------------------------------------------------------------------

  if _ACTION and _ACTION ~= "clean" then
    local infile = io.open("config-default.h", "r")
    local text = infile:read("*a")

    if _OPTIONS["no-trimesh"] then
      text = string.gsub(text, "#define dTRIMESH_ENABLED 1", "/* #define dTRIMESH_ENABLED 1 */")
      text = string.gsub(text, "#define dTRIMESH_OPCODE 1", "/* #define dTRIMESH_OPCODE 1 */")
    elseif (_OPTIONS["with-gimpact"]) then
      text = string.gsub(text, "#define dTRIMESH_OPCODE 1", "#define dTRIMESH_GIMPACT 1")
    end

    if _OPTIONS["with-ou"] or not _OPTIONS["no-threading-intf"] then
      text = string.gsub(text, "/%* #define dOU_ENABLED 1 %*/", "#define dOU_ENABLED 1")
      text = string.gsub(text, "/%* #define dATOMICS_ENABLED 1 %*/", "#define dATOMICS_ENABLED 1")
    end

    if _OPTIONS["with-ou"] then
      text = string.gsub(text, "/%* #define dTLS_ENABLED 1 %*/", "#define dTLS_ENABLED 1")
    end

    if _OPTIONS["no-threading-intf"] then
      text = string.gsub(text, "/%* #define dTHREADING_INTF_DISABLED 1 %*/", "#define dTHREADING_INTF_DISABLED 1")
    elseif _OPTIONS["with-builtin-threading-impl"] then
      text = string.gsub(text, "/%* #define dBUILTIN_THREADING_IMPL_ENABLED 1 %*/", "#define dBUILTIN_THREADING_IMPL_ENABLED 1")
    end

    if _OPTIONS["16bit-indices"] then
      text = string.gsub(text, "#define dTRIMESH_16BIT_INDICES 0", "#define dTRIMESH_16BIT_INDICES 1")
    end
  
    if _OPTIONS["old-trimesh"] then
      text = string.gsub(text, "#define dTRIMESH_OPCODE_USE_OLD_TRIMESH_TRIMESH_COLLIDER 0", "#define dTRIMESH_OPCODE_USE_OLD_TRIMESH_TRIMESH_COLLIDER 1")
    end
    
    local outfile = io.open("../ode/src/config.h", "w")
    outfile:write(text)
    outfile:close()
  end

----------------------------
-- Write precision headers
----------------------------
  if _ACTION and _ACTION ~= "clean" then
    function generateheader(headerfile, placeholder, precstr)
      local outfile = io.open(headerfile, "w")
      for i in io.lines(headerfile .. ".in")
      do
        local j,_ = string.gsub(i, placeholder, precstr)
        --print("writing " .. j .. " into " .. headerfile)
        outfile:write(j .. "\n")
      end
      outfile:close()
    end
    
    function generate(precstr)
      generateheader("../include/ode/precision.h", "@ODE_PRECISION@", "d" .. precstr)
      generateheader("../libccd/src/ccd/precision.h", "@CCD_PRECISION@", "CCD_" .. precstr)
    end
    
    if _OPTIONS["only-single"] then
      generate("SINGLE")
    elseif _OPTIONS["only-double"] then
      generate("DOUBLE")
    else 
      generate("UNDEFINEDPRECISION")
    end

    generateheader("../include/ode/version.h", "@ODE_VERSION@", ode_version)

  end


----------------------------------------------------------------------
-- The DrawStuff library project
----------------------------------------------------------------------

  if _OPTIONS["with-demos"] then

    project "drawstuff"

      location ( _OPTIONS["to"] or _ACTION )

      files {
        "../include/drawstuff/*.h",
        "../drawstuff/src/internal.h",
        "../drawstuff/src/drawstuff.cpp"
      }
      
      configuration { "Debug*" }
        targetname "drawstuffd"
            
      configuration { "only-static or *Lib" }
        kind    "StaticLib"
        defines { "DS_LIB" }
      
      configuration { "only-shared or *DLL" }
        kind    "SharedLib"
        defines { "DS_DLL", "USRDLL" }
      
      configuration { "Windows" }
        files   { "../drawstuff/src/resource.h", "../drawstuff/src/resources.rc", "../drawstuff/src/windows.cpp" }
        links   { "user32", "opengl32", "glu32", "winmm", "gdi32" }

      configuration { "MacOSX" }
	    defines     { "HAVE_APPLE_OPENGL_FRAMEWORK" }
        files       { "../drawstuff/src/osx.cpp" }
        linkoptions { "-framework Carbon -framework OpenGL -framework AGL" }

      configuration { "not Windows", "not MacOSX" }
        files   { "../drawstuff/src/x11.cpp" }
        links   { "X11", "GL", "GLU" }

  end


----------------------------------------------------------------------
-- The automated test application
----------------------------------------------------------------------


  if _OPTIONS["with-tests"] then
  
    project "tests"
  
      kind     "ConsoleApp"
      location ( _OPTIONS["to"] or _ACTION )

      includedirs { 
        "../tests/UnitTest++/src" 
      }
    
      files { 
        "../tests/*.cpp", 
        "../tests/joints/*.cpp", 
        "../tests/UnitTest++/src/*" 
      }

      links { "ode" }
    
      configuration { "Windows" }
        files { "../tests/UnitTest++/src/Win32/*" }
      
      configuration { "not Windows" }
        files { "../tests/UnitTest++/src/Posix/*" }

      -- add post-build step to automatically run test executable
      local path_to_lib = path.getrelative(location(), "../lib")
      local command = path.translate(path.join(path_to_lib, "%s/tests"))
      
      for _, name in ipairs(configurations()) do
        configuration { name }
          postbuildcommands { command:format(name) }
      end

  end


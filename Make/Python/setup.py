# -*- coding: utf-8 -*-
# Stolen from: https://github.com/pybind/cmake_example
import os
import shutil
import subprocess
import sys
from pathlib import Path

from setuptools import Extension, setup
from setuptools.command.build_ext import build_ext

# Convert distutils Windows platform specifiers to CMake -A arguments
PLAT_TO_CMAKE = {
    'win32': 'Win32',
    'win-amd64': 'x64',
    'win-arm32': 'ARM',
    'win-arm64': 'ARM64',
}


# A CMakeExtension needs a sourcedir instead of a file list.
# The name must be the _single_ output extension from the CMake build.
# If you need multiple extensions, see scikit-build.
class CMakeExtension(Extension):
    def __init__(self, name, sourcedir=''):
        Extension.__init__(self, name, sources=[])
        self.sourcedir = os.path.join(os.path.abspath(sourcedir), '..', '..', 'Make', 'CMake')


class CMakeBuild(build_ext):
    def build_extension(self, ext):
        extdir = os.path.abspath(os.path.dirname(self.get_ext_fullpath(ext.name)))

        # required for auto-detection of auxiliary "native" libs
        if not extdir.endswith(os.path.sep):
            extdir += os.path.sep

        cfg = 'Debug' if self.debug else 'Release'

        # CMake lets you override the generator - we need to check this.
        # Can be set with Conda-Build, for example.
        cmake_generator = os.environ.get('CMAKE_GENERATOR', '')

        cmake_args = [
            f'-DCMAKE_LIBRARY_OUTPUT_DIRECTORY={extdir}',
            f'-DREAL_VERSION_INFO={self.distribution.get_version()}',
            f'-DCMAKE_BUILD_TYPE={cfg}',  # not used on MSVC, but no harm
            '-DPYTHON_ONLY=TRUE',
            f'-DPython3_ROOT_DIR={sys.prefix}',
            f'-DPYTHON_VERSION={sys.version_info.major}.{sys.version_info.minor}.{sys.version_info.micro}',
            '-DFIND_PYTHON_EXTRA_ARGS=EXACT',
        ]
        build_args = []

        if self.compiler.compiler_type != 'msvc':
            # Using Ninja-build since it a) is available as a wheel and b)
            # multithreads automatically. MSVC would require all variables be
            # exported for Ninja to pick it up, which is a little tricky to do.
            # Users can override the generator with CMAKE_GENERATOR in CMake
            # 3.15+.
            if not cmake_generator:
                cmake_args += ['-GNinja']

        else:
            # Single config generators are handled "normally"
            single_config = any(x in cmake_generator for x in {'NMake', 'Ninja'})

            # CMake allows an arch-in-generator style for backward compatibility
            contains_arch = any(x in cmake_generator for x in {'ARM', 'Win64'})

            # Specify the arch if using MSVC generator, but only if it doesn't
            # contain a backward-compatibility arch spec already in the
            # generator name.
            if not single_config and not contains_arch:
                cmake_args += ['-A', PLAT_TO_CMAKE[self.plat_name]]

            # Multi-config generators have a different way to specify configs
            if not single_config:
                cmake_args += [
                    f'-DCMAKE_LIBRARY_OUTPUT_DIRECTORY_{cfg.upper()}={extdir}'
                ]
                build_args += ['--config', cfg]

        # Set CMAKE_BUILD_PARALLEL_LEVEL to control the parallel build level
        # across all generators.
        if 'CMAKE_BUILD_PARALLEL_LEVEL' not in os.environ:
            # self.parallel is a Python 3 only way to set parallel jobs by hand
            # using -j in the build_ext call, not supported by pip or PyPA-build.
            if hasattr(self, 'parallel') and self.parallel:
                # CMake 3.12+ only.
                build_args += [f'-j{self.parallel}']

        if not os.path.exists(self.build_temp):
            os.makedirs(self.build_temp)

        subprocess.check_call(
            ['cmake', ext.sourcedir] + cmake_args, cwd=self.build_temp
        )
        subprocess.check_call(
            ['cmake', '--build', '.', '--target', 'PythonLogs']
            + build_args,
            cwd=self.build_temp,
        )
        stubs_path = Path(self.build_temp) / 'stubs'
        sys.path.append(Path(self.build_lib).resolve().as_posix())
        try:
            self.stubgen(stubs_path, 'pybh.logs')
            shutil.copy(
                stubs_path / 'pybh' / 'logs.pyi', Path(self.build_lib) / 'pybh' / 'logs.pyi'
            )
        except ImportError:
            # skip stub generation if pybind11_stubgen could not be found
            pass

    def stubgen(self, output_dir: Path, module_name: str):
        from pybind11_stubgen import run
        from pybind11_stubgen.parser.mixins.error_handlers import (
            LogErrors,
            LoggerData,
            SuggestCxxSignatureFix,
        )
        from pybind11_stubgen.parser.mixins.filter import (
            FilterClassMembers,
            FilterInvalidIdentifiers,
            FilterPybind11ViewClasses,
            FilterPybindInternals,
            FilterTypingModuleAttributes,
        )
        from pybind11_stubgen.parser.mixins.fix import (
            FixBuiltinTypes,
            FixCurrentModulePrefixInTypeNames,
            FixMissing__all__Attribute,
            FixMissing__future__AnnotationsImport,
            FixMissingEnumMembersAnnotation,
            FixMissingFixedSizeImport,
            FixMissingImports,
            FixMissingNoneHashFieldAnnotation,
            FixNumpyArrayFlags,
            FixNumpyDtype,
            FixPEP585CollectionNames,
            FixPybind11EnumStrDoc,
            FixRedundantBuiltinsAnnotation,
            FixRedundantMethodsFromBuiltinObject,
            FixScipyTypeArguments,
            FixTypingTypeNames,
            FixValueReprRandomAddress,
            OverridePrintSafeValues,
            RemoveSelfAnnotation,
            ReplaceReadWritePropertyWithField,
            RewritePybind11EnumValueRepr,
        )
        from pybind11_stubgen.parser.mixins.parse import (
            BaseParser,
            ExtractSignaturesFromPybind11Docstrings,
            ParserDispatchMixin,
        )
        from pybind11_stubgen.printer import Printer
        from pybind11_stubgen.writer import Writer

        error_handlers_top: list[type] = [
            LoggerData,
        ]
        error_handlers_bottom: list[type] = [
            LogErrors,
            SuggestCxxSignatureFix,
        ]

        class Parser(
            *error_handlers_top,  # type: ignore[misc]
            FixMissing__future__AnnotationsImport,
            FixMissing__all__Attribute,
            FixMissingNoneHashFieldAnnotation,
            FixMissingImports,
            FilterTypingModuleAttributes,
            FixPEP585CollectionNames,
            FixTypingTypeNames,
            FixScipyTypeArguments,
            FixMissingFixedSizeImport,
            FixMissingEnumMembersAnnotation,
            OverridePrintSafeValues,
            FixNumpyDtype,
            FixNumpyArrayFlags,
            FixCurrentModulePrefixInTypeNames,
            FixBuiltinTypes,
            RewritePybind11EnumValueRepr,
            FilterClassMembers,
            ReplaceReadWritePropertyWithField,
            FilterInvalidIdentifiers,
            FixValueReprRandomAddress,
            FixRedundantBuiltinsAnnotation,
            FilterPybindInternals,
            FilterPybind11ViewClasses,
            FixRedundantMethodsFromBuiltinObject,
            RemoveSelfAnnotation,
            FixPybind11EnumStrDoc,
            ExtractSignaturesFromPybind11Docstrings,
            ParserDispatchMixin,
            BaseParser,
            *error_handlers_bottom,  # type: ignore[misc]
        ):
            pass

        parser = Parser()
        printer = Printer(invalid_expr_as_ellipses=True)
        run(
            parser,
            printer,
            module_name,
            output_dir.joinpath(*module_name.split('.')[:-1]),
            sub_dir=None,
            dry_run=False,
            writer=Writer(),
        )


# The information here can also be placed in setup.cfg - better separation of
# logic and declaration, and simpler if you include description/version in a file.
setup(
    name='pybh',
    version='0.3.10rc0',
    author='B-Human',
    author_email='b-human@uni-bremen.de',
    url='https://b-human.de',
    description='Python bindings for B-Human.',
    ext_modules=[CMakeExtension('pybh.')],  # . to create a folder for the libs
    cmdclass={'build_ext': CMakeBuild},
    zip_safe=False,
)

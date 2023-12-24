from setuptools import setup, Extension
from Cython.Build import cythonize
from pathlib import Path

# ext_modules = [
#     Extension(
#         "lkh3_wrapper",
#         sources=["src/lkh3_interface.pyx", "src/lkh3_interface.cpp"],
#         language="c++",
#     ),
# ]

# ext_modules = [
#     Extension(
#         "lkh3_wrapper",
#         sources=["src/lkh3_interface.pyx", "src/lkh3_interface.cpp"],
#         language="c++",
#         include_dirs=["include", "LKH-3.0.6/SRC/INCLUDE"],
#         libraries=["lkh_mtsp_solver"],  # Specify the name of your library (without 'lib' prefix or file extension)
#         library_dirs=["build"],
#     ),
# ]
#
# setup(
#     ext_modules=cythonize(ext_modules),
# )

# SRC_FILE = [str(p) for p in Path("LKH-3.0.6/SRC").glob("*.c")]
SRC_FILE = []
SRC_FILE.append("src/lkh3_interface.pyx")
SRC_FILE.append("src/lkh3_interface.cpp")

ext_modules = [
    Extension(
        "lkh3_wrapper",
        sources=SRC_FILE,
        include_dirs=["include", "LKH-3.0.6/SRC/INCLUDE"],
        library_dirs=["build"],
        libraries=["lkh_mtsp_solver"],
        language="c++",
        extra_compile_args=["-std=c++17"],
    ),
]

setup(
    ext_modules=cythonize(ext_modules),
)
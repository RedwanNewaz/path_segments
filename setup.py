from distutils.core import setup, Extension

target_module = Extension('cbd_ta',
                        sources=['src/cbd_ta/main_cbd_py.cpp'],
                        include_dirs=['/usr/local/include', 'include', '/usr/include/eigen3',
                                      'build/install/include'],
                        library_dirs=['/usr/lib/x86_64-linux-gnu','build', 'build/install/lib', '/usr/local/lib'],
                        runtime_library_dirs=['/usr/lib/x86_64-linux-gnu','build/install/lib', 'build'],
                        libraries=['boost_python3', 'fcl', 'ccd'])

setup(name='cbd_ta',
      version='0.1',
      description='Multiagent conflict based path decomposition',
      package_dir={'': 'src'},
      packages=['cbd_ta'],
      ext_modules=[target_module],
      python_requires=">=3.6"
      )

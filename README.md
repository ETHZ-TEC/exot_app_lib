This repository contains the application library used to build __benchmarking
tools__ and __covert channel applications__. The new library provides a core
framework making use of the *process networks* model of computation. The new
architecture defines a clearer structure for constructing covert channel
applications, and allows considerable degrees of reuse and extendability.
Additionally, the library features a breadth of utilities, which speed up the
development process, reduce code duplication, and improve the maintainability
of the codebase.

For further information and guildelines, please consult the wiki.

> __New__: The repository now contains a *tools* directory. You can find a new __containerised development environment__ there, additional information can be found in a new [README](tools/docker/README.md).

# How to use the library

This project uses [CMake](https://cmake.org/) as the build tool. It is
strongly encouraged that CMake is used for building all software based on the
library. The project's build configuration file defines a library target
called **exot**.

The library requires a number of third-party libraries:
[fmt](https://github.com/fmtlib/fmt) for string formatting,
[spdlog](https://github.com/gabime/spdlog) for logging,
[clipp](https://github.com/muellan/clipp) for command-line parsing, and
[doctest](https://github.com/onqtam/doctest) for testing. These are provided
as git submodules in the repository. To import them during cloning, run `git
clone --recursive <repo-url>`, to do that after the library has been imported,
make sure to run `git submodule update --init --recursive` to sync the
dependent repositories. To use the library at a specific tag or commit, git
submodules can also be utilised.

To use the library, include the repository in your project's path, and import
it in the project's CMakeLists.txt using `add_subdirectory`:

```cmake
cmake_minimum_required(VERSION 3.6)
project(project-that-uses-the-library VERSION 0.0.0 LANGUAGES CXX)

add_subdirectory(<path/to/library>)

# ...

target_link_libraries(<target/name> exot)

```

To successfully compile one needs:

- *CMake* ~> 3.6, preferably one that understands the C++17 standard (~> 3.8);
- a recent C++ standard library, conforming to the C++17 standard. This could be
  the GNU libstdc++ from [GCC 7] and higher (libstdc++.so.6.0.23 and higher).
  Please refer to the wiki for [futher guidelines];
- (optionally) *Boost* libraries, with *Boost/fiber* available,
  enabled by passing -Dexot_use_fibers to the CMake command.

[GCC 7]: https://gcc.gnu.org/projects/cxx-status.html#cxx17
[futher guidelines]: https://gitlab.ethz.ch/tec/research/data_leakage_evaluation/app_lib/wikis/How-to-use-the-library%3F

The build process can be driven by *toolchain files*, which can wrap compiler,
linker, and other environment parameters and tasks necessary for building the
library and applications. If no *toolchain file* is provided, the build will
be performed with the system compiler, or the one defined with environment
variables `CC`/`CXX`. The repository contains, in
[cmake/toolchains](cmake/toolchains), a number of toolchains for dynamic and
static linkage, and for cross-compilation.

A number of build configurations are available: *Debug*, *Release*,
*RelWithDebInfo*. To configure the build with the desired configuration, run:

```bash
# Debug configuration, with symbols, static analysis, and no optimisation
cmake -DCMAKE_TOOLCHAIN_FILE=<path/to/toolchain> \
	  -DCMAKE_BUILD_TYPE=Debug -Bbuild/Debug -H.
# Release build, no symbols, no debug statements, high optimisation
cmake -DCMAKE_TOOLCHAIN_FILE=<path/to/toolchain> \
	  -DCMAKE_BUILD_TYPE=Release -Bbuild/Release -H.
```

Then, to compile specific targets, choose the build folder and run:

```bash
cmake --build build/Debug --target <target name> -- -j 0
```

To build and execute the library tests, run:

```bash
cmake --build  <path/to/build> --target exot-test
./path/to/build/exot/exot-test
```

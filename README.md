# ppk2-lib

ppk2-lib is a lightweight C++ library and command-line interface for interacting with the Nordic Power Profiler Kit II (PPK2).

The project provides direct control of the PPK2 and access to its measurement data without requiring the standard desktop application, making it useful for automated measurements, scripting, and custom data acquisition workflows.

The repository contains:

* **C++ interface** — communication and control of the PPK2
* **Command-line interface** — configure and operate the PPK2 directly from a terminal
* **Measurement acquisition** — retrieve power measurement data for further processing or analysis

The project was developed to provide a simple, lightweight way of integrating the PPK2 into custom measurement and development workflows.

## Acknowledgements

This project was inspired by the [IRNAS ppk2-api-python](https://github.com/IRNAS/ppk2-api-python) project, an unofficial Python API for the Nordic Power Profiler Kit II.

ppk2-lib implements similar functionality in C++, with a focus on lightweight command-line usage and integration into C++ applications.

Many thanks to the IRNAS project for documenting and demonstrating communication with the PPK2.

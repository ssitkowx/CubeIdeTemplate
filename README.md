# CubeIdeTemplate

# I. Description:
Template for CubeIde embedded projects.

# II. Assumptions:
The code stored here is environment-specific, which means:
- The code depends on the environment in which it works,
- The code can use both generic and evironment-specific external libraries.

# III. Structure:
The embedded project has been divided into three parts:

External:
- This is where external generic libraries are stored,
- This is where documents and resources are stored.

Project:

- Logic:
  - This is where environment-specific but hardware-independent code is stored.

- Hardware:
  - This is where the environment-specific code is stored.

Tests:
- which uses gtest and/or gmock libraries.

# IV. Configuration:

- STM32CubeIDE Version: 1.11.2
- CubeMx 6.7.0.

# V. Builidng:
Packages in External:
- The same as in the case of projects based on the "Template".

Project:
- Add to the project (in the CMakeLists.txt) files located in the ".conan\download" in the "External" folder,
- Add to the project (in the CMakeLists.txt) files located in the "Logic" and "Hardware" folders,
- Open a project gui and build a project.

# VI. Tips:
- During bringing new hardware to the project copy only necessary files (compare by files in project tree).
- To enable C++ settings add org.eclipse.cdt.core.ccnature to .project.
- After generating the equipment using CubeMx, transfer the .cproject, .project, .mxproject files from the Atollic folder to the folder with the project
  name (AtollicTemplate).
  - In the C/C++ Build->Settings->Tool Settings->C++ Linker->General settings we provide the path to the linker script,
  - In C/C++ Build-> Target Settings we choose the type of processor,
  - In C/C++ General->Paths and Symbols->Include, we provide the hardware paths updates.

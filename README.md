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
  - This is were hardware-independent and general code is stored,
  -  At the end of development this layer should be included in packages.

- Middleware:
  - This is where environment-specific but hardware-independent code is stored.

- Hardware:
  - This is where the environment-specific code is stored.

Tests:
- which uses GTest and/or GMock libraries.

# IV. Configuration:

- STM32CubeIDE Version: 1.15.0
- CubeMx 6.11.0.

# V. Builidng:
Packages in External:
- The same as in the case of projects based on the "Template".

Project:
- Add to the Project to folder "External" files and paths located in the ".conan2/download/repos" directory,

# VI. Tips:
- To enable C++ settings add org.eclipse.cdt.core.ccnature to .project.
- After generating the equipment using CubeMx, transfer the .cproject, .project, .mxproject files from the CubeIde folder to the folder with the project
  name (CubeIdeTemplate).
  - In the C/C++ Build->Settings->Tool Settings-MCU >G++ Linker->General we provide the path to the linker script,
  - In C/C++ General->Paths and Symbols->Include, we provide the paths updates.
- In C/C++ Build->Build set variable APPLICATION_PACKAGES on /home/?/.conan2/download/repos
- During conan source . command execution might not find all paths, repeting operation might help.

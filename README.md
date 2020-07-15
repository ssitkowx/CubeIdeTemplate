# AtollicTemplate

# I. Description:
Template for Atollic embedded projects.

# II. Assumptions:
The code stored here is environment-specific, which means:
- The code depends on the environment in which it works,
- The code can use both generic and evironment-specific external libraries.

# III. Structure:
The embedded project has been divided into two parts:
Logic:
- This is where external generic libraries are stored,
- This is where environment-specific but hardware-independent code is stored.

Hardware:
- This is where the environment-specific code is kept.

# IV. Configuration:
Logic:
The same as in the case of projects based on the "Template".

Hardware:
- Atollic TrueSTUDIO® for STM32. Built on Eclipse Neon.1a. Version 9.3.1,
- CubeMx 5.3.0.

# V. Builidng:
Logic:
The same as in the case of projects based on the "Template".

Hardware:
- Add to the project files located in the ".conan\download" after "Logic" building,
- Add to the project files located in the "Hardware",
- Open a project gui and build a project.

# VI. Tips:
- Exclude 'main' from the build logic project,
- To enable C++ settings add <nature>org.eclipse.cdt.core.ccnature</nature> to .project.
- After generating the equipment using CubeMx, transfer the .cproject, .project, .mxproject files from the Atollic folder to the folder with the project name (AtollicTemplate). 
  - In the C/C++ Build->Settings->Settings->Tool Settings->C++ Linker->General settings we provide the path to the linker script,
  - In C/C++ Build-> Target Settings we choose the type of processor,
  - In C/C++ General->Paths and Symbols->Include, we provide the hardware paths updates.

# Police Scotland UAV - Firmware (Zephyr) Development Environment

This repo acts as the centre for all zephyr based firmware development for the UAV.

Just download the "Workbench for Zephyr" extension in VSCode, and "Initialize Workspace" and use this github repo when  it asks you for it.

To create a new app (new program) use the create app button in the Workbench menu. NOTE: After creating the app you must add ```list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..) ``` above "findpackage(...)" in the CMakeLists.txt file.
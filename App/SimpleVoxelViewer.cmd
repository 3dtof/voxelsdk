cd %~dp0
set VOXEL_SDK_PATH=%~dp0\..
set PATH=%~dp0;%~dp0\..\lib;%PATH%
set PYTHONPATH=%~dp0\..\lib\python2.7;%PYTHONPATH%
START /MAX cmd.exe /C "mode con cols=160 lines=500 && SimpleVoxelViewer.exe"
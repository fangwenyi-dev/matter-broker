@echo off
set IDF_PYTHON_ENV_PATH=D:\progrem\06290esp\Espressif\python_env\idf5.5_py3.11_env
set IDF_PATH=D:\progrem\06290esp\Espressif\frameworks\esp-idf-v5.5.4
set PATH=D:\progrem\06290esp\Espressif\python_env\idf5.5_py3.11_env\Scripts;D:\progrem\06290esp\Espressif\opt\xtensa-esp32s3-elf\bin;D:\progrem\06290esp\Espressif\tools\idf-exe\1.0.3;D:\progrem\06290esp\Espressif\tools\cmake\3.30.2\bin;D:\progrem\06290esp\Espressif\tools\ninja\1.12.1;D:\progrem\06290esp\Espressif\tools\idf-git\2.44.0\cmd;D:\progrem\06290esp\Espressif\tools\ccache\4.12.1\ccache-4.12.1-windows-x86_64;%PATH%
cd /d e:\AI\matter-broker\esp-mqttbroker\build
ninja all
echo NINJA_EXIT_CODE=%ERRORLEVEL%

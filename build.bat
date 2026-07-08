@echo off
set IDF_PYTHON_ENV_PATH=D:\progrem\06290esp\Espressif\python_env\idf5.5_py3.11_env
set IDF_PATH=D:\progrem\06290esp\Espressif\frameworks\esp-idf-v5.5.4
call D:\progrem\06290esp\Espressif\frameworks\esp-idf-v5.5.4\export.bat
cd /d e:\AI\matter-broker\esp-mqttbroker
idf.py build
echo BUILD_EXIT_CODE=%ERRORLEVEL%

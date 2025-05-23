$cmake = "C:\Program Files_Coding\CLion 2025.1.1\bin\cmake\win\x64\bin\cmake.exe"

& $cmake -S . -B build -A x64
& $cmake --build build --config Release

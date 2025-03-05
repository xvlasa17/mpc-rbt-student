LOG_LEVEL=0 cmake -S . -B build
LOG_LEVEL=1 cmake --build build
LOG_LEVEL=2 cmake --build build -t test

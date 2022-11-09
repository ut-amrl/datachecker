#acceptable buildTypes: Release/Debug/Profile
buildType=Release
# buildType=Debug

.SILENT:

all: compile_all

build/CMakeLists.txt.build: CMakeLists.txt
	mkdir -p build
	cd build && cmake -DCMAKE_BUILD_TYPE=$(buildType) ..
	cp CMakeLists.txt build/CMakeLists.txt.build

compile_all: build/CMakeLists.txt.build
	$(MAKE) --no-print-directory -C build

clean:
	rm -rf build
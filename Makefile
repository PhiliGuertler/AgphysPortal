all:
	mkdir -v -p build
	cd build; cmake ..; cd ..
	make -C build -j6
	./agphys

debug:
	mkdir -v -p build
	cd build; cmake ..; cd ..
	make -C build -j6
	gdb -ex run ./agphys

compile:
	mkdir -v -p build
	cd build; cmake ..; cd ..
	make -C build -j6

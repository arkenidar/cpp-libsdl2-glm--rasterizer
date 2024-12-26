all: build run

build:
	g++ rasterizer.cpp -o rasterizer $(shell pkg-config --cflags --libs sdl2)

run:
	./rasterizer test-cube.obj || ./rasterizer.exe test-cube.obj

wine:
	wine rasterizer.exe test-cube.obj

clean:
	rm -f rasterizer rasterizer.exe
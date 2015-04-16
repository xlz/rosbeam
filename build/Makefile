.PHONY: clean cleanall install all

CMAKE_FLAGS=-DBUILD_SHARED_LIBS:BOOL=OFF -DCMAKE_BUILD_TYPE=Release -DBoost_USE_STATIC_LIBS:BOOL=ON -DBoost_USE_STATIC_RUNTIME:BOOL=ON

SOURCES=../rosbeam/src/* ../rosbeam/*

DISTRO_SOURCES=indigo-minimal.rosinstall indigo-minimal.patch

all: install_isolated/lib/rosbeam/.stamp

python/.stamp:
	rm -rf python
	virtualenv python
	. python/bin/activate; pip install wstool rosdep empy
	touch $@

cache/.stamp: python/.stamp $(DISTRO_SOURCES)
	rm -rf cache
	. python/bin/activate; wstool init -j8 cache indigo-minimal.rosinstall
	touch $@

src/.stamp: cache/.stamp $(DISTRO_SOURCES)
	rm -rf src
	cp -a cache src
	patch -d src -p1 <indigo-minimal.patch
	touch $@

clean:
	rm -rf src *_isolated python

cleanall: clean
	rm -rf cache

install_isolated/.stamp: src/.stamp
	. python/bin/activate; ./src/catkin/bin/catkin_make_isolated --install --cmake-args $(CMAKE_FLAGS)
	touch $@

install_isolated/lib/rosbeam/.stamp: $(SOURCES) install_isolated/.stamp
	rm -rf src/rosbeam
	cp -a ../rosbeam src
	. python/bin/activate; ./src/catkin/bin/catkin_make_isolated --pkg rosbeam --install --cmake-args $(CMAKE_FLAGS)
	touch $@

install: install_isolated/lib/rosbeam/.stamp
	sudo -u '#1000' install -p -m 755 examples/platform.sh -D overlay/home/st/sw-dev/install/scripts/platform.sh
	install -p -o root -g root -m 755 examples/bridge.sh -D overlay/usr/bin/rosbeam-bridge.sh
	install -p -o root -g root -m 755 install_isolated/lib/rosbeam/bridge -D overlay/usr/bin/rosbeam-bridge
	install -p -o root -g root -m 4755 install_isolated/lib/rosbeam/libtexclient-inject.so -D overlay/usr/lib/x86_64-linux-gnu/libtexclient-inject.so
	tar -zcf overlay.tgz -C overlay .
	rm -rf overlay

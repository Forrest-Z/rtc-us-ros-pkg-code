all: cyton

TARBALL_32 = Robai-cyton-32.tar
TARBALL_64 = Robai-cyton-64.tar

UNPACK_CMD = tar xzf
PKG = $(shell rospack find cyton_driver)
ARCH = $(shell uname -m)

ifeq ($(ARCH),x86_64)
	tarball=$(TARBALL_64)
else
	tarball=$(TARBALL_32)
endif

cyton: $(result)
	tar -xvf $(PKG)/build/$(tarball) --directory=$(PKG)/build/uncompressed
	mkdir $(PKG)/lib
	cp $(PKG)/build/uncompressed/Cyton\ Viewer\ 3.0.20100623/lib/* $(PKG)/lib
	
	mkdir $(PKG)/include
	cp -R $(PKG)/build/uncompressed/Cyton\ Viewer\ 3.0.20100623/include/* $(PKG)/include

cyton_64:
	
clean: $(clean_result)
	rm -R $(PKG)/lib
	rm -R $(PKG)/include 
    
        
wipe: clean

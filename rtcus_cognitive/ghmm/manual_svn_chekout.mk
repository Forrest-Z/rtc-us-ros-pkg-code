all: installed

SVN_DIR = build/ghmm-svn
SVN_URL = https://ghmm.svn.sourceforge.net/svnroot/ghmm/trunk/ghmm
SVN_REVISION = -r 2290
SVN_PATCH = ghmm_python_setup.patch
include $(shell rospack find mk)/svn_checkout.mk

installed: $(SVN_DIR) 
	cd $(SVN_DIR) && ./autogen.sh
	cd $(SVN_DIR) && ./configure --prefix=$(shell rospack find ghmm)
	cd $(SVN_DIR) && make
	cd $(SVN_DIR) && make install
	touch installed
clean:
	-cd $(SVN_DIR) && make clean
	rm -rf stage installed patched
wipe: clean
	rm -rf $(SVN_DIR)
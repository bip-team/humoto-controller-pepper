TC?=mytoolchain

all: humoto

##################################################
# humoto
##################################################

humoto:
	git submodule update --init humoto;

##################################################
# OTHER
##################################################

controller-pepper: clean humoto
	qibuild configure -c ${TC}
	qibuild make -j4 -c ${TC}

doc:
	cd doc; ${MAKE}

doc-view:
	cd doc; ${MAKE} view

doc-clean:
	cd doc; ${MAKE} clean

clean:
	rm -Rf build-*
	rm -Rf controller/build-*

forceclean: clean
	rm -Rf ./humoto/

.PHONY: clean forceclean humoto doc

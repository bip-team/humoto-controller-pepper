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

clean:
	rm -Rf build-*
	rm -Rf controller/build-*

forceclean: clean
	rm -Rf ./humoto/

.PHONY: clean forceclean humoto

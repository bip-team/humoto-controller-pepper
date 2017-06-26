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
	qibuild configure
	qibuild make -j4

clean:
	rm -Rf build-*
	rm -Rf controller/build-*

forceclean: clean
	rm -Rf ./humoto/

.PHONY: clean forceclean humoto

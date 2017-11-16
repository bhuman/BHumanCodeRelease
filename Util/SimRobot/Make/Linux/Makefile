MARE := ../../Util/mare/Linux/bin/mare
CONFIG ?= Release
QTINCLUDE = `qtchooser -qt=qt5-x86_64-linux-gnu -print-env | grep QTTOOLDIR | sed "s%QTTOOLDIR=\"\(.*\)bin\"%\1%" | sed "s/^\/usr\/lib/\/usr\/include/"`

.PHONY: all clean

all:
	@$(MARE) config=$(CONFIG) qtinclude=$(QTINCLUDE)

clean:
	@$(MARE) clean config=$(CONFIG)

%:
	@$(MARE) $@ config=$(CONFIG) qtinclude=$(QTINCLUDE)


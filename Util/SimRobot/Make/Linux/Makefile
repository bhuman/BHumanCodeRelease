MARE := ../../Util/mare/Linux/bin/mare
CONFIG ?= Release
QTINCLUDE = `find /usr/include /usr/local/include -name qt5 -type d`

.PHONY: all clean

all:
	@$(MARE) config=$(CONFIG) qtinclude=$(QTINCLUDE)

clean:
	@$(MARE) clean config=$(CONFIG)

%:
	@$(MARE) $@ config=$(CONFIG) qtinclude=$(QTINCLUDE)


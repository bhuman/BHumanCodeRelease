MARE := ../../Util/mare/Linux/bin/mare
CONFIG ?= Release
QTINCLUDE = `for d in $(shell find /usr/include /usr/local/include -name qt5 \( -type l -or -type d \)); do [ -d $$d/QtCore ] && echo $$d; done | head -n1`

.PHONY: all clean

all:
	@$(MARE) config=$(CONFIG) qtinclude=$(QTINCLUDE)

clean:
	@$(MARE) clean config=$(CONFIG)

%:
	@$(MARE) $@ config=$(CONFIG) qtinclude=$(QTINCLUDE)


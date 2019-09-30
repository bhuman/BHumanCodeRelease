MARE := ../../Util/SimRobot/Util/mare/Linux/bin/mare
CONFIG ?= Develop
QTINCLUDE = `for d in $(shell find /usr/include /usr/local/include -name qt5 \( -type l -or -type d \)); do [ -d $$d/QtCore ] && echo $$d; done | head -n1`
CLANGINCLUDE = `for d in $(shell clang++ -c -v -E - </dev/null 2>&1 | sed -n -e '/here:/,/^End/{/here:/d; /^End/d; p}'); do [ -f $$d/xmmintrin.h ] && echo $$d; done | head -n1`
SSSE3 = `(grep ssse3 </proc/cpuinfo ; echo 'ssse3=false') | head -1 | sed 's/.*ssse3 .*/ssse3=true/'`
AVX2 = `(grep avx2 </proc/cpuinfo ; echo 'avx2=false') | head -1 | sed 's/.*avx2 .*/avx2=true/'`
FLAGS = $(SSSE3) $(AVX2)

.PHONY: all clean

all:
	@$(MARE) config=$(CONFIG) qtinclude=$(QTINCLUDE) clanginclude=$(CLANGINCLUDE) $(FLAGS)

clean:
	@$(MARE) clean config=$(CONFIG)

%:
	@$(MARE) $@ config=$(CONFIG) qtinclude=$(QTINCLUDE) clanginclude=$(CLANGINCLUDE) $(FLAGS)


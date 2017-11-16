MARE := ../../Util/SimRobot/Util/mare/Linux/bin/mare
CONFIG ?= Develop
QTINCLUDE = `qtchooser -qt=qt5-x86_64-linux-gnu -print-env | grep QTTOOLDIR | sed "s%QTTOOLDIR=\"\(.*\)bin\"%\1%" | sed "s/^\/usr\/lib/\/usr\/include/"`
SSSE3 = `(grep ssse3 </proc/cpuinfo ; echo 'ssse3=false') | head -1 | sed 's/.*ssse3 .*/ssse3=true/'`
AVX2 = `(grep avx2 </proc/cpuinfo ; echo 'avx2=false') | head -1 | sed 's/.*avx2 .*/avx2=true/'`
FLAGS = $(SSSE3) $(AVX2)

.PHONY: all clean

all:
	@$(MARE) config=$(CONFIG) qtinclude=$(QTINCLUDE) $(FLAGS)

clean:
	@$(MARE) clean config=$(CONFIG)

%:
	@$(MARE) $@ config=$(CONFIG) qtinclude=$(QTINCLUDE) $(FLAGS)


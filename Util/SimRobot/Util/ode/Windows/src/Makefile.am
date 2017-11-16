AUTOMAKE_OPTIONS = foreign
ACLOCAL_AMFLAGS = -I m4 --install

if ENABLE_OU
OU_DIR = ou
endif

if LIBCCD
if LIBCCD_INTERNAL
LIBCCD_DIR = libccd
endif
endif

if ENABLE_DEMOS
DRAWSTUFF_DIR = drawstuff
endif

if GIMPACT
GIMPACT_DIR = GIMPACT
endif

if OPCODE
OPCODE_DIR = OPCODE
endif

SUBDIRS = include \
          $(DRAWSTUFF_DIR) \
          $(GIMPACT_DIR) \
          $(OPCODE_DIR) \
          $(OU_DIR) \
          $(LIBCCD_DIR) \
          ode \
          tests

bin_SCRIPTS = ode-config

# Utility rule for making a release
release: dist-gzip dist-bzip2
	@echo Created release packages for ${PACKAGE}-${VERSION}.

EXTRA_DIST = bootstrap build tools \
        CHANGELOG.txt COPYING INSTALL.txt CSR.txt README.md \
        LICENSE.TXT LICENSE-BSD.TXT \
        bindings

pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = ode.pc

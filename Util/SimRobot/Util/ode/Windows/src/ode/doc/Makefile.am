EXTRA_DIST = Doxyfile.in main.dox pix

if HAVE_DOXYGEN

mostlyclean-local:
	rm -rf html

html-local: Doxyfile
	$(DOXYGEN) Doxyfile

endif

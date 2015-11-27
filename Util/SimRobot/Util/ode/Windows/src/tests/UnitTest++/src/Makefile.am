if WIN32
SUBDIRS = Win32
else
SUBDIRS = Posix
endif


check_LTLIBRARIES = libunittestpp.la

libunittestpp_la_SOURCES = AssertException.cpp       AssertException.h \
                            CheckMacros.h             Checks.cpp \
                            Checks.h                  Config.h \
                            DeferredTestReporter.cpp  DeferredTestReporter.h \
                            DeferredTestResult.cpp    DeferredTestResult.h \
                            MemoryOutStream.cpp       MemoryOutStream.h \
                            ReportAssert.cpp          ReportAssert.h \
                            Test.cpp                  TestDetails.cpp \
                            TestDetails.h             Test.h \
                            TestList.cpp              TestList.h \
                            TestMacros.h              TestReporter.cpp \
                            TestReporter.h            TestReporterStdout.cpp \
                            TestReporterStdout.h      TestResults.cpp \
                            TestResults.h             TestRunner.cpp \
                            TestRunner.h              TestSuite.h \
                            TimeConstraint.cpp        TimeConstraint.h \
                            TimeHelpers.h             UnitTest++.h \
                            XmlTestReporter.cpp       XmlTestReporter.h
if WIN32
libunittestpp_la_LIBADD = $(builddir)/Win32/libhelper.la
else
libunittestpp_la_LIBADD = $(builddir)/Posix/libhelper.la
endif


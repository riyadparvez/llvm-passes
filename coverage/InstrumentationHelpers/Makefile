
BCFILES = Instrumentation.bc

all: $(BCFILES)

$(BCFILES) : InstrumentationHelpers.cpp InstrumentationHelpers.h
	llvm-gcc -c -O2 -emit-llvm *.cpp
	llvm-link *.o -o Instrumentation.bc

.PHONY: clean
clean:
	rm *.o *.bc

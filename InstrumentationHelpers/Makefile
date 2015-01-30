
BCFILES = InstrumentationHelpers.o Patch.o

all : $(BCFILES)
$(BCFILES) : InstrumentationHelpers.cpp ../Patch.cpp
	llvm-gcc -O2 -c -emit-llvm *.cpp ../Patch.cpp
	llvm-link *.o -o Instrumentation.bc

.PHONY : clean
clean:
	rm *.o *.bc

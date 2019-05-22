UNAME := $(shell uname)
# The name of the output file 
OUTNAME=libBfbMessage.so
# List of source files
SRCCXX := BfbMessage.cpp

# Compiler flags that influence the behaviour of the c++ compiler
# -std=c++11 tells the compiler to use the c++11 standard. This standard includes for example shared pointers in the standard library. 
# -Wall tells the compiler to show most of the warnings (-Wextra will show even more). 
# -Wno-unused-parameter additionally 
# -O2 defines the opimization level the compiler should use (other options; -O, -O0, -O1, -O2, -O3, -Os). 
# -ggdb integrates debugging information in the executable that can be used by GDB. 
# -lm tells the compiler to include the math library. This enables the use of #include <math.h>
# -D_XOPEN_SOURCE=700 tells the compiler to add some additional functionality to interact with the operating system (if it is Linux or Mac)
# -g produces debugging information in the operating system's native format.
# -MD -MP generates the dependencies with the header files (ie. recompile if one changed)
CXXFLAGS=-std=c++11 -Wall -Wno-unused-parameter -O2 -D_XOPEN_SOURCE=700 -g -MD -MP

# Define the build directory
BUILDDIR=bin

# The c++ compiler that will be used to compile the source files.
CXX=g++

ifneq ($(UNAME), Darwin)
GCC_VER_GTE47 := $(shell echo `gcc -dumpversion | cut -f1-2 -d.` \>= 4.7 | sed -e 's/\./*100+/g' | bc )
ifeq ($(GCC_VER_GTE47),0)
        $(error The g++ version that is used as standard compiler is too old. At least version 4.7 must be used. You can check your version using the command "g++ --version". If you have a newer version installed but it is not your default compiler, you can change it using the script gcc-set-default-version from the hectorsim/Tools folder.)
endif
endif

# The libraries that should be included
# -lm tells the compiler to include the math library. This enables the use of #include <math.h>
# -lboost... includes certain libraries from the boost collection
# -pthread includes a library used for multithreading
# -lrt includes a realtime library
#LIBS= -lm -lboost_program_options -lboost_system -lboost_filesystem -lboost_regex -lboost_thread -pthread 
# 30 OCT 2013: here no linkage with used libs is required
# THIERRY'S JUSTIFICATION: because that's a shared lib being built, not an executable program
# ADVANTAGE: avoid caring about that, may accelerate the building process
LIBS = 

# Under MAC OS X the values have to be changed slightly
ifeq ($(UNAME), Darwin)
	OUTNAME := $(OUTNAME:%.so=%.dylib)
	CXXFLAGS=-std=c++11 -Wall -Wno-unused-parameter -O2 -g -MD -MP -stdlib=libc++ -D_DARWIN_C_SOURCE
        # The c++ compiler that will be used to compile the source files.
        CXX=clang++
        LIBS += -stdlib=libc++ -lboost_system
endif

# Replace all the "*.cpp"s (first line) and the "*.c"s (second line) in the source file list by "*.o"s and save the resulting list in new macro variables.
OBJSCXX := $(SRCCXX:%.cpp=${BUILDDIR}/%.o)

# This is a macro that will build an object file based on the c++ source file. 
# "$<" is the name of the first dependency (the "%.cpp" in the first line). 
# "$@" is the name of the object file. 
$(BUILDDIR)/%.o : %.cpp
	$(CXX) $(CXXFLAGS) -fPIC -c $< -o $@

# The executable is built. It depends on all of the object files. Therefore, if they aren't up-to-date, they will be compiled first using the compilation macros defined above.
all: $(BUILDDIR) $(OUTNAME)
$(BUILDDIR):
	mkdir $(BUILDDIR)
$(OUTNAME): $(SRCCXX:%.cpp=${BUILDDIR}/%.o)
	$(CXX) -o $@ $^ $(LIBS) -shared
-include $(SRCCXX:%.cpp=${BUILDDIR}/%.d)

EFFCPP: CXXFLAGS+=-Weffc++
EFFCPP: all

SWIG:   
	swig -c++ -python BfbMessage.i
	$(CXX) -std=c++11 -DNDEBUG -fPIC -O3 -c BfbMessage.cpp
	$(CXX) -std=c++11 -DNDEBUG -fPIC -O3 -c BfbMessage_wrap.cxx -I/usr/include/python3.2mu
	$(CXX) -std=c++11 -shared BfbMessage.o BfbMessage_wrap.o -o _BfbMessage.so -DNDEBUG -fPIC -O3

# All object files are deleted. 
clean:
	(cd $(BUILDDIR); rm -f *.o *.d)
	rm -f $(OUTNAME)

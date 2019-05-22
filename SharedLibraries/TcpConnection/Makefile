# The type of system this file is executed on (Mac/Linux/Windows)
UNAME := $(shell uname)

# The name of the output file
OUTNAME=libTcpConnection.so

# List of source files
SRCCXX := TcpConnection.cpp

# Path of the folder that contains all the sub-folders with the custom shared libraries (the ones written only for this project)
#CUSTOM_SHARED_LIB_DIR=../SharedLibraries
CUSTOM_SHARED_LIB_DIR=$(CURDIR)/..

# Names of the shared libraries used in this project
CUSTOM_SHARED_LIBS= BfbMessage

# The c++ compiler that will be used to compile the source files.
CXX=g++

ifneq ($(UNAME), Darwin)
GCC_VER_GTE47 := $(shell echo `gcc -dumpversion | cut -f1-2 -d.` \>= 4.7 | sed -e 's/\./*100+/g' | bc )
ifeq ($(GCC_VER_GTE47),0)
        $(error The g++ version that is used as standard compiler is too old. At least version 4.7 must be used. You can check your version using the command "g++ --version". If you have a newer version installed but it is not your default compiler, you can change it using the script gcc-set-default-version from the hectorsim/Tools folder.)
endif
endif

# Define the build directory
BUILDDIR=bin

# Compiler flags that influence the behaviour of the c++ compiler
# -std=c++11 tells the compiler to use the c++11 standard. This standard includes for example shared pointers in the standard library. 
# -Wall tells the compiler to show most of the warnings (-Wextra will show even more). 
# -O2 defines the opimization level the compiler should use (other options; -O, -O0, -O1, -O2, -O3, -Os). 
# -g produces debugging information in the operating system's native format.
# -MD -MP generates the dependencies with the header files (ie. recompile if one changed)
CXXFLAGS=-std=c++11 -Wall -O2 -g -MD -MP

# Preprocessor flags
CPPFLAGS=$(addprefix -I${CUSTOM_SHARED_LIB_DIR}/, ${CUSTOM_SHARED_LIBS})

# The libraries that should be included
# -lm tells the compiler to include the math library. This enables the use of #include <math.h>
# -lboost... includes certain libraries from the boost collection
# -pthread includes a library used for multithreading
LDLIBS= $(addprefix -l, ${CUSTOM_SHARED_LIBS})  -lm -lboost_program_options -lboost_system -lboost_filesystem -lboost_regex -lboost_thread -pthread 

TEMP=-Wl,-rpath=${CUSTOM_SHARED_LIB_DIR}/
LDFLAGS=$(addprefix ${TEMP}, ${CUSTOM_SHARED_LIBS})\
        $(addprefix -L${CUSTOM_SHARED_LIB_DIR}/, ${CUSTOM_SHARED_LIBS})

# The type of system this file is executed on (Mac/Linux/Windows)
UNAME := $(shell uname)
# Under MAC OS X the values have to be changed slightly
ifeq ($(UNAME), Darwin)
	OUTNAME := $(OUTNAME:%.so=%.dylib)
        CXXFLAGS += -stdlib=libc++ -D_DARWIN_C_SOURCE
        # The c++ compiler that will be used to compile the source files.
        CXX=clang++
        LDLIBS += -stdlib=libc++
        LDFLAGS=$(${CUSTOM_SHARED_LIBS})\
        $(addprefix -L${CUSTOM_SHARED_LIB_DIR}/, ${CUSTOM_SHARED_LIBS})
endif

# This is the default target that will be built for a call of "make". 
all: $(CUSTOM_SHARED_LIBS) $(BUILDDIR) $(OUTNAME)
$(BUILDDIR):
	mkdir $(BUILDDIR)

# Link the object files into one shared library
# This target depends on the object files of the individual *.cpp files. Therefore, in the first line (the dependency line), the list of *.cpp files is converted into a list of all *.o files. 
# "$^" is the names of all the dependencies separated by spaces, but with duplicate names removed. 
# "$@" is the file name of the target. 
$(OUTNAME): $(SRCCXX:%.cpp=${BUILDDIR}/%.o)
	$(CXX) -o $@ $^ $(LDFLAGS) $(LDLIBS) -shared

ifeq ($(UNAME), Darwin)
#For MAC OS X the dylib relative load path has to be hand tuned
	install_name_tool -change libBfbMessage.dylib "@loader_path/../BfbMessage/libBfbMessage.dylib" libTcpConnection.dylib
endif

# This is a macro that will build an object file based on the c++ source file. 
# "$<" is the name of the first dependency (the "%.cpp" in the first line). 
# "$@" is the name of the object file. 
$(BUILDDIR)/%.o : %.cpp
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) -fPIC -c $< -o $@

-include $(SRCCXX:%.cpp=${BUILDDIR}/%.d)

EFFCPP: CXXFLAGS+=-Weffc++
EFFCPP: all

# All object files are deleted. 
clean:
	rm -f $(BUILDDIR)/*.o $(BUILDDIR)/*.d
	rm -f $(OUTNAME)

#### SHARED LIBRARIES #########################################
cleanall: clean
	make clean -C $(CUSTOM_SHARED_LIB_DIR)/BfbMessage

BfbMessage:
	make -C $(CUSTOM_SHARED_LIB_DIR)/BfbMessage
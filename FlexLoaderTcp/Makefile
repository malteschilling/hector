# The type of system this file is executed on (Mac/Linux/Windows)
UNAME := $(shell uname)

# The name of the output file 
OUTNAME=FlexLoaderTcp

# Path of the folder that contains all the sub-folders with the custom shared libraries (the ones written only for this project)
CUSTOM_SHARED_LIB_DIR=../SharedLibraries

# Names of the shared libraries used in this project
CUSTOM_SHARED_LIBS= BfbMessage\
                    TcpConnection\
                    TcpServer


# Compiler flags that influence the behaviour of the c++ compiler
# -std=c++11 tells the compiler to use the c++11 standard. This standard includes for example shared pointers in the standard library. 
# -Wall tells the compiler to show most of the warnings (-Wextra will show even more). 
# -Wno-unused-parameter additionally 
# -O2 defines the opimization level the compiler should use (other options; -O, -O0, -O1, -O2, -O3, -Os). 
# -ggdb integrates debugging information in the executable that can be used by GDB. 
# -lm tells the compiler to include the math library. This enables the use of #include <math.h>
# -D_XOPEN_SOURCE=700 tells the compiler to add some additional functionality to interact with the operating system (if it is Linux or Mac)
# -g produces debugging information in the operating system's native format.
CXXFLAGS=-std=c++11 -Wall -Wno-unused-parameter -O2 -ggdb -lm -D_XOPEN_SOURCE=700 -g 

# Create references to the shared libraries for the linker 
INCLUDEPATHS =  $(addprefix -I${CUSTOM_SHARED_LIB_DIR}/, ${CUSTOM_SHARED_LIBS})\
                $(addprefix -L${CUSTOM_SHARED_LIB_DIR}/, ${CUSTOM_SHARED_LIBS})
# Create run time references to the shared libraries
# The variable "LDFLAGSPREFIX" is needed since the interpreter would - if the string was used directly in LDFLAGS - interpret the comma between "-Wl" and "-rpath" as separator
ifneq ($(UNAME), Darwin)
	LDFLAGSPREFIX=-Wl,-rpath=${CUSTOM_SHARED_LIB_DIR}/
	LDFLAGS= $(addprefix $(LDFLAGSPREFIX), ${CUSTOM_SHARED_LIBS})
endif
# Define the build directory
BUILDDIR=bin

# The c++ compiler that will be used to compile the source files.
CXX=g++

GCC_VER_GTE47 := $(shell echo `gcc -dumpversion | cut -f1-2 -d.` \>= 4.7 | sed -e 's/\./*100+/g' | bc )
ifeq ($(GCC_VER_GTE47),0)
        $(error The g++ version that is used as standard compiler is too old. At least version 4.7 must be used. You can check your version using the command "g++ --version". If you have a newer version installed but it is not your default compiler, you can change it using the script gcc-set-default-version from the hectorsim/Tools folder.)
endif

# The libraries that should be included
# -lm tells the compiler to include the math library. This enables the use of #include <math.h>
# -lboost... includes certain libraries from the boost collection
# -pthread includes a library used for multithreading
# -lrt includes a realtime library
LIBS=  -lTcpServer -lBfbMessage -lm -lboost_program_options  -lboost_system -lboost_filesystem -lboost_regex -lboost_thread  -lpthread

# Under MAC OS X the values have to be changed slightly
ifeq ($(UNAME), Darwin)
	CXXFLAGS += -stdlib=libc++ -D_DARWIN_C_SOURCE
    CXXFLAGS=-std=c++11 -stdlib=libc++ -Wall -Wno-unused-parameter -O2 -ggdb -D_DARWIN_C_SOURCE -g
    # The c++ compiler that will be used to compile the source files.
    CXX=clang++
	LIBS += -stdlib=libc++ -headerpad_max_install_names -L/usr/X11/lib
endif

# List of source files
SRCCXX := main.cpp\
	FlexLoader.cpp
         

# Replace all the "*.cpp"s (first line) and the "*.c"s (second line) in the source file list by "*.o"s and save the resulting list in new macro variables.
OBJSCXX := $(SRCCXX:%.cpp=${BUILDDIR}/%.o)

# This is a macro that will build an object file based on the c++ source file. 
# "$<" is the name of the first dependency (the "%.cpp" in the first line). 
# "$@" is the name of the object file. 
$(BUILDDIR)/%.o : %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDEPATHS) -c $< -o $@


# The executable is built. It depends on all of the object files. Therefore, if they aren't up-to-date, they will be compiled first using the compilation macros defined above.
all: ${OBJSCXX}
	$(CXX) $(LDFLAGS) $(INCLUDEPATHS) -o $(OUTNAME) $? $(LIBS)
ifeq ($(UNAME), Darwin)
	install_name_tool -change libBfbMessage.dylib "$(CUSTOM_SHARED_LIB_DIR)/BfbMessage/libBfbMessage.dylib" BioFlexServer
	install_name_tool -change libTcpServer.dylib "$(CUSTOM_SHARED_LIB_DIR)/TcpServer/libTcpServer.dylib" BioFlexServer
endif
	
EFFCPP: CXXFLAGS+=-Weffc++
EFFCPP: all

# All object files are deleted. 
clean:
	rm -f $(BUILDDIR)/*.o


# The type of system this file is executed on (Mac/Linux/Windows)
UNAME := $(shell uname)
# The name of the output file 
OUTNAME=RobotSim
# List of source files
SRCCXX :=       BfbClient.cpp\
                BfbMessageProcessor.cpp\
                BioFlexRotatory.cpp\
                Bodies.cpp\
                GeometricPrimitives.cpp\
                GeometryXmlParser.cpp\
                HelperFunctions.cpp\
                Imu.cpp\
                Joints.cpp\
                main.cpp\
                PressureSensor.cpp\
                Universe.cpp\

# Path of the folder that contains all the sub-folders with the custom shared libraries (the ones written only for this project)
CUSTOM_SHARED_LIB_DIR=../SharedLibraries
# Names of the shared libraries used in this project
CUSTOM_SHARED_LIBS= ode\
                    BfbMessage\
                    PugiXml\
                    TcpConnection\
                    TcpServer

# Compiler flags that influence the behaviour of the c++ compiler
# -std=c++11 tells the compiler to use the c++11 standard. This standard includes for example shared pointers in the standard library. 
# -Wall tells the compiler to show most of the warnings (-Wextra will show even more). 
# -Wno-unused-parameter additionally 
# -O2 defines the opimization level the compiler should use (other options; -O, -O0, -O1, -O2, -O3, -Os). 
# -ggdb integrates debugging information in the executable that can be used by GDB. 
# -Ddouble tells the compiler that a define named "double" is present. In this particular case, it tells ODE to use double precision. 
# -lm tells the compiler to include the math library. This enables the use of #include <math.h>
# -D_XOPEN_SOURCE=700 tells the compiler to add some additional functionality to interact with the operating system (if it is Linux or Mac)
# -g produces debugging information in the operating system's native format.
CXXFLAGS=-std=c++11 -Wall -Wno-unused-parameter -O2 -D_XOPEN_SOURCE=700 -g -MD -MP

# Create references to the shared libraries for the linker 
INCLUDEPATHS =	$(addprefix -I${CUSTOM_SHARED_LIB_DIR}/, ${CUSTOM_SHARED_LIBS})\
$(addprefix -L${CUSTOM_SHARED_LIB_DIR}/, ${CUSTOM_SHARED_LIBS})

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
# -lode includes the ode library. 
# -ldrawstuff includes the drawstuff library that can be used for visualisation. 
# -lGL includes the OpenGL library needed by drawstuff. 
# -lGLU includes the OpenGL utility library needed by drawstuff. 
# -lm tells the compiler to include the math library. This enables the use of #include <math.h>
# -lboost... includes certain libraries from the boost collection
# -pthread includes a library used for multithreading
LIBS= -lTcpServer -lTcpConnection -lBfbMessage -lPugiXml -lode -ldrawstuff -lGL -lGLU -lm -lboost_program_options  -lboost_system -lboost_thread -pthread

# Create run time references to the shared libraries
# The variable "LDFLAGSPREFIX" is needed since the interpreter would - if the string was used directly in LDFLAGS - interpret the comma between "-Wl" and "-rpath" as separator
ifneq ($(UNAME), Darwin)
        LDFLAGSPREFIX=-Wl,-rpath=${CUSTOM_SHARED_LIB_DIR}/
        LDFLAGS= $(addprefix $(LDFLAGSPREFIX), ${CUSTOM_SHARED_LIBS})
endif
# Under MAC OS X the values have to be changed slightly
ifeq ($(UNAME), Darwin)
	CXXFLAGS += -stdlib=libc++ -D_DARWIN_C_SOURCE
    CXXFLAGS=-std=c++11 -stdlib=libc++ -Wall -Wno-unused-parameter -O2 -D_DARWIN_C_SOURCE -g -MD -MP
    # The c++ compiler that will be used to compile the source files.
    CXX=clang++
	LIBS += -stdlib=libc++ -lboost_system -headerpad_max_install_names -L/usr/X11/lib
endif

# Replace all the "*.cpp"s (first line) and the "*.c"s (second line) in the source file list by "*.o"s and save the resulting list in new macro variables.
OBJSCXX := $(SRCCXX:%.cpp=${BUILDDIR}/%.o)

# This is a macro that will build an object file based on the c++ source file. 
# "$<" is the name of the first dependency (the "%.cpp" in the first line). 
# "$@" is the name of the object file. 
$(BUILDDIR)/%.o : %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDEPATHS) -c $< -o $@ 

ODEDIR = $(CUSTOM_SHARED_LIB_DIR)/ode

# The executable is built. It depends on all of the object files. Therefore, if they aren't up-to-date, they will be compiled first using the compilation macros defined above.
all: $(CUSTOM_SHARED_LIBS) $(BUILDDIR) $(OUTNAME)
$(BUILDDIR):
	mkdir $(BUILDDIR)

$(OUTNAME): $(SRCCXX:%.cpp=${BUILDDIR}/%.o)
	$(CXX)  $(LDFLAGS) $(INCLUDEPATHS) -o $@ $^  $(LIBS)
ifeq ($(UNAME), Darwin)
	install_name_tool -id libode.dylib $(ODEDIR)/libode.dylib
	install_name_tool -id libdrawstuff.dylib $(ODEDIR)/libdrawstuff.dylib
	install_name_tool -change libBfbMessage.dylib "$(CUSTOM_SHARED_LIB_DIR)/BfbMessage/libBfbMessage.dylib" RobotSim
	install_name_tool -change libPugiXml.dylib "$(CUSTOM_SHARED_LIB_DIR)/PugiXml/libPugiXml.dylib" RobotSim
	install_name_tool -change libTcpServer.dylib "$(CUSTOM_SHARED_LIB_DIR)/TcpServer/libTcpServer.dylib" RobotSim
	install_name_tool -change libTcpConnection.dylib "$(CUSTOM_SHARED_LIB_DIR)/TcpConnection/libTcpConnection.dylib" RobotSim
	install_name_tool -change libode.dylib "$(ODEDIR)/libode.dylib" RobotSim
	install_name_tool -change libdrawstuff.dylib "$(ODEDIR)/libdrawstuff.dylib" RobotSim
endif
-include $(SRCCXX:%.cpp=${BUILDDIR}/%.d)

EFFCPP: CXXFLAGS+=-Weffc++
EFFCPP: all

# All object files are deleted. 
clean:
	@(cd $(BUILDDIR); rm -f *.o *.d)
	rm -f $(OUTNAME)

#### SHARED LIBRARIES #########################################

ODEDIR = $(CUSTOM_SHARED_LIB_DIR)/ode
ABSODEDIR = $(realpath $(ODEDIR))
ODELIB = $(ODEDIR)/libode.so
DRAWSTUFFLIB = $(ODEDIR)/libdrawstuff.so 

ifeq ($(UNAME), Darwin)
	ODELIB := $(ODELIB:%.so=%.dylib)
	DRAWSTUFFLIB := $(DRAWSTUFFLIB:%.so=%.dylib)
endif

cleanall: clean
	(cd $(CUSTOM_SHARED_LIB_DIR)/BfbMessage; make clean)
	(cd $(CUSTOM_SHARED_LIB_DIR)/TcpConnection; make clean)
	(cd $(CUSTOM_SHARED_LIB_DIR)/TcpServer; make clean)
	(cd $(ODEDIR); make uninstall; make clean)

ode: $(ODELIB) $(DRAWSTUFFLIB) 
BfbMessage:
	(cd $(CUSTOM_SHARED_LIB_DIR)/BfbMessage; make)
PugiXml: 
	make -C $(CUSTOM_SHARED_LIB_DIR)/PugiXml
TcpConnection: 
	(cd $(CUSTOM_SHARED_LIB_DIR)/TcpConnection; make)
TcpServer: 
	(cd $(CUSTOM_SHARED_LIB_DIR)/TcpServer; make)

$(ODELIB): $(ODEDIR)/Makefile
	(cd $(ODEDIR); make; make install)

$(ODEDIR)/Makefile:
	(cd $(ODEDIR); ./configure --disable-demos --enable-shared --disable-static --with-drawstuff=X11 --enable-double-precision --enable-libccd --with-trimesh=opcode --prefix=$(ABSODEDIR)/install --libdir=$(ABSODEDIR) --includedir=$(ABSODEDIR))
	
$(DRAWSTUFFLIB): $(ODEDIR)/Makefile
	(cd $(ODEDIR)/drawstuff/src; make drawstuff.lo)
	(cd $(ODEDIR)/drawstuff/src; make x11.lo)
	(cd $(ODEDIR)/drawstuff/src; /bin/bash ../../libtool --tag=CXX   --mode=link g++  -g -O2 -version-info 1:0:0 -rpath $(ABSODEDIR) -o libdrawstuff.la  drawstuff.lo  x11.lo -L/opt/X11/lib -lX11   -lm  -lpthread)
	(cd $(ODEDIR)/drawstuff/src; /bin/bash ../../libtool --mode=install /usr/bin/install -c   libdrawstuff.la $(ABSODEDIR))
	(cd $(ODEDIR)/drawstuff/src; /bin/bash ../../libtool --finish $(ABSODEDIR))
	(cd $(ODEDIR); cp include/drawstuff/*.h drawstuff)

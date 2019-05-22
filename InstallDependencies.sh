# This will install python 3 development packages
sudo apt-get install python3-all-dev python3-numpy python3-scipy python3-tk python3-pip --assume-yes

# This will install a python library that produces Matlab-like plots
sudo apt-get install python3-matplotlib --assume-yes

# This will install a text to speech module
sudo apt-get install speech-dispatcher python3-speechd --assume-yes

# This will install the zero mq library for python3
sudo pip3 install pyzmq

# This will install the message pack library for python3
sudo pip3 install msgpack-python

# This will install matplotlib, a library that produces matlab-like plots
sudo pip3 install matplotlib

# This will install pkg-config that is used to get the name of the python header that is needed for the communication interface
sudo apt-get install pkg-config --assume-yes

# This will install gcc and g++ (the c & c++ compiler)
sudo apt-get install build-essential --assume-yes

# This will install the GL und GLU libraries needed by Drawstuff
sudo apt-get install libgl1-mesa-dev libglu1-mesa-dev --assume-yes

# This will install the boost library (all of it)
sudo apt-get install libboost-all-dev --assume-yes

# This will install swig (used to connect the python-part to the c++-part)
sudo apt-get install swig --assume-yes

# This will install automake (needed to configure ode)
sudo apt-get install automake --assume-yes

# This will install mercurial (this is an alternative to git that the developers of ode use)
sudo apt-get install mercurial --assume-yes

# This will install libtool. libtoolize is required by the bootstrap-command of ode
sudo apt-get install libtool --assume-yes
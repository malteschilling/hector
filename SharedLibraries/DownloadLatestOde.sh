# Delete the old ode folder
rm -rf ./ode

# Clone the mercurial repository to the folder "ode"
hg clone https://bitbucket.org/odedevs/ode ./ode

# Run the bootstrapper in the ode directory
(cd ./ode; ./bootstrap)



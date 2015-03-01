#iAnt-ARGoS

ARGoS (Autonomous Robots Go Swarming) is a multi-physics robot simulator. iAnt-ARGoS is an extension to ARGoS that implements the iAnt CPFA algorithm and provides a mechanism for performing experiments with iAnts.

###Quick Start Installation Guide

In order to use the iAnt CPFA in ARGoS, you must first install ARGoS on your system then download and compile the code in this repo to run with ARGoS.

#####1. Installing ARGoS

ARGoS is available for Linux and Macintosh systems. It is currently not supported on Windows. Detailed installation instructions can be found on the ARGoS website.
```
http://www.argos-sim.info/user_manual.php
```

######Linux Installation

1. Download the appropriate binary package for your Linux system.
  * http://www.argos-sim.info/core.php
2. In Terminal, run the following command in the directory of your installation file:
  * For Ubuntu and KUbuntu:
    - $ sudo dpkg -i argos3_simulator-*.deb
  * For OpenSuse:
    - $ sudo rpm -i argos3_simulator-*.rpm

######Macintosh Installation

1. For quick and easy installation, the Mac OSX installation of ARGoS requires Homebrew.
  * If you don't have it, install the Homebrew Package Manager by using the following command in Terminal.
    - $ ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
2. Obtain the Homebrew Tap for ARGoS using the following command in Terminal.
  * $ brew tap ilpincy/argos3
3. Once tapped, install ARGoS with the following command in Terminal. ARGoS and its required dependencies will be downloaded and installed using Homebrew.
  * $ brew install bash-completion qt lua argos3
4. Once installed, you can update ARGoS with the following two commands in Terminal.
  * $ brew update
  * $ brew upgrade argos3

#####2. Compiling and Running the iAnt CPFA in ARGoS

TODO

###Useful Links

* official ARGoS website and documentation
  * http://www.argos-sim.info/
* homebrew utility for Mac OSX installations
  * http://brew.sh/
* cmake utility information
  * http://www.cmake.org/documentation/

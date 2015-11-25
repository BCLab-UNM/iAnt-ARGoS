#iAnt-ARGoS

ARGoS (Autonomous Robots Go Swarming) is a multi-physics robot simulator. iAnt-ARGoS is an extension to ARGoS that implements the iAnt CPFA algorithm and provides a mechanism for performing experiments with iAnts.

**NOTE:** ARGoS is installed on the CS Linux machines. If you wish to use ARGoS on those machines, you only need to [download and compile](https://github.com/BCLab-UNM/iAnt-ARGoS#2-compiling-and-running-the-iant-cpfa-in-argos) this repository's code.

For more detailed information, please check the [iAnt-ARGoS wiki](https://github.com/BCLab-UNM/iAnt-ARGoS/wiki).

###Quick Start Installation Guide

In order to use the iAnt CPFA in ARGoS, you must first install ARGoS on your system then download and compile the code in this repo to run with ARGoS.

#####1. Installing ARGoS

ARGoS is available for Linux and Macintosh systems. It is currently not supported on Windows. Detailed installation instructions can be found on the [ARGoS Website](http://www.argos-sim.info/user_manual.php).

######Linux Installation

1. [Download](http://www.argos-sim.info/core.php) the appropriate binary package for your Linux system.
2. In Terminal, run the following command in the directory of your installation file:
  * for Ubuntu and KUbuntu:
    ```
    $ sudo dpkg -i argos3_simulator-*.deb
    ```

  * for OpenSuse:
    ```
    $ sudo rpm -i argos3_simulator-*.rpm
    ```

######Macintosh Installation

1. The Mac OSX installation of ARGoS uses the Homebrew Package Manager. If you don't have it, install Homebrew by using the following command in Terminal.
  ```
  $ ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
  ```

2. Obtain the Homebrew Tap for ARGoS using the following command in Terminal.
  ```
  $ brew tap ilpincy/argos3
  ```

3. Once tapped, install ARGoS with the following command in Terminal. ARGoS and its required dependencies will be downloaded and installed using Homebrew.
  ```
  $ brew install bash-completion qt lua argos3
  ```

4. Once installed, you can update ARGoS with the following two commands in Terminal.
  ```
  $ brew update
  $ brew upgrade argos3
  ```

#####2. Compiling and Running the iAnt CPFA in ARGoS

Once ARGoS is installed on your system. You can download the files in this repo, compile them for your system, and run the iAnt CPFA in ARGoS.

1. [Download](https://github.com/BCLab-UNM/iAnt-ARGoS/archive/master.zip) the iAnt-ARGoS files and unzip the folder in a directory of your choice.

2. From the Terminal, use the following commands to compile the iAnt CPFA code:
  ```
  $ cd iAnt-ARGoS-master  # go into the iAnt-ARGoS directory
  $ cd build              # go into the build directory
  $ cmake ..              # setup compilation with cmake
  $ make                  # compile the iAnt CPFA code
  $ cd ..                 # get out of the build directory
  ```

3. Launch ARGoS with the XML configuration file for your system:
  * for Linux systems:
    ```
    $ argos3 -c experiments/iAnt_linux.argos
    ```

  * for Mac systems:
    ```
    $ argos3 -c experiments/iAnt_mac.argos
    ```

###Useful Links

| Description                                 | Website                             |
|:--------------------------------------------|:------------------------------------|
| official ARGoS website and documentation    | http://www.argos-sim.info/          |
| homebrew utility for Mac OSX installations  | http://brew.sh/                     |
| cmake utility information                   | http://www.cmake.org/documentation/ |

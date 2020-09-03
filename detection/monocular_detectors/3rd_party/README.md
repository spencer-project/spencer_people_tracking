# libcudaHOG 
This library is provided by the RWTH Aachen and will be downloaded from http://www.vision.rwth-aachen.de/projects/

This directory contains a cmake wrapper file that will take care of downloading, unpacking, building and installing the tar-ball. It will also create a pkg-config file to allow cmake to find it in other projects.
At the moment this library is only needed to build the ground_hog package.

## Dependencies
* NVIDIA - CUDA Toolkit: Please follow instructions at https://developer.nvidia.com/cuda-downloads to find the latest version (tested with version 11.0 under Ubuntu 20.04 Focal)
	* This requires a NVIDIA graphics card
	* Under more recent Ubuntu versions, you can easily install the CUDA Toolkit via apt-get: e.g. `apt-get install cuda-toolkit-11-0` on Ubuntu Focal (20.04). Try to match the version of the toolkit to any other CUDA libraries you have already installed via the same method.
	* If installing by hand, make sure to follow the instructions especially the part about exporting the PATH and LD_LIBRARY_PATH variables. Add these statements to your `.bashrc`.
* qmake (Qt5)

## Installation
As mentioned the cmake file will take care of almost everything. Just follow these simple instructions:
* Change into the `rwth_perception_people/3rd_party` directory
* Create a build directory to keep it clean: `mkdir build; cd build`
* Run cmake: `cmake ..` _This will install everything to /usr/local and requires sudo rights._
	* To install it in a custom location: `cmake .. -DCMAKE_INSTALL_PREFIX=/my/path/`
	* If you choose to install it in a custom location, you have to make sure that pkg-config finds it: `export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:/my/path/lib/pkgconfig` _You can add this to your .bashrc because other projects may need to find it during compile time. If you installed the library to `/usr/local`, you do not have to export any paths._
* Run make: `make`. _This will download, unpack and build the files._
* Install library and headers: `sudo make install` _Omit the sudo if you chose a custom destination that does not require sudo rights._

## Troubleshooting
* If you are on a 32bit system, you will get an error when linking the library: `/usr/bin/ld: cannot find -lcudart`. To get rid of this you have to:
	* Edit the `build/libcudaHOG/src/libcudaHOG/cudaHOG/cudaHOG.pro` file. _Note: you have to run make before doing so because this will download and unzip the tarball. Otherwise you do not have that file._ 
	* Find the line that says: `LIBS += -lcudart -L/usr/local/cuda/lib64` and change it to: `LIBS += -lcudart -L/usr/local/cuda/lib`.
	* Now run `make` again.

* If there is an error "undefined reference to 'QString::fromAscii_helper" while running "make", edit "build/libcudaHOG/src/libcudaHOG/cudaHOG.pro" and remove the lines with the "cudaHOGDetect" and "cudaHOGDump" subdirs! (NOTE: This is now automatically patched by CMakeLists.txt).

* If there is an error "/usr/bin/ld: cannot find -lboost_program_options-mt", please make sure you have "libboost_program_options*.*" in your "/usr/lib" directory (use command `locate libboost_program_options`). If you have not yet installed Boost, you can try command `sudo apt-get install libboost_program_options-dev`. Else if you have "libboost_program_options*.*", change 'boost_program_options-mt' to 'boost_program_options' in your Makefile and other files (use command `grep 'boost_program_options-mt' -nr` in the build dir to find these files) (NOTE: This is now automatically done through the script fix_boost_dependency.sh, which is executed as part of CMakeLists.txt.)

* If there is an error "nvcc fatal   : Value 'sm_11' is not defined for option 'gpu-architecture'", please make sure the CUDA SDK has been installed and use the command `nvcc --help|grep "Allowed values for this option" -n` to see which gpu architecture is supported (eg. 'compute_20' or 'sm_20'), and change 'sm_11' to others in the files (use command `grep 'sm_11' -nr` in the build dir to find these files) (NOTE: CMakeLists.txt will now automatically apply a patch to change this to sm_30 in order to avoid this error. You can modify the patch in cudaHOG.pro.diff if you need a different architecture.)

* In case of CUDA Error 999 when launching sample applications from the CUDA SDK, this might be a permissions problem! Try if the samples work when run via sudo. In that case, a dirty workaround is to call one of the applications (e.g. deviceQueryDrv) once in an /etc/init/ script at login-session-start.


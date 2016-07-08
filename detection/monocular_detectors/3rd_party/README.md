# libcudaHOG 
This library is provided by the RWTH Aachen and will be downloaded from http://www.vision.rwth-aachen.de/projects/

This directory contains a cmake wrapper file that will take care of downloading, unpacking, building and installing the tar-ball. It will also create a pkg-config file to allow cmake to find it in other projects.
At the moment this library is only needed to build the ground_hog package.

## Dependencies
* NVIDIA - CUDA: Please follow instructions: http://developer.download.nvidia.com/compute/cuda/5_5/rel/docs/CUDA_Getting_Started_Linux.pdf or go directly to: https://developer.nvidia.com/cuda-downloads to find the latest version (tested with version 5.5)
	* This requires a NVIDIA graphics card
	* Make sure to follow the instructions especially the part about exporting the PATH and LD_LIBRARY_PATH variables. Add these statements to your `.bashrc`.
* qmake (Qt4)

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

## FURTHER CUDA SDK INSTALLATIONS NOTES FOR UBUNTU 12.04
* Use SDK 5.5. Versions 6.0 or 6.5 will not work, as they require a newer graphics driver which is not available for 12.04.
* CORRECTION: Version 6.5 works when removing all nvidia-* drivers and installing the 340 driver supplied with the SDK. Tested on 3.13 kernel. Requires the driver to be re-installed each time a new kernel update is installed.
* If laptop has Optimus, disable it in the BIOS and set to "discrete graphics"! With Optimus, it is hard to get even the CUDA samples running!
* Install the graphics driver "nvidia-331-updates" using the "Additional drivers" panel in the system settings GUI.
* After restart, make sure that the driver is actually in use. If it is activated but not in use according to GUI, make sure it is not blacklisted by checking "sudo modprobe nvidia". Also, running "nvidia-settings" should show info such as GPU frequency, temperature etc.
* Then install the CUDA SDK 5.5, but when asked if the graphics driver shall be installed, answer "no".
* vim /etc/ld.so.conf.d/cuda.conf, paste following lines:
    /usr/local/cuda-5.5/lib64
    /usr/local/cuda-5.5/lib
    /usr/lib/nvidia-331-updates/
* Then run "sudo ldconfig", and next run:
* cd /usr/lib && sudo ln -s /usr/lib/nvidia-331-updates/libcuda.so libcuda.so
* To verify, cd ~/NVIDIA_CUDA-5.5_Samples/NVIDIA_CUDA-5.5_Samples/1_Utilities/deviceQueryDrv/ && make && ../../bin/x86_64/linux/release/deviceQueryDrv.exe
* export PATH=/usr/local/cuda/bin:$PATH

Now perform installation as described above. If there is an error like "missing reference to __cudaInitModule", make sure CUDA SDK version is correct and remove the "build" folder, re-create it & run cmake again! It seems the nvcc compiler is run only once, and if you initially have got a wrong CUDA version, the libcudaHOG.so file is never re-built if you do not first delete the "build" folder!

## IMPORTANT!

* If there is an error "undefined reference to 'QString::fromAscii_helper" while running "make", edit "build/libcudaHOG/src/libcudaHOG/cudaHOG.pro" and remove the lines with the "cudaHOGDetect" and "cudaHOGDump" subdirs!

* If there is an error "/usr/bin/ld: cannot find -lboost_program_options-mt", please make sure you have "libboost_program_options*.*" in your "/usr/lib" directory (use command `locate libboost_program_options`). If you have not yet installed Boost, you can try command `sudo apt-get install libboost_program_options-dev`. Else if you have "libboost_program_options*.*", change 'boost_program_options-mt' to 'boost_program_options' in your Makefile and other files (use command `grep 'boost_program_options-mt' -nr` in the build dir to find these files)

* If there is an error "nvcc fatal   : Value 'sm_11' is not defined for option 'gpu-architecture'", please make sure the CUDA SDK has been installed and use the command `nvcc --help|grep "Allowed values for this option" -n` to see which gpu architecture is supported (eg. 'compute_20' or 'sm_20'), and change 'sm_11' to others in the files (use command `grep 'sm_11' -nr` in the build dir to find these files)

* In case of CUDA Error 999 when launching sample applications from the CUDA SDK, this might be a permissions problem! Try if the samples work when run via sudo. In that case, a dirty workaround is to call one of the applications (e.g. deviceQueryDrv) once in an /etc/init/ script at login-session-start.


# This script is executed by CMakeLists.txt in the parent folder. Do not run this manually.
# Patches libcudaHOG to remove the -mt suffix for the boost_program_options dependency

cd $1

if [ ! -f cudaHOG.sln ]; then
    echo "This script must be provided with the libcudaHOG source folder as argument!"
else
    echo "Patching boost program options dependency recursively in $PWD"
    grep -rl "boost_program_options-mt" src/ | xargs sed -i 's/boost_program_options-mt/boost_program_options/g'
fi

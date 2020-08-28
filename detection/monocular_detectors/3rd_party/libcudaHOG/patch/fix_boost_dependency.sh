# This removes the -mt suffix
cd $1
echo "Patching boost program options dependency recursively in $PWD"
grep -rl "boost_program_options-mt" src/ | xargs sed -i 's/boost_program_options-mt/boost_program_options/g'
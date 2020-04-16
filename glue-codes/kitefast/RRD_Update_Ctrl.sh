# Install script for KiteFAST libray related components on Debian Stretch (9)
# ONLY KiteFAST and MBDYN'S USER MODULE, not MBDYN per se

# source the helper functions
source kitefast_helpers.sh
 
# exit on error
set -e

##### configuration

# Set the directories in the variables below. These are the 
# directories where kitefast and mbdyn will ultimately go.
source_code_parent_directory="/mnt/d/Users/rdamiani"
mbdyn_directory=$source_code_parent_directory"/mbdyn-1.7.3"
openfast_directory=$source_code_parent_directory"/sandbox"

# set the fortran compiler path
fortran_compiler="/usr/bin/gfortran"

#####

### install required software
packages=`apt -qq list --installed`


# move into the parent directory
if [ ! -d $source_code_parent_directory ]; then
  echo "source_code_parent_directory does not exist as given: "$source_code_parent_directory
  exit 1
fi
cd $source_code_parent_directory

# build KiteFASTController: no pulling and no symlinks, for git pulls and symlinks go to the original Install.sh script
# [these can be important if the usermodule (kitefastmbd.cc) gets modified as well]

export FC=$fortran_compiler
cd $openfast_directory
if [ ! -d build ]; then
  mkdir build
fi
cd build
cmake .. -DGENERATE_TYPES=ON -DDOUBLE_PRECISION=OFF
make  kitefastcontrollerlib kitefastcontroller_driver kitefastcontroller_controller kitefastcontroller_testexe C_MATH C_MATH/fast


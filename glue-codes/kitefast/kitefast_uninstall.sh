# Uninstall script for all KiteFAST related components on Debian Stretch (9)

# source the helper functions
source kitefast_helpers.sh

# exit on error
set -e

##### configuration

# set the directories in the variables below. these are the 
# directories where kitefast and mbdyn will ultimately go
source_code_parent_directory="/Users/rmudafor/Development/makani"
mbdyn_directory=$source_code_parent_directory"/mbdyn-1.7.3"
openfast_directory=$source_code_parent_directory"/sandbox"

# set the fortran compiler path
fortran_compiler="/usr/bin/gfortran"

#####

### uninstall dependencies
packages=`apt -qq list --installed`

# install these general software development tools
uninstall_if_found "git"
uninstall_if_found "cmake"
uninstall_if_found "build-essential"
uninstall_if_found "software-properties-common"
uninstall_if_found "gfortran-6"
uninstall_if_found "libblas-dev"   # blas math library
uninstall_if_found "liblapack-dev" # lapack math library
uninstall_if_found "libltdl-dev"   # libltdl headers, used in mbdyn for linking
uninstall_if_found "libgsl-dev"    # used in the STI controller
uninstall_if_found "python3-pip"   # used in the STI controller

# remove lingering packages
sudo apt-get autoremove -y

# remove KiteFAST build directory
rm -rf $openfast_directory/build

# remove the mbdyn external module from mbdyn
sudo rm -rf $mbdyn_directory/modules/module-kitefastmbd*

# clean the mbdyn build
cd $mbdyn_directory
sudo make clean

# remove the mbdyn installation
sudo rm -rf /usr/local/mbdyn

# Install script for KiteFAST libray related components on Debian Stretch (9)
# ONLY KiteFAST and MBDYN'S USER MODULE, not MBDYN per se

# source the helper functions
source kitefast_helpers.sh
 
# exit on error
set -e

##### configuration

# Set the directories in the variables below. These are the 
# directories where kitefast and mbdyn will ultimately go.

#Use either one depending on platform, if windows the first, else the second
#source_code_parent_directory="/mnt/d/Users/rdamiani"
source_code_parent_directory="/home/rdamiani"

mbdyn_directory=$source_code_parent_directory"/mbdyn-1.7.3"
openfast_directory=$source_code_parent_directory"/sandbox"

# set the fortran compiler path
fortran_compiler=/usr/bin/gfortran 
#home/rdamiani/anaconda3/bin/x86_64-conda_cos6-linux-gnu-gfortran

#####

### install required software
# packages=`apt -qq list --installed`

# update apt-get repo
#sudo apt update

# install these general software development tools
# install_if_not_found "git"
# install_if_not_found "cmake"
# install_if_not_found "build-essential"
# install_if_not_found "software-properties-common"
# install_if_not_found "gfortran-6"
# install_if_not_found "libblas-dev"   # blas math library
# install_if_not_found "liblapack-dev" # lapack math library
# install_if_not_found "libltdl-dev"   # libltdl headers, used in mbdyn for linking
# install_if_not_found "libgsl-dev"    # used in the STI controller
# install_if_not_found "python3-pip"   # used in the STI controller
# install_if_not_found "libnetcdf-dev"
# install_if_not_found "libnetcdf-cxx-legacy-dev"

# remove lingering packages
# sudo apt-get autoremove

# move into the parent directory
if [ ! -d $source_code_parent_directory ]; then
  echo "source_code_parent_directory does not exist as given: "$source_code_parent_directory
  exit 1
fi
cd $source_code_parent_directory

#>>>>>>>>>  !!!!!!!!!!!!build KiteFAST: NO-PULLING for git pulls go to the original Install.sh script  <<<<<<<<<!!!!!!!!!

# >>> STILL POINT TO THE NEW Gfortran on CONDA, but no need to activate env! <<<
export FC=$fortran_compiler

cd $openfast_directory
if [ ! -d build ]; then
  mkdir build
fi
cd build

#/usr/lib/lapack  /usr/lib/x86_64-linux-gnu/libgslcblas.so.0
cmake ..  -DCMAKE_BUILD_TYPE=DEBUG -DDOUBLE_PRECISION=OFF  -DGENERATE_TYPES=ON
make -j 2 kitefastlib kitefastoslib kitefastcontroller_controller

# download mbdyn, configure, and build
cd $source_code_parent_directory

# if mbdyn  doesnt exist, something is wrong
if [ ! -d $mbdyn_directory ]; then
  echo "mbdyn_directory does not exist as given: "$mbdyn_directory
  exit 1
fi

#sudo rm -fr $mbdyn_directory/modules/module-kitefastmbd/
#sudo rm -fr $mbdyn_directory/modules/module-kitefastmbd-os/

# create the links for the onshore module
# if [ ! -d $mbdyn_directory/modules/module-kitefastmbd ]; then
#   mkdir $mbdyn_directory/modules/module-kitefastmbd
# fi
# destination_directory="$mbdyn_directory/modules/module-kitefastmbd"
# create_link $openfast_directory/glue-codes/kitefast/module-kitefastmbd/Makefile.inc $destination_directory/Makefile.inc
# create_link $openfast_directory/glue-codes/kitefast/module-kitefastmbd/module-kitefastmbd.cc $destination_directory/module-kitefastmbd.cc
# create_link $openfast_directory/glue-codes/kitefast/module-kitefastmbd/module-kitefastmbd.h $destination_directory/module-kitefastmbd.h
# create_link $openfast_directory/build/modules/kitefast-library/libkitefastlib.a $destination_directory/libkitefastlib.a
# create_link $openfast_directory/build/modules/nwtc-library/libnwtclibs.a $destination_directory/libnwtclibs.a
# create_link $openfast_directory/build/modules/moordyn/libmoordynlib.a $destination_directory/libmoordynlib.a
# create_link $openfast_directory/build/modules/kiteaerodyn/libkiteaerodynlib.a $destination_directory/libkiteaerodynlib.a
# create_link $openfast_directory/build/modules/vsm/libvsmlib.a $destination_directory/libvsmlib.a
# create_link $openfast_directory/build/modules/actuatordisk/libactuatordisklib.a $destination_directory/libactuatordisklib.a
# create_link $openfast_directory/build/modules/aerodyn/libairfoilinfolib.a $destination_directory/libairfoilinfolib.a
# create_link $openfast_directory/build/modules/inflowwind/libifwlib.a $destination_directory/libifwlib.a
# create_link $openfast_directory/build/modules/version/libversioninfolib.a $destination_directory/libversioninfolib.a
# create_link $openfast_directory/build/modules/kitefast-controller/libkitefastcontrollerlib.a $destination_directory/libkitefastcontrollerlib.a

# create the links for the offshore module
# if [ ! -d $mbdyn_directory/modules/module-kitefastmbd-os ]; then
#   mkdir $mbdyn_directory/modules/module-kitefastmbd-os
# fi
# destination_directory="$mbdyn_directory/modules/module-kitefastmbd-os"
# create_link $openfast_directory/glue-codes/kitefast/module-kitefastmbd-os/Makefile.inc $destination_directory/Makefile.inc
# create_link $openfast_directory/glue-codes/kitefast/module-kitefastmbd-os/module-kitefastmbd-os.cc $destination_directory/module-kitefastmbd-os.cc
# create_link $openfast_directory/glue-codes/kitefast/module-kitefastmbd-os/module-kitefastmbd-os.h $destination_directory/module-kitefastmbd-os.h
# create_link $openfast_directory/glue-codes/kitefast/module-kitefastmbd-os/KiteFASTNode.cc $destination_directory/KiteFASTNode.cc
# create_link $openfast_directory/build/modules/kitefast-library/libkitefastoslib.a $destination_directory/libkitefastoslib.a
# create_link $openfast_directory/build/modules/nwtc-library/libnwtclibs.a $destination_directory/libnwtclibs.a
# create_link $openfast_directory/build/modules/moordyn/libmoordynlib.a $destination_directory/libmoordynlib.a
# create_link $openfast_directory/build/modules/kiteaerodyn/libkiteaerodynlib.a $destination_directory/libkiteaerodynlib.a
# create_link $openfast_directory/build/modules/vsm/libvsmlib.a $destination_directory/libvsmlib.a
# create_link $openfast_directory/build/modules/actuatordisk/libactuatordisklib.a $destination_directory/libactuatordisklib.a
# create_link $openfast_directory/build/modules/aerodyn/libairfoilinfolib.a $destination_directory/libairfoilinfolib.a
# create_link $openfast_directory/build/modules/inflowwind/libifwlib.a $destination_directory/libifwlib.a
# create_link $openfast_directory/build/modules/version/libversioninfolib.a $destination_directory/libversioninfolib.a
# create_link $openfast_directory/build/modules/kitefast-controller/libkitefastcontrollerlib.a $destination_directory/libkitefastcontrollerlib.a
# create_link $openfast_directory/build/modules/hydrodyn/libhydrodynlib.a $destination_directory/libhydrodynlib.a

# Now build mbdyn's USER MODULE and install it in the right place

cd $mbdyn_directory

cd modules                     # move to the module directory
make clean
# sudo make                      # build the user defined element
sudo make # install              # install the user defined element in the right place


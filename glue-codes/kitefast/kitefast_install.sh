# Install script for all KiteFAST related components on Debian Stretch (9)

# source the helper functions
source kitefast_helpers.sh

# exit on error
set -e

##### configuration

# Set the directories in the variables below. These are the 
# directories where kitefast and mbdyn will ultimately go.
source_code_parent_directory="/Users/rmudafor/Development/makani"
mbdyn_directory=$source_code_parent_directory"/mbdyn-1.7.3"
openfast_directory=$source_code_parent_directory"/sandbox"

# set the fortran compiler path
fortran_compiler="/usr/bin/gfortran"

#####

### install required software
packages=`apt -qq list --installed`

# update apt-get repo
sudo apt update

# install these general software development tools
install_if_not_found "git"
install_if_not_found "cmake"
install_if_not_found "build-essential"
install_if_not_found "software-properties-common"
install_if_not_found "gfortran-6"
install_if_not_found "libblas-dev"   # blas math library
install_if_not_found "liblapack-dev" # lapack math library
install_if_not_found "libltdl-dev"   # libltdl headers, used in mbdyn for linking
install_if_not_found "libgsl-dev"    # used in the STI controller
install_if_not_found "python3-pip"   # used in the STI controller
install_if_not_found "libnetcdf-dev"
install_if_not_found "libnetcdf-cxx-legacy-dev"

# remove lingering packages
sudo apt-get autoremove

# move into the parent directory
if [ ! -d $source_code_parent_directory ]; then
  echo "source_code_parent_directory does not exist as given: "$source_code_parent_directory
  exit 1
fi
cd $source_code_parent_directory

# build KiteFAST
git_branch="dev"
if [ -d $openfast_directory ]; then
  cd $openfast_directory
  git checkout $git_branch
  git pull origin $git_branch
else
  git config --global http.sslVerify false
  git clone -b $git_branch https://makani-private.googlesource.com/kite_fast/sandbox
fi
if [ ! -d $openfast_directory ]; then
   echo "openfast_directory does not exist as given: "$openfast_directory
   exit 1
fi

export FC=$fortran_compiler
cd $openfast_directory
if [ ! -d build ]; then
  mkdir build
fi
cd build
cmake .. -DDOUBLE_PRECISION=OFF
make -j 2 kitefastlib kitefastcontroller_controller

# download mbdyn, configure, and build
cd $source_code_parent_directory

# if it doesnt already exist, download it
if [ ! -d $mbdyn_directory ]; then
  wget "https://www.mbdyn.org/userfiles/downloads/mbdyn-1.7.3.tar.gz" --no-check-certificate
  gunzip mbdyn-1.7.3.tar.gz
  tar -xf mbdyn-1.7.3.tar
fi

# if it still doesnt exist, something went wrong
if [ ! -d $mbdyn_directory ]; then
  echo "mbdyn_directory does not exist as given: "$mbdyn_directory
  exit 1
fi

# create the links for the module
if [ ! -d $mbdyn_directory/modules/module-kitefastmbd ]; then
  mkdir $mbdyn_directory/modules/module-kitefastmbd
fi
destination_directory="$mbdyn_directory/modules/module-kitefastmbd"
create_link $openfast_directory/glue-codes/kitefast/module-kitefastmbd/Makefile.inc $destination_directory/Makefile.inc
create_link $openfast_directory/glue-codes/kitefast/module-kitefastmbd/module-kitefastmbd.cc $destination_directory/module-kitefastmbd.cc
create_link $openfast_directory/glue-codes/kitefast/module-kitefastmbd/module-kitefastmbd.h $destination_directory/module-kitefastmbd.h
create_link $openfast_directory/glue-codes/kitefast/module-kitefastmbd/KiteFASTNode.cc $destination_directory/KiteFASTNode.cc
create_link $openfast_directory/build/modules/kitefast-library/libkitefastlib.a $destination_directory/libkitefastlib.a
create_link $openfast_directory/build/modules/nwtc-library/libnwtclibs.a $destination_directory/libnwtclibs.a
create_link $openfast_directory/build/modules/moordyn/libmoordynlib.a $destination_directory/libmoordynlib.a
create_link $openfast_directory/build/modules/kiteaerodyn/libkiteaerodynlib.a $destination_directory/libkiteaerodynlib.a
create_link $openfast_directory/build/modules/vsm/libvsmlib.a $destination_directory/libvsmlib.a
create_link $openfast_directory/build/modules/actuatordisk/libactuatordisklib.a $destination_directory/libactuatordisklib.a
create_link $openfast_directory/build/modules/aerodyn/libairfoilinfolib.a $destination_directory/libairfoilinfolib.a
create_link $openfast_directory/build/modules/inflowwind/libifwlib.a $destination_directory/libifwlib.a
create_link $openfast_directory/build/modules/version/libversioninfolib.a $destination_directory/libversioninfolib.a
create_link $openfast_directory/build/modules/kitefast-controller/libkitefastcontrollerlib.a $destination_directory/libkitefastcontrollerlib.a

# # configure and build mbdyn
export LDFLAGS=-rdynamic
cd $mbdyn_directory

# modify the line below as needed
# for debug, add --enable-debug
./configure --enable-runtime-loading --with-module="kitefastmbd" --enable-netcdf --with-lapack --enable-eig

sudo make                      # build mbdyn
cd modules                     # move to the module directory
sudo make                      # build the user defined element
cd ..                          # move back to mbdyn
sudo make install              # install everything

# install dependencies for the mbdyn preprocessor
pip3 install -r $openfast_directory/glue-codes/kitefast/preprocessor/requirements.txt

# add the mbdyn installation directory to your .bashrc
echo -e '\nPATH="/usr/local/mbdyn/bin:$PATH"\n' >> ~/.bashrc
# close and reopen the terminal

### optional
# visualization / post processing
# install_package blender
# wget https://github.com/zanoni-mbdyn/blendyn/archive/master.zip
# see instructions at https://github.com/zanoni-mbdyn/blendyn/wiki/Installation

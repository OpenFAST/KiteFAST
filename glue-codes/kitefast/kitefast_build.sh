# Build script for all KiteFAST related components on Ubuntu 14.04

### configuration

# download mbdyn and openfast and put both in the same directory. also, set their parent directory below
source_code_parent_directory="/home/makani/Desktop/"
if [ ! -d $source_code_parent_directory ]; then
  echo "source_code_parent_directory does not exist as given: "$source_code_parent_directory
  exit 1
fi
cd $source_code_parent_directory

# download mbdyn and unzip:
wget "https://www.mbdyn.org/userfiles/downloads/mbdyn-1.7.3.tar.gz"
gunzip mbdyn-1.7.3.tar.gz
tar -xf mbdyn-1.7.3.tar
mbdyn_directory=$source_code_parent_directory"/mbdyn-1.7.3/"
if [ ! -d $mbdyn_directory ]; then
  echo "mbdyn_directory does not exist as given: "$mbdyn_directory
  exit 1
fi

# clone openfast
# git clone https://makani-private.googlesource.com/kite_fast/nrel_source openfast
openfast_directory=$source_code_parent_directory"/sandbox/"
if [ ! -d $openfast_directory ]; then
  echo "openfast_directory does not exist as given: "$openfast_directory
  exit 1
fi

###

### install required software
packages=`apt -qq list --installed`

function install_if_not_found {
  if ! package_installed $1; then
    install_package $1
  fi
}

function package_installed {
  echo "*** Checking for "$1
  echo $packages | grep -q $1
  installed=$?
  return $installed
}

function install_package {
  echo "*** Installing "$1
  sudo apt install -y $1
}

# if git is not installed, add this external ppa
which git
if [[ $? -eq 1 ]]; then
    install_package "software-properties-common"
    sudo apt-add-repository ppa:git-core/ppa
fi

# update apt-get repo
sudo apt update

# install these general software development tools
install_if_not_found "git"
install_if_not_found "build-essential"
install_if_not_found "software-properties-common"
install_if_not_found "gfortran" # this does not come on 14.04 by default
install_if_not_found "cmake"
install_if_not_found "libblas-dev" # blas math library
install_if_not_found "liblapack-dev" # lapack math library
install_if_not_found "libltdl-dev" # libltdl headers, used in mbdyn for linking
install_if_not_found "libgsl-dev" # used in the STI controller

# remove lingering packages
sudo apt-get autoremove

# build KiteFAST
export FC=/usr/bin/gfortran
cd $openfast_directory
mkdir build
cd build
cmake ..
make -j 2 kitefastlib

# copy the external module from openfast to mbdyn
cp -r $openfast_directory/glue-codes/kitefast/module-kitefastmbd $mbdyn_directory/modules/.

# link kitefast lib and its module file to the module-kitefastmbd directory
ln -s $openfast_directory/build/modules-local/kitefast-library/libkitefastlib.a $mbdyn_directory/modules/module-kitefastmbd/.
ln -s $openfast_directory/build/modules-local/nwtc-library/libnwtclibs.a $mbdyn_directory/modules/module-kitefastmbd/.
ln -s $openfast_directory/build/modules-ext/moordyn/libmoordynlib.a $mbdyn_directory/modules/module-kitefastmbd/.
ln -s $openfast_directory/build/modules-local/kiteaerodyn/libkiteaerodynlib.a $mbdyn_directory/modules/module-kitefastmbd/.
ln -s $openfast_directory/build/modules-local/vsm/libvsmlib.a $mbdyn_directory/modules/module-kitefastmbd/.
ln -s $openfast_directory/build/modules-local/actuatordisk/libactuatordisklib.a $mbdyn_directory/modules/module-kitefastmbd/.
ln -s $openfast_directory/build/modules-local/aerodyn/libairfoilinfolib.a $mbdyn_directory/modules/module-kitefastmbd/.
ln -s $openfast_directory/build/modules-local/inflowwind/libifwlib.a $mbdyn_directory/modules/module-kitefastmbd/.
ln -s $openfast_directory/build/modules-local/kitefast-controller/libkitefastcontrollerlib.a $mbdyn_directory/modules/module-kitefastmbd/.

# configure and build mbdyn
export LDFLAGS=-rdynamic
cd $mbdyn_directory
./configure --enable-runtime-loading --with-module="kitefastmbd" --enable-debug
sudo make                      # build mbdyn
cd modules                     # move to the module directory
sudo make                      # build the user defined element
cd ..                          # move back to mbdyn
sudo make install              # install everything

# add the mbdyn installation directory to your .bashrc
echo -e '\nPATH="/usr/local/mbdyn/bin:$PATH"\n' >> ~/.bashrc
source ~/.bashrc

### optional
# visualization / post processing
# install_package blender

# for the blender plotting functionality… but this has not been successfully installed
# sudo apt install libhdf5-dev -y
# sudo apt install libnetcdf-dev
# sudo apt install libhdf5-serial-dev netcdf-bin libnetcdf-dev

# wget https://github.com/zanoni-mbdyn/blendyn/archive/master.zip
# see instructions at https://github.com/zanoni-mbdyn/blendyn/wiki/Installation

# controller specific:
# set LD_LIBRARY_PATH to the kitefast controller build directory
#export LD_LIBRARY_PATH="/home/raf/Desktop/makani/makani_openfast/build/modules-local/kitefast-controller"

# To run mbdyn, move into the directory containing the model and run
#mbdyn model_file.mbd

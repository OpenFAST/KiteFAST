# Build script for all KiteFAST related components on Debian Strech (9)

# exit on error
set -e

##### configuration

# set the directories in the variables below. these are the 
# directories where kitefast and mbdyn will ultimately go
source_code_parent_directory="/home/parallels/Desktop"
mbdyn_directory=$source_code_parent_directory"/mbdyn-1.7.3"
openfast_directory=$source_code_parent_directory"/sandbox"

if [ ! -d $source_code_parent_directory ]; then
  echo "source_code_parent_directory does not exist as given: "$source_code_parent_directory
  exit 1
fi
cd $source_code_parent_directory

#####

### helpers

function print {
  echo "*** "$@
}

### install required software
packages=`apt -qq list --installed`

function install_if_not_found {
  if package_installed $1; then
    print $1" already installed."
    return
  fi

  if ! install_package $1; then
    print $1" could not be installed."
  else
    print $1" successfully installed."
  fi
}

function package_installed {
  print "Checking for "$1
  echo $packages | grep -q $1
  return $?
}

function install_package {
  print "Installing "$1
  sudo apt install -y $1
  return $?
}

function create_link {
  if [ ! -L $2 ]; then
    ln -s $1 $2
  fi
}

# update apt-get repo
sudo apt update

# install these general software development tools
install_if_not_found "git/stable"
install_if_not_found "cmake/stable"
install_if_not_found "build-essential/stable"
install_if_not_found "software-properties-common/stable"
install_if_not_found "gfortran-6/stable"
install_if_not_found "libblas-dev/stable"   # blas math library
install_if_not_found "liblapack-dev/stable" # lapack math library
install_if_not_found "libltdl-dev/stable"   # libltdl headers, used in mbdyn for linking
install_if_not_found "libgsl-dev/stable"    # used in the STI controller
install_if_not_found "python3-pip"    # used in the STI controller

# remove lingering packages
sudo apt-get autoremove

# build KiteFAST
if [ -d $openfast_directory ]; then
  cd $openfast_directory
  git checkout dev
  git pull origin dev
else
  git config --global http.sslVerify false
  git clone -b dev https://makani-private.googlesource.com/kite_fast/sandbox
fi
if [ ! -d $openfast_directory ]; then
   echo "openfast_directory does not exist as given: "$openfast_directory
   exit 1
fi

export FC=/usr/bin/gfortran
cd $openfast_directory
if [ ! -d build ]; then
  mkdir build
fi
cd build
cmake ..
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

# # copy the external module from openfast to mbdyn
cp -r $openfast_directory/glue-codes/kitefast/module-kitefastmbd $mbdyn_directory/modules/.

# # link kitefast lib and its module file to the module-kitefastmbd directory
create_link $openfast_directory/build/modules-local/kitefast-library/libkitefastlib.a $mbdyn_directory/modules/module-kitefastmbd/libkitefastlib.a
create_link $openfast_directory/build/modules-local/nwtc-library/libnwtclibs.a $mbdyn_directory/modules/module-kitefastmbd/libnwtclibs.a
create_link $openfast_directory/build/modules-ext/moordyn/libmoordynlib.a $mbdyn_directory/modules/module-kitefastmbd/libmoordynlib.a
create_link $openfast_directory/build/modules-local/kiteaerodyn/libkiteaerodynlib.a $mbdyn_directory/modules/module-kitefastmbd/libkiteaerodynlib.a
create_link $openfast_directory/build/modules-local/vsm/libvsmlib.a $mbdyn_directory/modules/module-kitefastmbd/libvsmlib.a
create_link $openfast_directory/build/modules-local/actuatordisk/libactuatordisklib.a $mbdyn_directory/modules/module-kitefastmbd/libactuatordisklib.a
create_link $openfast_directory/build/modules-local/aerodyn/libairfoilinfolib.a $mbdyn_directory/modules/module-kitefastmbd/libairfoilinfolib.a
create_link $openfast_directory/build/modules-local/inflowwind/libifwlib.a $mbdyn_directory/modules/module-kitefastmbd/libifwlib.a
create_link $openfast_directory/build/modules-local/kitefast-controller/libkitefastcontrollerlib.a $mbdyn_directory/modules/module-kitefastmbd/libkitefastcontrollerlib.a

# # configure and build mbdyn
export LDFLAGS=-rdynamic
cd $mbdyn_directory
./configure --enable-runtime-loading --with-module="kitefastmbd" # --enable-debug
sudo make                      # build mbdyn
cd modules                     # move to the module directory
sudo make                      # build the user defined element
cd ..                          # move back to mbdyn
sudo make install              # install everything

# install dependencies for the mbdyn preprocessor
pip3 install -r $openfast_directory/glue-codes/kitefast/preprocessor/requirements.txt

# add the mbdyn installation directory to your .bashrc
echo -e '\nPATH="/usr/local/mbdyn/bin:$PATH"\n' >> ~/.bashrc
source ~/.bashrc

### optional
# visualization / post processing
# install_package blender
# wget https://github.com/zanoni-mbdyn/blendyn/archive/master.zip
# see instructions at https://github.com/zanoni-mbdyn/blendyn/wiki/Installation

# controller specific:
# set LD_LIBRARY_PATH to the kitefast controller build directory
#export LD_LIBRARY_PATH="/home/raf/Desktop/makani/makani_openfast/build/modules-local/kitefast-controller"

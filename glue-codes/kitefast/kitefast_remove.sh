# Build script for all KiteFAST related components on Ubuntu 14.04

### configuration

# download mbdyn and openfast and put both in the same directory. also, set their parent directory below
source_code_parent_directory="/home/raf/Desktop/"

# download mbdyn and unzip:
# wget "https://www.mbdyn.org/userfiles/downloads/mbdyn-1.7.3.tar.gz"
# gunzip mbdyn-1.7.3.tar.gz
# tar -xf mbdyn-1.7.3.tar
mbdyn_directory=$source_code_parent_directory"/mbdyn-1.7.3/"

# clone openfast
# git clone https://makani-private.googlesource.com/kite_fast/nrel_source openfast
openfast_directory=$source_code_parent_directory"/nrel_source/"

###

### install required software
packages=`apt -qq list --installed`

function uninstall_if_found {
  if package_installed $1; then
    uninstall_package $1
  fi
}

function package_installed {
  echo "*** Checking for "$1
  echo $packages | grep -q $1
  installed=$?
  return $installed
}

function uninstall_package {
  echo "*** Uninstalling "$1
  sudo apt remove -y $1
}

# if git is not installed, add this external ppa
which git
if [[ $? -eq 1 ]]; then
    sudo add-apt-repository ppa:git-core/ppa
fi

# install these general software development tools
uninstall_if_found "git"
uninstall_if_found "build-essential"
uninstall_if_found "software-properties-common"
uninstall_if_found "gfortran" # this does not come on 14.04 by default
uninstall_if_found "cmake"
uninstall_if_found "libblas-dev" # blas math library
uninstall_if_found "liblapack-dev" # lapack math library
uninstall_if_found "libltdl-dev" # libltdl headers, used in mbdyn for linking

# remove lingering packages
sudo apt-get autoremove -y

# remove KiteFAST build directory
rm -rf $openfast_directory/build

# remove the mbdyn external module from  mbdyn
sudo rm -rf $mbdyn_directory/modules/module-kitefastmbd

# clean the mbdyn build
cd $mbdyn_directory
sudo make clean

# remove the mbdyn installation
sudo rm -rf /usr/local/mbdyn

# add the mbdyn installation directory to your .bashrc
#echo 'PATH="/usr/local/mbdyn/bin:$PATH"' >> ~/.bashrc

# Build script for all KiteFAST related components on Debian Strech (9)

# exit on error
set -e

##### configuration

source_code_parent_directory="/Users/rmudafor/Development/makani"
mbdyn_directory=$source_code_parent_directory"/mbdyn-1.7.3"
openfast_directory=$source_code_parent_directory"/makani_openfast"

if [ ! -d $source_code_parent_directory ]; then
  echo "source_code_parent_directory does not exist as given: "$source_code_parent_directory
  exit 1
fi
cd $source_code_parent_directory

#####

### helpers

require_clean_work_tree () {
    # Update the index
    git update-index -q --ignore-submodules --refresh
    err=0

    # Disallow unstaged changes in the working tree
    if ! git diff-files --quiet --ignore-submodules --
    then
        echo >&2 "cannot $1: you have unstaged changes."
        git diff-files --name-status -r --ignore-submodules -- >&2
        err=1
    fi

    # Disallow uncommitted changes in the index
    if ! git diff-index --cached --quiet HEAD --ignore-submodules --
    then
        echo >&2 "cannot $1: your index contains uncommitted changes."
        git diff-index --cached --name-status -r --ignore-submodules HEAD -- >&2
        err=1
    fi

    if [ $err = 1 ]
    then
        echo >&2 "Please commit or stash them."
        exit 1
    fi
}

function print {
  echo "*** "$@
}

# move into kitefast
if [ ! -d $openfast_directory ]; then
   echo "openfast_directory does not exist as given: "$openfast_directory
   exit 1
fi
cd $openfast_directory

require_clean_work_tree "git checkout"
git checkout dev
git pull origin dev

if [ ! -d build ]; then
  mkdir build
fi
cd build
cmake ..
make -j 2 kitefastlib kitefastcontroller_controller

# rebuild the mbdyn user module
cd $source_code_parent_directory
if [ ! -d $mbdyn_directory ]; then
  echo "mbdyn_directory does not exist as given: "$mbdyn_directory
  exit 1
fi
cd $mbdyn_directory
sudo make                      # build mbdyn
cd modules                     # move to the module directory
sudo make                      # build the user defined element
cd ..                          # move back to mbdyn
sudo make install              # install everything

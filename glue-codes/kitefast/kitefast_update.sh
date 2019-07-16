# Update script for all KiteFAST related components on Debian Strech (9)

# exit on error
set -e

##### configuration

source_code_parent_directory="/home/parallels/Desktop"
mbdyn_directory=$source_code_parent_directory"/mbdyn-1.7.3"
openfast_directory=$source_code_parent_directory"/sandbox"
$git_branch="dev-offshore"

#####

##### helpers

function print {
  echo "*** "$@
}

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

#####

# check the directories
if [ ! -d $source_code_parent_directory ]; then
  print "source_code_parent_directory does not exist as given: "$source_code_parent_directory
  exit 1
fi

if [ ! -d $openfast_directory ]; then
   print "openfast_directory does not exist as given: "$openfast_directory
   exit 1
fi

if [ ! -d $openfast_directory/build ]; then
  print "kitefast build directory does not exist as given: "$openfast_directory/build
  exit 1
fi

if [ ! -d $mbdyn_directory ]; then
  print "mbdyn_directory does not exist as given: "$mbdyn_directory
  exit 1
fi

# rebuild kitefast
cd $openfast_directory
require_clean_work_tree "git checkout"
git checkout $git_branch
git pull origin $git_branch
cd build
cmake ..
make -j 2 kitefastlib kitefastoslib kitefastcontroller_controller

# rebuild the mbdyn user module
cd $mbdyn_directory/modules
make clean && sudo make install  # build the user defined element
cd ..                            # move back to mbdyn
sudo make install                # install everything

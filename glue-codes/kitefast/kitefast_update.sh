# Update script for all KiteFAST related components on Debian Strech (9)

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
git_branch="dev"

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
make -j 2 kitefastlib kitefastcontroller_controller

# rebuild the mbdyn user module
cd $mbdyn_directory/modules
make clean && sudo make install  # build the user defined element
cd ..                            # move back to mbdyn
sudo make install                # install everything

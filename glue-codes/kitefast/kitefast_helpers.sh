

# bash helpers
function print {
  echo "*** "$@
}

function create_link {
  if [ -e "$2" ]; then
    mv $2 $2.back
    print "Symlink exists so its been saved as "$2".back"
  fi
  ln -s $1 $2
}


# Package handling
function package_installed {
  print "Checking for "$1
  echo $packages | grep -q $1
  return $?
}

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

function uninstall_if_found {
  if package_installed $1; then
    uninstall_package $1
  fi
}

function install_package {
  print "Installing "$1
  sudo apt install -y $1
  return $?
}

function uninstall_package {
  print "Uninstalling "$1
  sudo apt remove -y $1
}


# Git status
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


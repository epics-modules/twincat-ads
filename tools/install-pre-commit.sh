#!/bin/bash
#
# Script to install packages need by pre-commit
#
##############################################################################
# functions
#
#
checkAndInstallSystemPackage()
{
  while test $# -gt 1; do
    PACKAGENAME=$1
    shift
    if which yum >/dev/null 2>&1; then
      sudo yum install $PACKAGENAME && return 0
    fi
    if which apt-get >/dev/null 2>&1; then
      sudo apt-get install -y $PACKAGENAME && return 0
    fi
    if which port >/dev/null 2>&1; then
      sudo port install $PACKAGENAME && return 0
    fi
  done
  echo >&1 install $PACKAGENAME failed
  return 1
}

########################################
checkAndInstallPythonPackage()
{
  IMPORTNAME=$1

  if ! python -c "import $IMPORTNAME" >/dev/null 2>&1; then
    while test $# -gt 1; do
      shift
      PACKAGEINSTALL=$1
      echo failed: $PYTHON -c "import $IMPORTNAME"
      $PACKAGEINSTALL && return 0
    done
    echo >&1  $PACKAGEINSTALL failed
    exit 1
  fi
}
########################################

MYVIRTUALENV=""
if which pyenv-virtualenv >/dev/null 2>&1; then
  # brew has pyenv-virtualenv
  # and a bug, "pyenv-root" should be written as "pyenv root"
  PYENV_ROOT="$(pyenv root)"
  export PYENV_ROOT
  MYVIRTUALENV="pyenv virtualenv"
  if test -e /usr/local/opt/python@3.7/bin; then
    PATH=/usr/local/opt/python@3.7/bin:$PATH
    export PATH
  fi
elif type virtualenv >/dev/null 2>&1; then
  MYVIRTUALENV=virtualenv
fi

PYTEST=pytest
PYTHON=python3
if test -z "$MYVIRTUALENV"; then
  n=19 # python 3.19 does not yet exist, but may some day
  # Loop from high python versions down to the lowest, 3.6
  # 3.6 is needed because of the f-strings
  while test $n -ge 6; do
    if test -z "$MYVIRTUALENV"; then
      if which virtualenv-3.$n >/dev/null 2>&1; then
        MYVIRTUALENV=virtualenv-3.$n
        if which python3.$n >/dev/null 2>&1; then
          PYTHON=python3.$n
          break
        elif which python3$n >/dev/null 2>&1; then
          PYTHON=python3$n
          break
        fi
      fi
    fi
    n=$((n -1))
  done
fi

echo MYVIRTUALENV=$MYVIRTUALENV
export MYVIRTUALENV

########################################
if test -z "$MYVIRTUALENV"; then
  echo >&2 "vitualenv not found"
  exit 1
fi

##############################################################################
if test -n "$MYVIRTUALENV" && type $MYVIRTUALENV >/dev/null 2>&1; then
  VIRTUALENVDIR=venv$PYTHON
  if test -d $HOME/.pyenv/versions/$VIRTUALENVDIR/bin/ ; then
    VIRTUALENVDIR=$HOME/.pyenv/versions/$VIRTUALENVDIR
  fi
  if test -r $VIRTUALENVDIR/bin/activate; then
    .  $VIRTUALENVDIR/bin/activate
  elif test -z "$MYVIRTUALENV"; then
    checkAndInstallSystemPackage py37-virtualenv virtualenv python-virtualenv || {
      echo >2 "could not install virtualenv"
    }
    echo >2 "virtualenv has been installed"
    echo >2 "Re-run the script"
    exit 1
  else
    $MYVIRTUALENV --python=$PYTHON $VIRTUALENVDIR || {
      echo >&2 $MYVIRTUALENV failed
      exit 1
    }
  fi
  if test -r $VIRTUALENVDIR/bin/activate; then
    .  $VIRTUALENVDIR/bin/activate
  fi
else
  if which conda >/dev/null 2>&1; then
    checkAndInstallPythonPackage pytest "conda install -c conda-forge pyTest"
    checkAndInstallPythonPackage epics  "conda install -c https://conda.anaconda.org/GSECARS pyepics" "conda install pyepics"
  fi
fi
checkAndInstallPythonPackage epics "pip3 install pyepics" "pip install pyepics" &&
checkAndInstallPythonPackage p4p "pip3 install p4p" "pip install p4p"
checkAndInstallPythonPackage pyads "pip3 install pyads" "pip install pyads"
checkAndInstallPythonPackage pytest "pip3 install $PYTEST" "pip install $PYTEST" || {
  echo >&2 Installation problem:
  echo >&2 pip not found
  echo >&2 easy_install not found
  exit 1
}

if ! type pre-commit >/dev/null 2>&1; then
  pip install pre-commit
fi

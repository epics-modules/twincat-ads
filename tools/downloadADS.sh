#!/bin/sh
#
#    This file is part of twincat-ads.
#
#    twincat-ads is free software: you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
#
#    twincat-ads is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License along with twincat-ads. If not, see <https://www.gnu.org/licenses/>.
#
#echo "$0 PWD=$PWD" "$@"

if ! test -e BeckhoffADS/AdsLib/AdsLib.h; then
  (
    git submodule init BeckhoffADS && git submodule update BeckhoffADS
  )
fi
action=$1
shift
case "$action" in
clean)
  rm -f adsApp/src/ADS_FROM_BECKHOFF_SUPPORTSOURCES.mak
  for src in "$@"; do
    dst=${src##*/}
    rm -f adsApp/src/$dst
  done
  ;;
build)
  >adsApp/src/ADS_FROM_BECKHOFF_SUPPORTSOURCES.mak
  for src in "$@"; do
    dst=${src##*/}
    rm -f adsApp/src/$dst
    ln -s ../../$src adsApp/src/$dst || cp -v $src adsApp/src/$dst
    echo "ADS_FROM_BECKHOFF_SUPPORTSOURCES += $dst" >>adsApp/src/ADS_FROM_BECKHOFF_SUPPORTSOURCES.mak
  done
  ;;
*)
  echo >&2 "$0 [clean|build] <filenames>"
  exit 1
  ;;
esac

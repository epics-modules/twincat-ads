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
TAB=$(printf '\t')
LF=$(printf '\n')
echo TABX=${TAB}X

fileswscheck=$(git ls-files '*.[ch]' '*.cpp' '*.hpp' '*.py')
if test -n "$fileswscheck"; then
  echo fileswscheck=$fileswscheck
  cmd=$(printf "%s %s" 'egrep -n "$TAB| \$"' "$fileswscheck")
  echo cmd=$cmd
  eval $cmd
  if test $? -eq 0; then
    #TABS found
    exit 1
  fi
fi
exit 0

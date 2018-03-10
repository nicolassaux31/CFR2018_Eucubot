#!/bin/bash

echo "Enter the path to the packages3d folder. This script will symlink all 3d models into this folder"
echo -n "Path: "
read packagedir

shimattadir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd $packagedir
for lib in $shimattadir/*.shapes3d; do
	echo "$lib"
	if [[ -d "$lib" ]]; then
		ln -s "$lib" 			
	fi
done

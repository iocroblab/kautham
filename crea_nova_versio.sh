#!/bin/bash
# Script per crear una nova versió del Kautham

echo "Aquest script mostra la numeració de la versió actual"
echo "i preguntarà per modificar-la en els fitxers:"
echo "package.xml i en el CMakeLists.txt"

Answer=no
VersioP="`cat package.xml | grep -F "<version" | sed 's/version//g' | sed 's/<>//g'| sed 's/<\/>//g'`"
VersioP=${VersioP// /}

VersioC="`cat CMakeLists.txt | grep "set( KAUTHAM_VERSION" | sed 's/set( KAUTHAM_VERSION//g' | sed 's/)//g'`"
VersioC=${VersioC// /}

while [ "$Answer" == "no" ]; do
	echo "Gràcies, la versió del package.xml és $VersioP i la del CMakeLists és $VersioC"
	read -p "Quina versió voleu posar: " newVersion
	echo "La nova versió proposada és la $newVersion"
	echo
	echo "Estàs segur d'actualitzar fixers?" 
	read -p ' (si/no):' Answer
	if [ "$Answer" == "si" ] || [ "$Answer" == "yes" ] ; 
	then
		sed -i "s/KAUTHAM_VERSION ${VersioC}/KAUTHAM_VERSION ${newVersion}/g" CMakeLists.txt
		sed -i "s/version>${VersioP}</version>${newVersion}</g" package.xml
	fi
	git diff
	echo "Fitxers canviats. Recordeu de fer:"
	echo "git commit CMakeLists.txt package.xml -m\"New version of Kautham\""
	
done
echo




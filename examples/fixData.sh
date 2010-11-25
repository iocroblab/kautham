#! /bin/sh
#	Ensayo de script shell

DIRLIST=`ls *.xml`
for j in $DIRLIST
do
	# SEAR="TH=\"1.5708\""
	# REPL="TH=\"180.0\""
	# cat $j | sed s:$SEAR:$REPL: > "$j"_old
	# SEAR="TH=\"3.14159\""
	# REPL="TH=\"360.0\""
	# cat "$j"_old | sed s:$SEAR:$REPL: > $j
	# rm -f "$j"_old
	SEAR="Theta"
	REPL="THETA"
	cat $j | sed s:$SEAR:$REPL: > "$j"_old
	SEAR="Wx"
	REPL="WX"
	cat "$j"_old | sed s:$SEAR:$REPL: > "$j"_old1
	rm -f "$j"_old
	SEAR="Wy"
	REPL="WY"
	cat "$j"_old1 | sed s:$SEAR:$REPL: > "$j"_old2
	rm -f "$j"_old1
	SEAR="Wz"
	REPL="WZ"
	cat "$j"_old2 | sed s:$SEAR:$REPL: > $j
	rm -f "$j"_old2
done

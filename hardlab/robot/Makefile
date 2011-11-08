INCLUDES=-I/home/users/leopold.palomo/system/usr/include -I./

default: test.exe

txrobot.o: txrobot.cpp
	g++ -Wall -DVERBOSE -O3 $(INCLUDES) -c txrobot.cpp;

test.o: test.cpp
	g++ -Wall -O3 $(INCLUDES) -c test.cpp;

test.exe: txrobot.o test.o
	g++ -Wall -O3 test.o txrobot.o -o test.exe; 

clean:
	rm *.o test.exe;
	
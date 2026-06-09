andorsvr: andorsvr.o
	g++ -o andorsvr andorsvr.o -landor -latspectrograph

clean:
	rm -rf *.o andorsvr

CXX= g++
CXXFLAGS = -Wall
#RASPI_LDLIBS= -lrealsense2 -lpthread -lraspicam
LDLIBS= -lrealsense2 -lpthread 
FILES= rs_server.cpp
OBJS= rs_server.o
#OPTIONS
#JSON= json/D415/pruebaHRHDP.json
#XML= json/SR305/conf.xml
#PLY= ply/HRHDP_curva
#OPTIONS= -p

main: $(OBJS)
	g++ $(CXXFLAGS) -o rs_streamer $(FILES) $(LDLIBS)

#run:
#	./main -j $(JSON) -x $(XML) -f $(PLY) $(OPTIONS)

clean:
	rm *.o

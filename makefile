CC = g++
CPP_COMP_FLAGS=-m64 -fpic -Wall -Wno-unused -fmax-errors=4 -msse2 -std=c++11
CLIBS = -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_features2d -lpcl_io -lpcl_common -lpcl_visualization -lOpenNI -lboost_system -std=c++11 -lfreenect2
#-lflycapture -ltriclops -lflycapture2bridge -lpnmutils
CPP_INCLUDES = -I/usr/local/include -I/usr/include/triclops/ -I/usr/include/flycapture -I/usr/local/include/pcl-1.8 -I/usr/include/eigen3 -I/usr/include/vtk-5.8 -I/usr/include/ni

CPP_FLAGS = $(CPP_COMP_FLAGS) $(CPP_INCLUDES)
LINK_FLAGS=-lm -lstdc++ -lpthread -ldl -lm $(CLIBS)

TARGET = bum
SRCS =

INC =

OBJS = CameraKinect.o CameraOpenNI.o main.o

$(TARGET):$(OBJS)
	$(CC) -o $@ $^ $(LINK_FLAGS)

%.o:%.cpp
	$(CC) $(CPP_FLAGS) $(INC) -o $@ -c $<

clean:
	rm *.o bum

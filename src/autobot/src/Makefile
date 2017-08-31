NAME1=camera_control
NAME2=camera_test

CC=g++
SRC1=cameraControl.cpp
SRC2=cameraReceiverTest.cpp

CFLAGS=

all: all1 all2

LinuxAll: LinuxAll1 LinuxAll2

all1:
	# $(CC) `pkg-config --cflags --libs opencv3`  $(SRC1) -o $(NAME1)
	$(CC) -L/usr/local/opt/opencv3/lib -I/usr/local/opt/opencv3/include $(SRC1) -o $(NAME1) -lopencv_core -lopencv_imgproc -lopencv_videoio -lopencv_imgcodecs -lopencv_objdetect -lopencv_features2d -lopencv_highgui -lopencv_dnn 

LinuxAll1:
	$(CC) -L/usr/local/lib $(SRC1) -o $(NAME1) -lopencv_core -lopencv_imgproc -lopencv_videoio -lopencv_imgcodecs -lopencv_objdetect -lopencv_features2d -lopencv_highgui -lopencv_dnn

all2:
	# $(CC) `pkg-config --cflags --libs opencv3`  $(SRC1) -o $(NAME2)
	$(CC) -L/usr/local/opt/opencv3/lib -I/usr/local/opt/opencv3/include $(SRC2) -o $(NAME2) -lopencv_core -lopencv_imgproc -lopencv_videoio -lopencv_imgcodecs -lopencv_objdetect -lopencv_features2d -lopencv_highgui -lopencv_dnn 

LinuxAll2:
	$(CC) -L/usr/local/lib $(SRC2) -o $(NAME2) -lopencv_core -lopencv_imgproc -lopencv_videoio -lopencv_imgcodecs -lopencv_objdetect -lopencv_features2d -lopencv_highgui -lopencv_dnn

clean:
	rm $(NAME1)

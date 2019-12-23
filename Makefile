LDFLAGS += -lm -lrt -lpthread
LDFLAGS += -lwampcc -lwampcc_json -lssl -luv
CXXFLAGS += -g -std=c++17 -O2

OBJS = \
    Compensation.o\
    Control.o\
    HMC5883L.o\
    MPU6050.o\
    I2cPort.o\
    MadgwickAHRS.o\
    main.o

all: imu

imu: $(OBJS)
	$(CXX) $(LDFLAGS) $(OBJS) -o imu

clean:
	rm *.o imu

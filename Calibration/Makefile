CXX ?= g++
NAVIO = ../../../Navio
INCLUDES = -I ../../..

all:
	$(CXX) $(INCLUDES) $(NAVIO)/MPU9250.cpp $(NAVIO)/LSM9DS1.cpp $(NAVIO)/Util.cpp $(NAVIO)/PWM.cpp $(NAVIO)/RCInput.cpp Calibration.cpp -o Calibration -lncurses -lrt -lpthread

clean:
	rm Calibration

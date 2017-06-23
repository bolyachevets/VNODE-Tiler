INSTALL_DIR = $(HOME)/Desktop/UBC/USRA/Interval_Method
CONFIG_FILE = $(INSTALL_DIR)/vnodelp/config/WindowsWithProfil

include $(CONFIG_FILE)

CXXFLAGS += -I$(INSTALL_DIR)/vnodelp/include\
	-I$(INSTALL_DIR)/vnodelp/FADBAD++
LDFLAGS  += -L$(INSTALL_DIR)/vnodelp/lib 

quadrotor_tile_generator_universal: 	quadrotor_tile_generator_universal.o tiler.o
	$(CXX) $(LDFLAGS) -o $@ quadrotor_tile_generator_universal.o tiler.o -lvnode $(LIBS)

tiler.o: tiler.cc tiler.h
	$(CXX) $(CXXFLAGS) -c tiler.cc -lvnode $(LIBS)

clean:
	@-$(RM) *.o core.* quadrotor_tile_generator_universal
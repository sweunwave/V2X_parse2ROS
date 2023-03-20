CXX ?= g++
AR ?= ar
RM ?= rm -f
LTO = -flto
ODIR = objs
SDIR = src

INC += -I ./include 
LIBS += -lstdc++ -lc
CXXFLAGS +=  -Wunused-variable -Wall -std=c++11

APP_NAME = obu_sample 	 

APP_OBJ =  main.o \
		  

APP_OBJS = $(patsubst %,$(ODIR)/%,$(APP_OBJ)) 
 
all: $(APP_NAME)  
  
$(APP_NAME) : $(APP_OBJS)
	$(CXX) -o $@ $(LDFLAGS) $(APP_OBJS) $(LIBS)
  
$(ODIR)/%.o: $(SDIR)/%.cpp
	$(CXX) $< -o $@ -c $(CXXFLAGS) $(LTO) $(INC) -pedantic

clean:  
	$(RM) ./objs/*.o $(APP_NAME)


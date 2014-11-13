#sigverse header
SIG_SRC  = $(SIGVERSE_PATH)/include/sigverse

# To be added to the Python path
# export PYTHONPATH=$PYTHONPATH:/home/raghav/SigverseDevelopment/examplesForPractice/MyWorld/finalproject/postAwakening/generalization

# Please add the following lie in your bashrc file. We want to add the lib/ directory of your boost installation.
# export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/pool/lib


#all: $(OBJS)

#compile
#./%.so: ./%.cpp
#	g++ -DCONTROLLER -DNDEBUG -DUSE_ODE -DdDOUBLE -I$(SIG_SRC) -I$(SIG_SRC)/comm/controller -fPIC -shared -o $@ $<

#clean:
#	rm ./*.so


CC		= gcc
CPP     = g++
AS		= as
LD		= g++
AR		= ar
RANLIB	= ranlib
OBJCOPY	= objcopy

# external libraries and headers

# change the next line to point to the location of your boost installation
EXTERN_DIR =    $(HOME)/lib
EXTERN_LIBDIR = $(EXTERN_DIR)/lib
EXTERN_INCDIR = $(EXTERN_DIR)/include
EXTERN_BINDIR = $(EXTERN_DIR)/bin

BOOST_PYTHON_LIB = $(EXTERN_LIBDIR)/libboost_python.a

INCDIRS = . 
INCDIRS += $(EXTERN_INCDIR)
# you may also need to change this directory if you want to pin to a specific
#   python version
INCDIRS += /usr/include/python2.7
INCDIRS += /usr/lib/python2.7/dist-packages/numpy/core/include

INCDIR = $(foreach dir, $(INCDIRS), -I$(dir))

WORKDIR= .

%.o: %.cpp
	$(CPP) $(INCDIR) $(CFLAGS) $(CPPFLAGS) -c $< -o $@ 

#specifying of object file
OBJS =   libControlPosition.so libStopMotion.so libMeasureDisplacement.so ControllerSample_11.so ControllerSample_12.so ControllerSample_13.so ControllerSample_21.so ControllerSample_22.so ControllerSample_23.so ControllerSample_31.so ControllerSample_32.so ControllerSample_33.so ControllerSample_41.so ControllerSample_42.so ControllerSample_43.so ControllerSample_51.so ControllerSample_52.so ControllerSample_53.so 
# OBJS =    libControlPosition.so ToolController.so libSegmentPositions.so libCalculateNormals.so libControlPosition.so libToolPlacement.so



all: $(OBJS)


libControlPosition.so:: ControlPosition.cpp
	g++ -DCONTROLLER -DNDEBUG -DUSE_ODE -DdDOUBLE  -I$(SIG_SRC) -I$(SIG_SRC)/comm/controller -I$(INCDIR) -fPIC -shared -o $@ $^ $(LDFLAGS) -L$(EXTERN_LIBDIR) -lboost_python -lpython2.7 

libApplyAction.so:: ApplyAction.cpp libControlPosition.so
	g++ -DCONTROLLER -DNDEBUG -DUSE_ODE -DdDOUBLE  -I$(SIG_SRC) -I$(SIG_SRC)/comm/controller -I$(INCDIR) -fPIC -shared -o $@ $^ $(LDFLAGS) -L$(EXTERN_LIBDIR) -L$(WORKDIR) -lboost_python -lpython2.7  -lControlPosition

libEntityMotionStatus.so:: EntityMotionStatus.cpp
	g++ -DCONTROLLER -DNDEBUG -DUSE_ODE -DdDOUBLE  -I$(SIG_SRC) -I$(SIG_SRC)/comm/controller -I$(INCDIR) -fPIC -shared -o $@ $^ $(LDFLAGS) -L$(EXTERN_LIBDIR)  -lboost_python -lpython2.7 


libResetEntity.so:: ResetEntity.cpp libControlPosition.so
	g++ -DCONTROLLER -DNDEBUG -DUSE_ODE -DdDOUBLE  -I$(SIG_SRC) -I$(SIG_SRC)/comm/controller -I$(INCDIR) -fPIC -shared -o $@ $^ $(LDFLAGS) -L$(EXTERN_LIBDIR) -L$(WORKDIR) -lboost_python -lpython2.7  -lControlPosition


libStopMotion.so:: StopMotion.cpp
	g++ -DCONTROLLER -DNDEBUG -DUSE_ODE -DdDOUBLE  -I$(SIG_SRC) -I$(SIG_SRC)/comm/controller -I$(INCDIR) -fPIC -shared -o $@ $^ $(LDFLAGS) -L$(EXTERN_LIBDIR) -lboost_python -lpython2.7 


libMeasureDisplacement.so:: MeasureDisplacement.cpp
	g++ -DCONTROLLER -DNDEBUG -DUSE_ODE -DdDOUBLE  -I$(SIG_SRC) -I$(SIG_SRC)/comm/controller -I$(INCDIR) -fPIC -shared -o $@ $^ $(LDFLAGS) -L$(EXTERN_LIBDIR) -lboost_python -lpython2.7 


ControllerSample_11.so :: ControllerSample_11.cpp libStopMotion.so libMeasureDisplacement.so libApplyAction.so libEntityMotionStatus.so libResetEntity.so
	g++ -DCONTROLLER -DNDEBUG -DUSE_ODE -DdDOUBLE  -I$(SIG_SRC) -I$(SIG_SRC)/comm/controller -I$(INCDIR) -fPIC -shared -o $@ $^ $(LDFLAGS) -L$(EXTERN_LIBDIR) -L$(WORKDIR) -lboost_python -lpython2.7  -lResetEntity  -lApplyAction  -lMeasureDisplacement -lStopMotion -lEntityMotionStatus


ControllerSample_12.so :: ControllerSample_12.cpp libStopMotion.so libMeasureDisplacement.so libApplyAction.so libEntityMotionStatus.so libResetEntity.so
	g++ -DCONTROLLER -DNDEBUG -DUSE_ODE -DdDOUBLE  -I$(SIG_SRC) -I$(SIG_SRC)/comm/controller -I$(INCDIR) -fPIC -shared -o $@ $^ $(LDFLAGS) -L$(EXTERN_LIBDIR) -L$(WORKDIR) -lboost_python -lpython2.7  -lResetEntity  -lApplyAction  -lMeasureDisplacement -lStopMotion -lEntityMotionStatus


ControllerSample_13.so :: ControllerSample_13.cpp libStopMotion.so libMeasureDisplacement.so libApplyAction.so libEntityMotionStatus.so libResetEntity.so
	g++ -DCONTROLLER -DNDEBUG -DUSE_ODE -DdDOUBLE  -I$(SIG_SRC) -I$(SIG_SRC)/comm/controller -I$(INCDIR) -fPIC -shared -o $@ $^ $(LDFLAGS) -L$(EXTERN_LIBDIR) -L$(WORKDIR) -lboost_python -lpython2.7  -lResetEntity  -lApplyAction  -lMeasureDisplacement -lStopMotion -lEntityMotionStatus


ControllerSample_21.so :: ControllerSample_21.cpp libStopMotion.so libMeasureDisplacement.so libApplyAction.so libEntityMotionStatus.so libResetEntity.so
	g++ -DCONTROLLER -DNDEBUG -DUSE_ODE -DdDOUBLE  -I$(SIG_SRC) -I$(SIG_SRC)/comm/controller -I$(INCDIR) -fPIC -shared -o $@ $^ $(LDFLAGS) -L$(EXTERN_LIBDIR) -L$(WORKDIR) -lboost_python -lpython2.7  -lResetEntity  -lApplyAction  -lMeasureDisplacement -lStopMotion -lEntityMotionStatus


ControllerSample_22.so :: ControllerSample_22.cpp libStopMotion.so libMeasureDisplacement.so libApplyAction.so libEntityMotionStatus.so libResetEntity.so
	g++ -DCONTROLLER -DNDEBUG -DUSE_ODE -DdDOUBLE  -I$(SIG_SRC) -I$(SIG_SRC)/comm/controller -I$(INCDIR) -fPIC -shared -o $@ $^ $(LDFLAGS) -L$(EXTERN_LIBDIR) -L$(WORKDIR) -lboost_python -lpython2.7  -lResetEntity  -lApplyAction  -lMeasureDisplacement -lStopMotion -lEntityMotionStatus


ControllerSample_23.so :: ControllerSample_23.cpp libStopMotion.so libMeasureDisplacement.so libApplyAction.so libEntityMotionStatus.so libResetEntity.so
	g++ -DCONTROLLER -DNDEBUG -DUSE_ODE -DdDOUBLE  -I$(SIG_SRC) -I$(SIG_SRC)/comm/controller -I$(INCDIR) -fPIC -shared -o $@ $^ $(LDFLAGS) -L$(EXTERN_LIBDIR) -L$(WORKDIR) -lboost_python -lpython2.7  -lResetEntity  -lApplyAction  -lMeasureDisplacement -lStopMotion -lEntityMotionStatus


ControllerSample_31.so :: ControllerSample_31.cpp libStopMotion.so libMeasureDisplacement.so libApplyAction.so libEntityMotionStatus.so libResetEntity.so
	g++ -DCONTROLLER -DNDEBUG -DUSE_ODE -DdDOUBLE  -I$(SIG_SRC) -I$(SIG_SRC)/comm/controller -I$(INCDIR) -fPIC -shared -o $@ $^ $(LDFLAGS) -L$(EXTERN_LIBDIR) -L$(WORKDIR) -lboost_python -lpython2.7  -lResetEntity  -lApplyAction  -lMeasureDisplacement -lStopMotion -lEntityMotionStatus


ControllerSample_32.so :: ControllerSample_32.cpp libStopMotion.so libMeasureDisplacement.so libApplyAction.so libEntityMotionStatus.so libResetEntity.so
	g++ -DCONTROLLER -DNDEBUG -DUSE_ODE -DdDOUBLE  -I$(SIG_SRC) -I$(SIG_SRC)/comm/controller -I$(INCDIR) -fPIC -shared -o $@ $^ $(LDFLAGS) -L$(EXTERN_LIBDIR) -L$(WORKDIR) -lboost_python -lpython2.7  -lResetEntity  -lApplyAction  -lMeasureDisplacement -lStopMotion -lEntityMotionStatus


ControllerSample_33.so :: ControllerSample_33.cpp libStopMotion.so libMeasureDisplacement.so libApplyAction.so libEntityMotionStatus.so libResetEntity.so
	g++ -DCONTROLLER -DNDEBUG -DUSE_ODE -DdDOUBLE  -I$(SIG_SRC) -I$(SIG_SRC)/comm/controller -I$(INCDIR) -fPIC -shared -o $@ $^ $(LDFLAGS) -L$(EXTERN_LIBDIR) -L$(WORKDIR) -lboost_python -lpython2.7  -lResetEntity  -lApplyAction  -lMeasureDisplacement -lStopMotion -lEntityMotionStatus


ControllerSample_41.so :: ControllerSample_41.cpp libStopMotion.so libMeasureDisplacement.so libApplyAction.so libEntityMotionStatus.so libResetEntity.so
	g++ -DCONTROLLER -DNDEBUG -DUSE_ODE -DdDOUBLE  -I$(SIG_SRC) -I$(SIG_SRC)/comm/controller -I$(INCDIR) -fPIC -shared -o $@ $^ $(LDFLAGS) -L$(EXTERN_LIBDIR) -L$(WORKDIR) -lboost_python -lpython2.7  -lResetEntity  -lApplyAction  -lMeasureDisplacement -lStopMotion -lEntityMotionStatus


ControllerSample_42.so :: ControllerSample_42.cpp libStopMotion.so libMeasureDisplacement.so libApplyAction.so libEntityMotionStatus.so libResetEntity.so
	g++ -DCONTROLLER -DNDEBUG -DUSE_ODE -DdDOUBLE  -I$(SIG_SRC) -I$(SIG_SRC)/comm/controller -I$(INCDIR) -fPIC -shared -o $@ $^ $(LDFLAGS) -L$(EXTERN_LIBDIR) -L$(WORKDIR) -lboost_python -lpython2.7  -lResetEntity  -lApplyAction  -lMeasureDisplacement -lStopMotion -lEntityMotionStatus


ControllerSample_43.so :: ControllerSample_43.cpp libStopMotion.so libMeasureDisplacement.so libApplyAction.so libEntityMotionStatus.so libResetEntity.so
	g++ -DCONTROLLER -DNDEBUG -DUSE_ODE -DdDOUBLE  -I$(SIG_SRC) -I$(SIG_SRC)/comm/controller -I$(INCDIR) -fPIC -shared -o $@ $^ $(LDFLAGS) -L$(EXTERN_LIBDIR) -L$(WORKDIR) -lboost_python -lpython2.7  -lResetEntity  -lApplyAction  -lMeasureDisplacement -lStopMotion -lEntityMotionStatus


ControllerSample_51.so :: ControllerSample_51.cpp libStopMotion.so libMeasureDisplacement.so libApplyAction.so libEntityMotionStatus.so libResetEntity.so
	g++ -DCONTROLLER -DNDEBUG -DUSE_ODE -DdDOUBLE  -I$(SIG_SRC) -I$(SIG_SRC)/comm/controller -I$(INCDIR) -fPIC -shared -o $@ $^ $(LDFLAGS) -L$(EXTERN_LIBDIR) -L$(WORKDIR) -lboost_python -lpython2.7  -lResetEntity  -lApplyAction  -lMeasureDisplacement -lStopMotion -lEntityMotionStatus


ControllerSample_52.so :: ControllerSample_52.cpp libStopMotion.so libMeasureDisplacement.so libApplyAction.so libEntityMotionStatus.so libResetEntity.so
	g++ -DCONTROLLER -DNDEBUG -DUSE_ODE -DdDOUBLE  -I$(SIG_SRC) -I$(SIG_SRC)/comm/controller -I$(INCDIR) -fPIC -shared -o $@ $^ $(LDFLAGS) -L$(EXTERN_LIBDIR) -L$(WORKDIR) -lboost_python -lpython2.7  -lResetEntity  -lApplyAction  -lMeasureDisplacement -lStopMotion -lEntityMotionStatus


ControllerSample_53.so :: ControllerSample_53.cpp libStopMotion.so libMeasureDisplacement.so libApplyAction.so libEntityMotionStatus.so libResetEntity.so
	g++ -DCONTROLLER -DNDEBUG -DUSE_ODE -DdDOUBLE  -I$(SIG_SRC) -I$(SIG_SRC)/comm/controller -I$(INCDIR) -fPIC -shared -o $@ $^ $(LDFLAGS) -L$(EXTERN_LIBDIR) -L$(WORKDIR) -lboost_python -lpython2.7  -lResetEntity  -lApplyAction  -lMeasureDisplacement -lStopMotion -lEntityMotionStatus










#$^ , $+	The names of all the prerequisites, with spaces between them. The value of $^ omits duplicate prerequisites, 
# while $+ retains them and preserves their order



clean:
	rm -rf *.so *.o 


#-I$(INCDIR) -I$(BOOST_PYTHON_LIB) -I$(LDFLAGS) $(CFLAGS) $(CPPFLAGS) -lpython2.7

#-lpython2.7
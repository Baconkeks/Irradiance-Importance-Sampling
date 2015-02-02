Mitsuba itself needs 4.4 GB disk space. These are all new or modified files:

platform.h replaces include/mitsuba/core/platform.h

myenvmap.h
nanoflann.hpp
nnkdtree.h
nnpointclout.h
are new, add to include / mitsuba

docs.xml replaces src/mtsgui/resources/docs.xml

renderjob.cpp replaces src/librender/renderjob.cpp

myIntegrator.cpp
myPathTracerAndPhotonMapper.cpp
myPhotonMapper.cpp
pathExplicit.cpp
are new, add to src/integrators
The final code is in myPathTracerAndPhotonMapper.cpp


SConscript replaces src/integrators/SConscript
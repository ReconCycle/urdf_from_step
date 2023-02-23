

# urdf_from_step

![alt text](./documentation/jsi-logo-1-150x150.png)
![alt text](./documentation/reconcycle-transparent.png)

## Description



## Instalation

### Docker

link

### Instalation from source

sudo apt-get install ninja-build swig

wget 'https://git.dev.opencascade.org/gitweb/?p=occt.git;a=snapshot;h=fecb042498514186bd37fa621cdcf09eb61899a3;sf=tgz' -O occt-fecb042.tar.gz

tar -zxvf occt-fecb042.tar.gz >> extracted_occt753_files.txt


mkdir occt-fecb042/build

cd occt-fecb042/build

sudo ninja install


https://github.com/tpaviot/pythonocc-core/blob/master/INSTALL.md

###
git clone https://github.com/tpaviot/pythonocc-core

cd pythonocc-core
###

wget 'https://github.com/tpaviot/pythonocc-core/archive/refs/tags/7.7.0.tar.gz' -O 7.7.0.tar.gz
 tar -zxvf 7.7.0.tar.gz 

cd pythonocc-core-7.7.0

mkdir cmake-build

cd cmake-build

cmake \
 -DOCE_INCLUDE_PATH=/opt/build/occt753/include/opencascade \
 -DOCE_LIB_PATH=/opt/build/occt753/lib \
 -DPYTHONOCC_BUILD_TYPE=Release \
 ..

if doesnt work::

cmake \
-DPython3_EXECUTABLE=/usr/bin/python3.8 \
 -DOCE_INCLUDE_PATH=/opt/build/occt753/include/opencascade \
 -DOCE_LIB_PATH=/opt/build/occt753/lib \
 -DPYTHONOCC_BUILD_TYPE=Release \
 ..



## Citations

OCC:

pythonOCC

Import asembly: heare and in code




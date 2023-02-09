
git clone git://github.com/tpaviot/pythonocc-core.git pythonocc
cd pythonocc
mkdir build
cd build
cmake -DOCE_INCLUDE_PATH=/usr/include/opencascade -DOCE_LIB_PATH=/usr/lib/x86_64-linux-gnu ..
make
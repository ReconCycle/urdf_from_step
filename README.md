

# urdf_from_step

![alt text](./documentation/jsi-logo-1-150x150.png)
![alt text](./documentation/reconcycle-transparent.png)

<img src="./documentation/jsi-logo-1-150x150.png" 
     alt="jsi" height="170"  style="float: left; margin-right: 10px;" >  
     
<img src="./documentation/reconcycle-transparent.png" 
     alt="reconcycle" height="70" style="float: left; margin-right: 10px;" >  


## About

This is ROS package for automated conversion of STEP models to URDF format. The program takes as input the STEP file of the desired robot or robot-like maschine and creates a new ROS package. The package created contains the URDF description, the STL mesh files required by URDF description, and the ROS launch file to load the data into the ROS control system. 

legacy code: [URDF CREATOR](https://github.com/ReconCycle/urdf_creator) 

### Title

The URDF file is generated in the following steps. First, the STEP file is loaded and its contents are analyzed using tools from the Open Cascade Technology (OCCT) library. The analysis looks for keywords such as "joint" and "link" in the part names or in the assembly names in the model design tree. The instances with these keywords in their names represent the corresponding "joint" and "link" building blocks of URDF. The remaining part names containing the keyword encode the connections between individual URDF elements and their names in the URDF file. Once these instances and their connections have been identified, the correct local transformation between them must be computed from the values of their base coordinate systems in the STEP file. The calculated local transformations are transformed accordingly into the coordinate system values of the "joint" and "link" URDF definitions. The instances that do not have keywords in their names represent geometric shapes. They are transformed into the STL mesh specified in the appropriate local coordinate system according to the given URDF tree structure. From the collected and computed URDF data, the XML in URDF format is created using the urdfdom parser library. Finally, everything is stored in a newly created ROS package.


## Example

In our development process, we used the Fusion 360 CAD program for STEP file creation, but any of the standard CAD programs could be used instead.



## Instalation



### Docker

Docker repository is [hier](https://github.com/ReconCycle/urdf-from-step-docker).

Pull prepared docker image

```bash
docker pull ghcr.io/reconcycle/urdf-from-step:latest
```

### Build from source



```bash
# comment
sudo apt-get install ninja-build swig

wget 'https://git.dev.opencascade.org/gitweb/?p=occt.git;a=snapshot;h=fecb042498514186bd37fa621cdcf09eb61899a3;sf=tgz' -O occt-fecb042.tar.gz

tar -zxvf occt-fecb042.tar.gz >> extracted_occt753_files.txt


mkdir occt-fecb042/build

cd occt-fecb042/build

sudo ninja install


https://github.com/tpaviot/pythonocc-core/blob/master/INSTALL.md
```
###


```bash
git clone https://github.com/tpaviot/pythonocc-core

cd pythonocc-core
```

###

```bash
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
```

if doesnt work::

cmake \
-DPython3_EXECUTABLE=/usr/bin/python3.8 \
 -DOCE_INCLUDE_PATH=/opt/build/occt753/include/opencascade \
 -DOCE_LIB_PATH=/opt/build/occt753/lib \
 -DPYTHONOCC_BUILD_TYPE=Release \
 ..



## Citations

* OCC:

* pythonocc: Thomas Paviot. (2022). pythonocc (7.7.0). Zenodo. https://doi.org/10.5281/zenodo.3605364

Import asembly: heare and in code



<img src="./documentation/rosin_eu_flag.jpg" 
     alt="eu_flag" height="45" align="left" >  

This project has received funding from the European Union’s Horizon 2020 research and innovation programme under grant agreement No. 871352. 

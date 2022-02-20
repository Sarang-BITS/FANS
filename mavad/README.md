# mavad

## Branch for T formation using only NS3

## Prerequisites:
### Follow the documentation for guidelines
- Install NS3 using Cmake from the following [link](https://github.com/Gabrielcarvfer/NS3)  
---
## Steps:
```bash
cd NS3 #The ns3 folder cloned in first installation guide
git clone https://github.com/ojitmehta123/mavad.git
```
### 1. Add the following in the CMakeLists.txt in NS3 folder:
```cmake
#Build mavad scripts
add_subdirectory(mavad)
``` 
### 2. Edit the following:
```cmake
option(NS3_EMU "Build with emu support" ON)
set(NS3_EMU ON)

```

### 3. Also examples, utils folder can be commented as they are not necessary to be built.

### 4. Now build the scripts.

### 5. Run the Executable mavad_main in the build/bin folder
---



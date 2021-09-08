## Description

High Quality DeNoise 3D is an AviSynth port of the MPlayer filter of the same name. It performs a 3-way low-pass filter, which can completely remove high-frequency noise while minimizing blending artifacts.

High bit depth support ported from the ffmpeg plugin.

### Requirements:

- AviSynth 2.60 / AviSynth+ 3.4 or later

- Microsoft VisualC++ Redistributable Package 2022 (can be downloaded from [here](https://github.com/abbodi1406/vcredist/releases))

### Usage:

```
hqdn3d (clip input, float "ls", float "cs", float "lt", float "ct", int "restart", int "y", int "u", int "v")
```

### Parameters:

- input\
    A clip to process.\
    It must be in YUV 8..16-bit planar format.
    
- ls\
    Spatial luma strength.\
    Increasing the value will improve the smoothing but may overblur.\
    Anything above about 10 is probably not a good idea.\
    Must be between 0.0..255.0.\
    Default: 4.0.
    
- cs\
    Spatial chroma strength.\
    Increasing the value will improve the smoothing but may overblur.\
    Anything above about 10 is probably not a good idea.\
    Must be between 0.0..255.0.\
    Default: 3.0 \* ls / 4.0.

- lt\
    Luma temporal strength.\
    Increasing the values will improve the smoothing but may cause ghosting.\
    Anything above about 13 is probably not a good idea.\
    Must be between 0.0..255.0.\
    Default: 6.0 \* ls / 4.0.
    
- ct\
    Chroma temporal strength.\
    Increasing the values will improve the smoothing but may cause ghosting.\
    Anything above about 13 is probably not a good idea.\
    Must be between 0.0..255.0.\
    Default: lt \* cs / ls.
    
- restart\
    Whenever a frame is requested out of order, restart filtering this many frames before.\
    While seeking still slightly affects the content of the frames returned, this should reduce the disturbance to an unnoticeable level.\
    Must be non-negative value.
    Default: max(2, 1 + max(lt, ct)).
    
- y, u, v\
    Planes to process.\
    1: Return garbage.\
    2: Copy plane.\
    3: Process plane.\
    Default: y = u = v = 3.
    
### Building:

- Windows\
    Use solution files.

- Linux
    ```
    Requirements:
        - Git
        - C++11 compiler
        - CMake >= 3.16
    ```
    ```
    git clone https://github.com/Asd-g/AviSynth-hqdn3d && \
    cd AviSynth-hqdn3d && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j$(nproc) && \
    sudo make install
    ```

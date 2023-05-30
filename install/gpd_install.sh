#!/bin/bash
# install GPD
git clone https://github.com/atenpas/gpd
cd gpd
mkdir build && cd build
cmake ..
make -j6
sudo make install

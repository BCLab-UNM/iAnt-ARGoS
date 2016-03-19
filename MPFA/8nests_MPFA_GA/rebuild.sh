cd build
rm -r *
cmake -DCMAKE_BUILD_TYPE=Release -DARGOS_BUILD_NATIVE=ON ..
make
cd ..

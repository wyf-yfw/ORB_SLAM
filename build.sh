echo "Configuring and building Thirdparty/DBoW2 ..."

cd Thirdparty/Pangolin
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

cd Thirdparty/eigen
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

cd Thirdparty/DBoW2
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j
sudo make install

cd ../../Sophus

echo "Configuring and building Thirdparty/Sophus ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

cd ../../../

echo "Uncompress vocabulary ..."

cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..

echo "Configuring and building ORB_SLAM3 ..."

sudo apt install libpython2.7-dev

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4

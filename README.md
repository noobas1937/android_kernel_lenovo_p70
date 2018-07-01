# Lenovo P70-A nougat kernel. Based on Zormax sources.
# How to build
    cd ~/p70
    make ARCH=arm64 P70_defconfig
    make ARCH=arm64 CROSS_COMPILE=/home/YourSelf/p70/aarch64-linux-android-4.9/bin/aarch64-linux-android- -j8
# Clean
    make clean && make mrproper

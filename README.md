export PATH=~/Your_Toolchain_PATH/
for example /alps/prebuilts/gcc/linux-x86/arm/arm-linux-androideabi-4.6/bin
Build Command:
kernel
======
cd kernel directory
TARGET_PRODUCT=huawei82_wet_jb5 MTK_ROOT_CUSTOM=../mediatek/custom/ MTK_PATH_SOURCE=../mediatek/kernel MTK_PATH_PLATFORM=../mediatek/platform/mt6582/kernel ARCH_MTK_PLATFORM=mt6582 make 

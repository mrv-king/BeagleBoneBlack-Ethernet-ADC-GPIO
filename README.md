# BeagleBoneBlack-Ethernet-ADC-GPIO

This is a bare metal application, designed for BeagleBone Black. The application does reading of 3 ADC channels (AD0, AD1 and AD2), generation of IEC 61850-9-2 SV frame that includes the analog data, and control of GPIO (pin 'P8_11'). This example is based on libraries and binary files, designed by Texas Instruments as a part of StarterWare toolkit.

# Files and Resources
1. [StarterWare User Guide 02 00 01](https://usermanual.wiki/Document/UserGuide02000101.45177949/html)
2. [Bare Metal on the BeagleBone (Black and Green)](https://opencoursehub.cs.sfu.ca/bfraser/grav-cms/ensc351/guides/files/BareMetalGuide.pdf)
3. [Technical Reference Manual](https://www.ti.com/lit/ug/spruh73q/spruh73q.pdf)

# How to build
1. [Download](https://launchpad.net/gcc-arm-embedded/4.7/4.7-2012-q4-major) and install gcc-arm-none-eabi toolchain.
2. Add "toolchain_parent_folder\arm_none_eabi\10 2021.10\bin" to your "path" variable (here "toolchain_parent_folder" is a folder path where you have installed gcc-arm-none-eabi toolchain).
3. Go to "parent_folder\BeagleBoneBlack-Ethernet-ADC-GPIO-main\BeagleBoneBlack-Ethernet-ADC-GPIO-main\build\armv7a\gcc\am335x\beaglebone\61850_mu_mimic" where "parent_folder" is a folder path where you have upploaded "BeagleBoneBlack-Ethernet-ADC-GPIO".
4. run "make clean" (run this command every time when you have made changes in 61850_mu_mimic.c file) 
5. run "make"
6. The binary file of interest is called "61850_mu_mimic.bin" (the one that should be upploaded in the BeagleBone) will be [here](https://github.com/mrv-king/BeagleBoneBlack-Ethernet-ADC-GPIO/tree/main/binary/armv7a/gcc/am335x/beaglebone/61850_mu_mimic/Release)


⋅⋅⋅ In case if you want to make changes in the designed application:
7. Open "parent_folder\BeagleBoneBlack-Ethernet-ADC-GPIO-main\BeagleBoneBlack-Ethernet-ADC-GPIO-main\examples\beaglebone\61850_mu_mimic\61850_mu_mimic.c" and make necessary changes.
8. Do not forget to save document before runnig "make".
9. Run "make clean" and then "make" from "parent_folder\BeagleBoneBlack-Ethernet-ADC-GPIO-main\BeagleBoneBlack-Ethernet-ADC-GPIO-main\build\armv7a\gcc\am335x\beaglebone\61850_mu_mimic" folder.

# How to run on Beaglebone

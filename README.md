==================================
evaluate_utias_dataset
==================================

The GT evaluation and benchamrking package that is supposed to be executed with mhls_utias_dataset


==================================
Pre-requisites
==================================
1. Make sure you have ros indigo installed
2. Make sure you have the following package installed and the message files provided in this package built.

    1. https://github.com/aamirahmad/utiasdata_to_rosbags
    2. https://github.com/aamirahmad/mhls_utias_dataset
    
3. Make sure you have latest versions of the following libraries installed

   1. boost
   2. eigen
   3. g2o
   
4. You might have to add the location of installed g2o libraries in the CMakeLists.txt of this package

==================================
Instructions to use this package.
==================================
0. Note that running this package makes sense only with the mhls_utias_dataset package.
1. The best way to use this package is to uncomment the node line for this package in any of the launch files provided with the mhls_utias_dataset package

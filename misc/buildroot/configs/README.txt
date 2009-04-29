AVAILABLE CONFIGURATIONS
^^^^^^^^^^^^^^^^^^^^^^^^

arm-defconfig
	Builds an ARM toolchain using gcc 3.4.5

arm7tdmi-defconfig-4.2.4
arm920t-defconfig-4.2.4
arm926t-defconfig-4.2.4
	Builds an ARM toolchain using gcc 4.2.4.  This configuration
	builds both gcc and g++.  There are thred versions: one for 
	arm7tdmi (armv4t) , arm920t (armv4t) and arm926t (arv5t) because
	of differences in the way that soft floating is handled in between
	the armv4t and arm5t architectures.

	NOTE: At present, there are issues with some of the binutils
	programs (arm-elf-objcopy in particular) that cause Floating
	point exceptions when trying to build NuttX on certain platforms,
	specifically, those that execute from FLASH and use arm-elf-objcopy
	to relocate the .data section into flash.  This bug is probably
	due to improperly positioned sections and can probably fixed
	by changing the architecture's ld.script file.

cortexm3-defconfig-4.3.3
	Builds an ARM toolchain for the Cortex-M3 using gcc 4.3.3.
	This configuration builds both gcc and g++.

	NOTE:  This configuration is untested as of this writing.

bfin-defconfig-4.2.4
	Builds an Blackfin toolchain using gcc 4.2.4

h8300_config
	Builds an H8/300 toolchain using gcc 3.4.5

m68hc11-config
	Builds an hc11 toolchain using gcc 3.4.5

m68k-config
	Builds an M68K toolchain using gcc 3.4.5

sh-defconfig
	Builds an SH-1/2 toolchain using gcc 3.4.5

GENERAL BUILD STEPS
^^^^^^^^^^^^^^^^^^^

1. CD to the correct directory.

   Change to the directory just above the NuttX installation.  If <nuttx-dir> is
   where NuttX is installed, then cd to <nuttx-dir>/..

2. Get and Install the buildroot Module

   a. Using a release tarball:

     cd <nuttx-dir>/..
     Download the appropriate buildroot package.
     unpack the buildroot package
     rename the directory to buildroot

   b. Using CVS
   
     Check out the misc/buildroot module. CVS checkout instructions:

        cvs -d:pserver:anonymous@nuttx.cvs.sourceforge.net:/cvsroot/nuttx login
        cvs -z3 -d:pserver:anonymous@nuttx.cvs.sourceforge.net:/cvsroot/nuttx co -P misc/buildroot

     Move the buildroot Source Tree and create the archive directory

        mv misc/buildroot .

   Make the archive directory:
  
     mkdir archive

   The <nuttx-dir>/../buildroot is where the toolchain is built;
   The <nuttx-dir>/../archive directory is where toolchain sources will be downloaded.

3. Make sure that NuttX is configured

     cd <nuttx-dir>/tools
     ./configure.sh <nuttx-configuration>
     
4. Configure and Make the buildroot

     cd buildroot
     cp configs/<config-file> .config
     make oldconfig
     make

   This will download the large source packages for the toolchain and build the toolchain.
   The resulting binaries will be under buildroot/build_<arch>.  There will also be a
   large build directory called something like toolchain_build_<arch>; this directory
   can be removed once the build completes successfully.

   Where <config-file> is one of the configuration files listed above and <arch> is an
   archtecture name.  Examples: build_m32c, build_arm_nofpu, etc.

Cygwin GCC BUILD NOTES
^^^^^^^^^^^^^^^^^^^^^^
   On Cygwin, the buildroot 'make' command will fail with an error like:

   "...
      build/genchecksum cc1-dummy > cc1-checksum.c
      opening cc1-dummy: No such file or directory
   ..."

   This is caused because on Cygwin, host executables will be generated with the extension .exe
   and, apparently, the make variable "exeext" is set incorrectly.  A work around after the
   above occurs is:

      cd toolchain_build_<arch>/gcc-4.2.4-build/gcc	# Go to the directory where error occurred
      mv cc1-dummy.exe cc1-dummy			# Rename the executable without .exe
      rm cc1-checksum.c					# Get rid of the bad generated file

   Then resume the buildroot make:

      cd -						# Back to the buildroot make directory
      make						# Restart the build

   If you build g++, you will see another similar error:

   ...
      build/genchecksum cc1plus-dummy > cc1plus-checksum.c
      opening cc1plus-dummy: No such file or directory
   ...

   The fix is similar:

      cd toolchain_build_<arch>/gcc-4.2.4-build/gcc	# Go to the directory where error occurred
      mv cc1plus-dummy.exe cc1plus-dummy		# Rename the executable without .exe
      rm cc1plus-checksum.c				# Get rid of the bad generated file

   Then resume the buildroot make:

      cd -						# Back to the buildroot make directory
      make						# Restart the build
   

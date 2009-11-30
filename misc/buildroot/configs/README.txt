AVAILABLE CONFIGURATIONS
^^^^^^^^^^^^^^^^^^^^^^^^

arm-defconfig
	Builds an ARM toolchain using gcc 3.4.5

arm7tdmi-defconfig-4.2.4
arm920t-defconfig-4.2.4
arm926t-defconfig-4.2.4
	Builds an ARM toolchain using gcc 4.2.4.  This configuration
	builds both gcc and g++.  There are three versions: one for 
	arm7tdmi (armv4t), arm920t (armv4t) and arm926t (arv5t) because
	of differences in the way that soft floating is handled in between
	the armv4t and arm5t architectures.

	NOTE: The newer versions of GCC generate new sections and can
	cause some problems for NuttX configurations developed under older
	toolchains.  In particular, arm-elf-objcopy may fail with strange
	errors.  If this occurs, try adding the following arguments to the
	arm-elf-objcopy command "-R .note -R .note.gnu.build-id -R .comment"

	This logic is several configuration Make.defs files:

	HOSTOS			=  ${shell uname -o}

	ARCHCCVERSION		= ${shell $(CC) -v 2>&1 | sed -n '/^gcc version/p' | sed -e 's/^gcc version \([0-9\.]\)/\1/g' -e 's/[-\ ].*//g' -e '1q'}
	ARCHCCMAJOR		= ${shell echo $(ARCHCCVERSION) | cut -d'.' -f1}

	ifeq ($(ARCHCCMAJOR),4)
	ifneq ($(HOSTOS),Cygwin)
	OBJCOPYARGS		= -R .note -R .note.gnu.build-id -R .comment
	endif
	endif

	This change probably applies to other architectures as well (?)

arm920t-defconfig-4.3.3
arm7tdmi-defconfig-4.3.3
	Builds an ARM toolchain using gcc 4.3.3.  These configurations
	builds both gcc and g++ for the arm7tdmi (armv4t) or the arm920t
	(armv4t).  These are udates to *-defconfig-4.2.4 (see notes above).

avr-defconfig-4.3.3
	Builds an AVR toolchain using gcc 4.3.3.  This configuration
	builds both gcc and g++ for the AVR (armv4t). This toolchain
	is intended to support the NuttX ATmega128 port.

cortexm3-defconfig-4.3.3
	Builds an ARM toolchain for the Cortex-M3 using gcc 4.3.3.
	This configuration builds gcc, g++ and the NXFLAT toolchain.

cortexm3-defconfig-nxflat
	This configuration build an NXFLAT toolchain (only) for
	use with the Cortex-M3

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

1. Configure your host machine.  You host PC should have a relatively complete
   C development environment.  I don't have a full list of the package requirements.
   The later tool chains also require GMP and MPRF development packages or the
   build will fail with errors like:

    "configure: error: Building GCC requires GMP 4.1+ and MPFR 2.3.0+. ...
     Copies of these libraries' source code can be found at their respective
     hosting sites as well as at ftp://gcc.gnu.org/pub/gcc/infrastructure/.
     See also http://gcc.gnu.org/install/prerequisites.html for additional info.
     If you obtained GMP and/or MPFR from a vendor distribution package, make
     sure that you have installed both the libraries and the header files.
     They may be located in separate packages."

   You should try your package manager for whatever Linux version you are using
   first.  The header files are normally included in versions of the packages that
   have "-devel" in the package name.

2. CD to the correct directory.

   Change to the directory just above the NuttX installation.  If <nuttx-dir> is
   where NuttX is installed, then cd to <nuttx-dir>/..

3. Get and Install the buildroot Module

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

4. Make sure that NuttX is configured

     cd <nuttx-dir>/tools
     ./configure.sh <nuttx-configuration>

5. Configure and Make the buildroot

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

   Cygwin normally creates a /home directory with your Windows user name.  Unfortunately,
   that could very likely include spaces.  In that case, the Cygwin build will have
   lots of problems.  Here is how I worked around that:

   - I created a /home/buildroot directory and copied buildroot to that location
     (/home/build/buildroot/buildroot)
   - I have the archives directory at /home/buildroot/archives
   - And a symbolic link to the nuttx build directory at /home/buildroot/nuttx

   With those workarounds, the buildroot will build.  However, you will also need
   to either edit the setenv.sh file to reference this new location, or else move
   resulting build diectory.

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
   

#!/bin/sh
#set -x

WD=`pwd`
VERSION=$1

TAR="tar cvf"
ZIP=gzip

# Make sure we know what is going on

if [ -z ${VERSION} ] ; then
   echo "You must supply a version like xx.yy.zz as a parameter"
   exit 1;
fi

# Find the directory we were executed from and where we expect to
# see the directory to tar up

MYNAME=`basename $0`
BRNAME=buildroot-${VERSION}

if [ -x ${WD}/${MYNAME} ] ; then
   MISCDIR=`dirname ${WD}`
else
   if [ -x ${WD}/${BRNAME}/${MYNAME} ] ; then
     MISCDIR=${WD}
   else
     echo "You must cd into the misc/ or misc/${BRNAME}/ directory to execute this script."
     exit 1
   fi
fi

# Get the path to the parent directory

SUBDIR=`basename ${MISCDIR}`/${BRNAME}
PARENT=`dirname ${MISCDIR}`

# The name of the directory must match the version number

cd ${PARENT} || \
   { echo "Failed to cd to ${PARENT}" ; exit 1 ; }

if [ ! -d ${SUBDIR} ] ; then
   echo "${PARENT}/${SUBDIR} does not exist!"
   exit 1
fi

TAR_NAME=${BRNAME}.tar
ZIP_NAME=${TAR_NAME}.gz

# Prepare the buildroot directory -- Remove editor garbage

find ${SUBDIR} -name '*~' -exec rm -f '{}' ';' || \
      { echo "Removal of emacs garbage failed!" ; exit 1 ; }
find ${SUBDIR} -name '*.swp' -exec rm -f '{}' ';' || \
      { echo "Removal of VI garbage failed!" ; exit 1 ; }

# Remove garbage directories that creap into CVS because the
# are obsoleted or bad imports

oldlist="\
  toolchain/binutils/2.14.90.0.8\
  toolchain/binutils/2.15\
  toolchain/binutils/2.15.94.0.2.2\
  toolchain/binutils/2.16.1\
  toolchain/binutils/2.16.90.0.3\
  toolchain/binutils/2.16.91.0.5\
  toolchain/binutils/2.16.91.0.6\
  toolchain/binutils/2.16.91.0.7\
  toolchain/binutils/2.17.50.0.10\
  toolchain/binutils/2.17.50.0.2\
  toolchain/binutils/2.17.50.0.3\
  toolchain/binutils/2.17.50.0.4\
  toolchain/binutils/2.17.50.0.5\
  toolchain/binutils/2.17.50.0.6\
  toolchain/binutils/2.17.50.0.7\
  toolchain/binutils/2.17.50.0.8\
  toolchain/binutils/2.17.50.0.9\
  toolchain/binutils/2.19.1/.svn\
  toolchain/gcc/3.3.5\
  toolchain/gcc/3.3.6\
  toolchain/gcc/3.4.2\
  toolchain/gcc/3.4.3\
  toolchain/gcc/3.4.4\
  toolchain/gcc/3.4.5\
  toolchain/gcc/4.0.0\
  toolchain/gcc/4.0.1\
  toolchain/gcc/4.0.2\
  toolchain/gcc/4.0.3\
  toolchain/gcc/4.0.4\
  toolchain/gcc/4.1.0\
  toolchain/gcc/4.1.1\
  toolchain/gcc/4.1.2\
  toolchain/gcc/4.2\
  toolchain/gdb/6.2.1\
  toolchain/gdb/6.4\
  toolchain/gdb/6.5\
"

for dir in $oldlist; do
	echo "Removing ${SUBDIR}/$dir"
	rm -rf ${SUBDIR}/$dir
done

# Remove any previous tarballs

if [ -f ${TAR_NAME} ] ; then
   echo "Removing ${TAR_NAME}"
   rm -f ${TAR_NAME} || \
      { echo "rm ${TAR_NAME} failed!" ; exit 1 ; }
fi

if [ -f ${ZIP_NAME} ] ; then
   echo "Removing ${ZIP_NAME}"
   rm -f ${ZIP_NAME} || \
      { echo "rm ${ZIP_NAME} failed!" ; exit 1 ; }
fi

# Then zip it

${TAR} ${TAR_NAME} ${SUBDIR} || \
      { echo "tar of ${TAR_NAME} failed!" ; exit 1 ; }
${ZIP} ${TAR_NAME} || \
      { echo "zip of ${TAR_NAME} failed!" ; exit 1 ; }

cd ${WD}

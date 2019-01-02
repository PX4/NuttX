pipeline {
  agent none
  stages {

    stage('Build') {

      parallel {

        stage('imxrt1050-evk/libcxxtest') {
          agent {
            docker {
              image 'px4io/px4-dev-nuttx:2018-11-25'
              args '-e CCACHE_BASEDIR=$WORKSPACE -v ${CCACHE_DIR}:${CCACHE_DIR}:rw'
            }
          }
          steps {
            sh 'export'
            sh 'git clean -ff -x -d .'
            sh 'ccache -z'
            sh 'git clone --branch nuttx-wip https://github.com/PX4/libcxx.git'
            sh 'cd libcxx && ./install.sh ${WORKSPACE}/'
            sh 'git clone --branch master --depth 1 https://github.com/PX4/NuttX-apps.git'
            sh './tools/configure.sh -l -a NuttX-apps imxrt1050-evk/libcxxtest'
            sh 'make --no-print-directory --quiet'
            sh 'ccache -s'
          }
          post {
            always {
              sh 'git clean -ff -x -d .'
            }
          }
        }

        stage('imxrt1060-evk/libcxxtest') {
          agent {
            docker {
              image 'px4io/px4-dev-nuttx:2018-11-25'
              args '-e CCACHE_BASEDIR=$WORKSPACE -v ${CCACHE_DIR}:${CCACHE_DIR}:rw'
            }
          }
          steps {
            sh 'export'
            sh 'git clean -ff -x -d .'
            sh 'ccache -z'
            sh 'git clone --branch nuttx-wip https://github.com/PX4/libcxx.git'
            sh 'cd libcxx && ./install.sh ${WORKSPACE}/'
            sh 'git clone --branch master --depth 1 https://github.com/PX4/NuttX-apps.git'
            sh './tools/configure.sh -l -a NuttX-apps imxrt1060-evk/libcxxtest'
            sh 'make --no-print-directory --quiet'
            sh 'ccache -s'
          }
          post {
            always {
              sh 'git clean -ff -x -d .'
            }
          }
        }

        stage('stm32f4discovery/testlibcxx') {
          agent {
            docker {
              image 'px4io/px4-dev-nuttx:2018-11-25'
              args '-e CCACHE_BASEDIR=$WORKSPACE -v ${CCACHE_DIR}:${CCACHE_DIR}:rw'
            }
          }
          steps {
            sh 'export'
            sh 'git clean -ff -x -d .'
            sh 'ccache -z'
            sh 'git clone --branch nuttx-wip https://github.com/PX4/libcxx.git'
            sh 'cd libcxx && ./install.sh ${WORKSPACE}/'
            sh 'git clone --branch master --depth 1 https://github.com/PX4/NuttX-apps.git'
            sh './tools/configure.sh -l -a NuttX-apps stm32f4discovery/testlibcxx'
            sh 'make --no-print-directory --quiet'
            sh 'ccache -s'
          }
          post {
            always {
              sh 'git clean -ff -x -d .'
            }
          }
        }

      } // parallel
    } // stage Build

  }
  environment {
    CCACHE_DIR = '/tmp/ccache'
  }
  options {
    buildDiscarder(logRotator(numToKeepStr: '5'))
    timeout(time: 60, unit: 'MINUTES')
  }
}


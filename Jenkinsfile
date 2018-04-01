pipeline {
  agent none
  stages {

    stage('Build') {
      steps {
        script {
          def builds = [:]

          def docker_nuttx = "px4io/px4-dev-nuttx:2017-12-30"

          // stm32f4discovery
          // TODO: cxxtest, ipv6, netnsh, nxlines, rndis, testlibcxx, uavcan, usbmsc, winbuild
          for (def option in ["canard", "elf", "kostest", "nsh", "pm", "posix_spawn", "pseudoterm", "usbnsh", "xen1210"]) {
            def node_name = "stm32f4discovery/${option}"
            builds[node_name] = createBuildNode(docker_nuttx, "stm32f4discovery", option)
          }

          // stm32f103-minimum
          // TODO: jlx12864g
          for (def option in ["audio_tone", "buttons", "mcp2515", "nsh", "rfid-rc522", "rgbled", "usbnsh", "userled", "veml6070"]) {
            def node_name = "stm32f103-minimum/${option}"
            builds[node_name] = createBuildNode(docker_nuttx, "stm32f103-minimum", option)
          }

          // stm32f769i-disco
          for (def option in ["nsh", "nsh-ethernet"]) {
            def node_name = "stm32f769i-disco/${option}"
            builds[node_name] = createBuildNode(docker_nuttx, "stm32f769i-disco", option)
          }

          parallel builds
        } // script
      } // steps
    } // stage Builds

  }
  environment {
    CCACHE_DIR = '/tmp/ccache'
  }
  options {
    buildDiscarder(logRotator(numToKeepStr: '5'))
    timeout(time: 60, unit: 'MINUTES')
  }
}

def createBuildNode(String docker_repo, String board, String config) {
  return {
    node {
      docker.image(docker_repo).inside('-e CCACHE_BASEDIR=$WORKSPACE -v ${CCACHE_DIR}:${CCACHE_DIR}:rw') {
        stage("build") {
          sh('export')
          checkout scm
          sh('git clean -ff -x -d .')
          sh('ccache -z')
          sh('git clone --branch master --depth 1 https://github.com/PX4/NuttX-apps.git')
          sh('tools/configure.sh -l -a NuttX-apps ' + board + '/' + config)
          sh('make --no-print-directory --quiet')
          sh('ccache -s')
          sh('size nuttx')
        }
      }
    }
  }
}

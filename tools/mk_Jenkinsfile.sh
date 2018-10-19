#! /bin/bash

# change to configs directory
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
cd ${DIR}/../configs

echo "#!/usr/bin/env groovy

pipeline {
  agent none
  stages {

    stage('Build') {
      steps {
        script {

          def build_nodes = [:]

          def docker_images = [
            armhf: \"px4io/px4-dev-armhf:2019-07-29\",
            base: \"px4io/px4-dev-base-bionic:2019-07-29\",
            nuttx: \"px4io/px4-dev-nuttx:2019-07-29\"
          ]

"

echo "          def builds = ["


# loop over all boards
for board in *;
do

	if [ -d "${board}" ]; then

		# jenkinsfile entry for each board
		echo "            [
              board: \"${board}\",
              configs: ["


		for dir in ${board}/*;
		do
			if [ -f "${dir}/defconfig" ]; then
				config=$(basename ${dir})

				echo "                \"${config}\","

			fi
		done

		# each board end
		echo "                ]
            ],"


	fi

done








echo "          ]"


echo "

          for (def board_type = 0; board_type < builds.size(); board_type++) {

            build_nodes.put(
              builds[board_type].board,
              createBuildNode(docker_images.nuttx, builds[board_type].board, builds[board_type].configs)
            )

          }

        parallel build_nodes

        } // script
      } // steps
    } // stage Build

  } // stages
  environment {
    APPSDIR = 'NuttX-apps'
    CCACHE_DIR = '/tmp/ccache'
    CI = true
  }
  options {
    buildDiscarder(logRotator(numToKeepStr: '2', artifactDaysToKeepStr: '14'))
    timeout(time: 60, unit: 'MINUTES')
  }
}

def createBuildNode(String docker_image, String board, List<String> configs) {
  return {

    node {
      docker.withRegistry('https://registry.hub.docker.com', 'docker_hub_dagar') {
        docker.image(docker_image).inside('-e CCACHE_BASEDIR=\${WORKSPACE} -v \${CCACHE_DIR}:\${CCACHE_DIR}:rw') {
          stage(board) {
            try {
              sh('export')
              checkout(scm)
              sh('make distclean')
              sh('git fetch --tags')
              sh('git clone --branch pr-cmake --depth 1 https://github.com/PX4/NuttX-apps.git')
              sh('ccache -z')

              for (def cfg = 0; cfg < configs.size(); cfg++) {
                sh('make ' + board + '/' + configs[cfg] || true)
              }

              sh('ccache -s')
              sh('make sizes')
              archiveArtifacts(allowEmptyArchive: true, artifacts: 'build/*/*.elf, build/*/*.bin', fingerprint: true, onlyIfSuccessful: true)
            }
            catch (exc) {
              throw (exc)
            }
            finally {
              sh('make distclean')
            }
          }
        }
      }
    }
  }
}
"

#!groovy

node('docker') {
  step([$class: 'StashNotifier'])

  try {
    checkout scm

    docker.build('fci-docs-ci-worker').inside {
      withEnv(["HOME=${env.WORKSPACE}"]) {
        sh 'npm install eclint@2.8.1'
        sh 'pip3 install --user --upgrade -r requirements.txt'
        sh 'make clean'

        stage('Build HTML') {
          sh 'make html'
          publishHTML([allowMissing: false,
                       alwaysLinkToLastBuild: false,
                       keepAll: true,
                       reportDir: 'build/html',
                       reportFiles: 'index.html',
                       reportName: 'FCI Documentation'])
        }

        stage('Run linter') {
          sh 'node $(npm bin)/eclint check source/*.rst'
          sh 'make linkcheck'
        }
      }
    }

    currentBuild.result = 'SUCCESS'
  } catch (e) {
    currentBuild.result = 'FAILED'
    throw e;
  } finally {
    step([$class: 'StashNotifier'])
  }
}

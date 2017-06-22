#!groovy

node {
  step([$class: 'StashNotifier'])

  try {
    stage('Checkout') {
      checkout scm
      make clean
    }

    stage('Build HTML') {
      sh 'make html'
      dir('build') {
        sh 'tar cfz research-interface.tar.gz html'
        archive 'research-interface.tar.gz'
        publishHTML([allowMissing: false,
                     alwaysLinkToLastBuild: false,
                     keepAll: true,
                     reportDir: 'html',
                     reportFiles: 'index.html',
                     reportName: 'User Documentation'])
      }
    }

    stage('Build PDF') {
      sh 'make latexpdf'
      archive 'build/latex/research-interface.pdf'
    }

    stage('Run linter') {
      sh 'npm install eclint'
      sh 'node $(npm bin)/eclint check source/*.rst'
      // TODO(FWA): run 'make linkcheck'
    }

    currentBuild.result = 'SUCCESS'
  } catch (e) {
    currentBuild.result = 'FAILED'
    throw e;
  } finally {
    step([$class: 'StashNotifier'])
  }
}

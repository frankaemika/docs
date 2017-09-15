#!groovy

node {
  step([$class: 'StashNotifier'])

  try {
    stage('Checkout') {
      checkout scm
      sh 'make clean'
    }

    stage('Build HTML') {
      sh 'make html'
      dir('build') {
        sh 'tar cfz fci.tar.gz html'
        archive 'fci.tar.gz'
        publishHTML([allowMissing: false,
                     alwaysLinkToLastBuild: false,
                     keepAll: true,
                     reportDir: 'html',
                     reportFiles: 'index.html',
                     reportName: 'FCI Documentation'])
      }
    }

    stage('Build PDF') {
      sh 'make latexpdf'
      archive 'build/latex/fci.pdf'
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

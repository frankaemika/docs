#!groovy

node('python3 && nodejs') {
  step([$class: 'StashNotifier'])

  try {
    stage('Prepare') {
      checkout scm
      sh 'npm install eclint'
      sh 'pip3 install --user -r requirements.txt'
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

    stage('Run linter') {
      sh 'node $(npm bin)/eclint check source/*.rst'
      sh 'make linkcheck'
    }

    currentBuild.result = 'SUCCESS'
  } catch (e) {
    currentBuild.result = 'FAILED'
    throw e;
  } finally {
    step([$class: 'StashNotifier'])
  }
}

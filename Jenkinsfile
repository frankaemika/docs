#!groovy

node {
  step([$class: 'StashNotifier'])

  try {
    stage('Checkout') {
      checkout scm
    }

    stage('Build documentation') {
      sh 'make html'
      sh 'make latexpdf'
    }

    stage('Pack HTML documentation') {
      dir('build') {
        sh 'tar cfz libfranka.tar.gz html'
      }
    }

    stage('Archive results') {
      archive 'build/libfranka.tar.gz, build/latex/libfranka.pdf'
      publishHTML([allowMissing: false,
                   alwaysLinkToLastBuild: false,
                   keepAll: true,
                   reportDir: 'build/html',
                   reportFiles: 'index.html',
                   reportName: 'User Documentation'])
    }
    currentBuild.result = 'SUCCESS'
  } catch (e) {
    currentBuild.result = 'FAILED'
    throw e;
  } finally {
    step([$class: 'StashNotifier'])
  }
}

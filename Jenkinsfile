#!/usr/bin/env groovy
// See: https://github.com/jenkinsci/pipeline-examples/blob/master/docs/BEST_PRACTICES.md
// See: https://jenkins.io/doc/book/pipeline/
pipeline {
    agent none
    options {
        timeout(time: 1, unit: 'HOURS')
        buildDiscarder(logRotator(numToKeepStr: '30', daysToKeepStr: '30'))
        skipDefaultCheckout()
    }
    stages {
        stage("Jenkins environment injection") {
            agent {
                label 'docker || docker-drive-agx-ubuntu'
            }
            steps {
                populateEnv()
            }
        }
        stage("Kill off any older (running) builds") {
            when {
                environment name: 'IS_RELEASE_BRANCH', value: 'false'
            }
            steps {
                script {
                    killOldBuilds()
                }
            }
        }
        // builds for different platforms are running concurrently
        // be careful for potential race conditions when updating global variables like env.XXX, even when updating them in different boxes
        // if we only want a variable to be available across stages within platform but not across platforms,
        // we may use stage level environment variables as shown below
        stage('Parallel processing') {
            matrix {
                // define configurations for parallel runs
                axes {
                    axis {
                        name 'PLATFORM'
                        // axis so far can only support string literal and thus cannot use environment env.PLATFORM_X86_LINUX
                        values 'x86_linux', 'arm64_linux'
                    }
                }
                environment {
                    // please include all platform-specific treatments here in one place
                    // also please use stage level environment variables to avoid race conditions due to matrix parallelism
                    // ideally things after this block are platform agnostic

                    // override base image to use based on a given platform
                    BASE_IMAGE = "${PLATFORM == env.PLATFORM_X86_LINUX ? env.BASE_IMAGE : env.BASE_IMAGE_ARM}"

                    // override ROS_HOME as different boxes may be configured with different jenkins remote
                    // directories and thus different workspaces, which will lead to different ROS_HOME
                    ROS_HOME = "${env.WORKSPACE}@tmp/.ros"
                }
                agent {
                    // select qualified boxes to build a given platform
                    label "${PLATFORM == env.PLATFORM_X86_LINUX ? "docker" : "docker-drive-agx-ubuntu"}"
                }
                stages {
                    stage("Cleanup") {
                        steps {
                            deleteDir()
                        }
                    }
                    stage("Checkout") {
                        steps {
                            checkoutScm()
                        }
                    }
                    stage("Environment setup") {
                        steps {
                            sh """
                            rm -rf ${env.ROS_HOME}
                            mkdir ${env.ROS_HOME}
                            """
                        }
                    }
                    stage("Building + packaging") {
                        steps {
                            script {
                                docker.withRegistry("${env.DOCKER_REPO_URL}", "${env.DOCKER_REPO_HOST_CRED}") {
                                    runWithImage(env.BASE_IMAGE, "./package.sh")
                                }
                            }
                        }
                    }
                    stage("Packages capture") {
                        steps {
                            archiveArtifacts(artifacts: 'packages/*.deb', fingerprint: true)
                        }
                    }
                    stage("Uploading packages") {
                        when {
                            environment name: 'IS_RELEASE_BRANCH', value: 'true'
                        }
                        steps {
                            script {
                                def am_sent = uploadPackages("${env.WORKSPACE}/packages", "${env.WORKSPACE}/packages")
                                if (am_sent == 0) {
                                    throw new RuntimeException("No packages were built (or sent)!")
                                }
                            }
                        }
                    }
                }
                post {
                    always {
                        echo "clean up workspace for ${PLATFORM} build in node ${env.NODE_NAME}"
                        deleteDir()
                    }
                    success {
                        echo "${PLATFORM} build succeeeded!"
                    }
                    unstable {
                        echo "${PLATFORM} build unstable :/"
                    }
                    aborted {
                        echo "${PLATFORM} build got aborted :/"
                    }
                    failure {
                        echo "${PLATFORM} build failed :/"
                    }
                }
            }
        }
    }
}

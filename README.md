# motorAcsMotion
EPICS motor drivers for controllers from [ACS Motion Control](https://www.acsmotioncontrol.com/) supporting the ACSPL+ command set.

[![Build Status](https://github.com/epics-motor/motorAcsMotion/actions/workflows/ci-scripts-build.yml/badge.svg)](https://github.com/epics-motor/motorAcsMotion/actions/workflows/ci-scripts-build.yml)
<!--[![Build Status](https://travis-ci.org/epics-motor/motorAcsMotion.png)](https://travis-ci.org/epics-motor/motorAcsMotion)-->

motorAcsMotion is a submodule of [motor](https://github.com/epics-modules/motor).  When motorAcsMotion is built in the ``motor/modules`` directory, no manual configuration is needed.

motorAcsMotion can also be built outside of motor by copying it's ``EXAMPLE_RELEASE.local`` file to ``RELEASE.local`` and defining the paths to ``MOTOR`` and itself.

motorAcsMotion contains an example IOC that is built if ``CONFIG_SITE.local`` sets ``BUILD_IOCS = YES``.  The example IOC can be built outside of driver module.

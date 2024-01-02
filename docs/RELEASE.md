# motorAcsMotion Releases

## __R2-2 (2023-06-06)__
R2-2 is a release based on the master branch.

### Changes since R2-1

#### New features
* None

#### Modifications to existing features
* Pull request [#50](https://github.com/epics-motor/motorAcsMotion/pull/50): Added homing max distance, homing offset positive, and homing offset negative

#### Bug fixes
* Pull request [#48](https://github.com/epics-motor/motorAcsMotion/pull/48) from [Keenan Lang](https://github.com/keenanlang): Don't build for vxWorks; vxWorks version 6.9 doesn't support lround.

## __R2-1 (2023-05-10)__
R2-1 is a release based on the master branch.

### Changes since R2-0

#### New features
* Pull request [#22](https://github.com/epics-motor/motorAcsMotion/pull/22) and [#23](https://github.com/epics-motor/motorAcsMotion/pull/23): Added auxiliary I/O support
* Pull request [#26](https://github.com/epics-motor/motorAcsMotion/pull/26): [Paul Richards](https://github.com/prichards-wmko) added a "homed" record to show when the homing procedure is complete
* Pull request [#28](https://github.com/epics-motor/motorAcsMotion/pull/28): Added support for closed-loop stages with encoders
* Pull request [#29](https://github.com/epics-motor/motorAcsMotion/pull/29): Improved error messages
* Pull request [#40](https://github.com/epics-motor/motorAcsMotion/pull/40) and [#42](https://github.com/epics-motor/motorAcsMotion/pull/42): Made the ENC and ENC2 bits of the FAULT status available as PVs
* Pull request [#39](https://github.com/epics-motor/motorAcsMotion/pull/39): [Tim Speight](https://github.com/tim-speight) implemented jogging and added a jogDirection status record
* Pull request [#43](https://github.com/epics-motor/motorAcsMotion/pull/43): Minor jog improvements
* Pull request [#44](https://github.com/epics-motor/motorAcsMotion/pull/44): Allow the encoder offsets to be zeroed
* Pull request [#46](https://github.com/epics-motor/motorAcsMotion/pull/46): Query the firmware version string and include it in asynReport output

#### Modifications to existing features
* Pull request [#21](https://github.com/epics-motor/motorAcsMotion/pull/21): Moved communication methods to a new SPiiPlusCommDriver class
* Pull request [#45](https://github.com/epics-motor/motorAcsMotion/pull/45): Allow PINI to be specified for global var output records, so they initialize properly in IOCs with autosave.

#### Bug fixes
* Commit [cad837e](https://github.com/epics-motor/motorAcsMotion/commit/cad837ef846f8278f875897ee0514beb41c2fcd0): [Paul Richards](https://github.com/prichards-wmko) found and fixed a significant memory leak
* Pull request [#25](https://github.com/epics-motor/motorAcsMotion/pull/25): Include header files to fix RHEL7 build problems
* Pull request [#36](https://github.com/epics-motor/motorAcsMotion/pull/36): Fixed inconsistencies and typos in max params db and req files
* Commit [2cfc752](https://github.com/epics-motor/motorAcsMotion/commit/2cfc752ccc82cb5143b41f1049076dc4b480cd46): Modified starting locations for MEDM/caQtDM screens so they don't appear outside of FHD displays
* Pull request [#34](https://github.com/epics-motor/motorAcsMotion/pull/34): Applied fixes from [Mark Rivers](https://github.com/MarkRivers) for SPiiPlusAuxDriver build problems on Windows (dynamic)

#### Continuous integration
* Commit [fdfe967](https://github.com/epics-motor/motorAcsMotion/commit/fdfe967817fc871fd4358e33a7a8199d6e8aa1b9): Added ci-scripts (v3.3.0)
* Pull request [#27](https://github.com/epics-motor/motorAcsMotion/pull/27): Configured to build with Github Actions

## __R2-0-1 (2022-06-08)__
R2-0-1 is a bugfix release based on the R2-0-bugfix branch.

### Changes since R2-0

#### Bug fixes

* Commit [8e125cc](https://github.com/epics-motor/motorAcsMotion/commit/8e125cc6e0b4bc868fd837115f5a86fb9f7b6727): Applied fix for memory leak, found and fixed in the master branch by [Paul Richards](https://github.com/prichards-wmko) in [cad837e](https://github.com/epics-motor/motorAcsMotion/commit/cad837ef846f8278f875897ee0514beb41c2fcd0)

## __R2-0 (2022-01-21)__
R2-0 is a release based on the master branch.

### Changes since R1-0

#### New features

* Commit [3b25e12](https://github.com/epics-motor/motorAcsMotion/commit/3b25e1210ef5584d4cab74eee7650e97b9c932b4): Axes can be enabled/disabled from EPICS.
* Pull request [#6](https://github.com/epics-motor/motorAcsMotion/pull/6) Implemented profile moves and data recording.
* Pull request [#7](https://github.com/epics-motor/motorAcsMotion/pull/7) Implemented homing using predefined homing methods.
* Pull request [#8](https://github.com/epics-motor/motorAcsMotion/pull/8) Allow any SPiiPlus user units to be used; the axis resolution (STEPF) is queried during initialization.
* Pull request [#9](https://github.com/epics-motor/motorAcsMotion/pull/9) Allow the max velocity and acceleration to be read and set from EPICS.
* Pull request [#10](https://github.com/epics-motor/motorAcsMotion/pull/10) Allow reading/writing global variables from EPICS.
* Pull request [#11](https://github.com/epics-motor/motorAcsMotion/pull/11) Allow programs to be started and stopped in buffers on the controller.
* Pull request [#12](https://github.com/epics-motor/motorAcsMotion/pull/12) Make Safe Torque Off (STO) status available.
* Pull request [#13](https://github.com/epics-motor/motorAcsMotion/pull/13) Use binary queries for polling.

#### Modifications to existing features

* Commit [42c580b](https://github.com/epics-motor/motorAcsMotion/commit/42c580b9d3d1376e3756e2821e1097c1d394b93e): Pass IOC prefix to dbLoadTemplate in example IOC
* Commit [664e4e4](https://github.com/epics-motor/motorAcsMotion/commit/664e4e43876fe576c4ce1cca0fefdd143ca8ecae): Added a tcp/ip example to acsMotionIOC.
* Commit [7099c0d](https://github.com/epics-motor/motorAcsMotion/commit/7099c0d7264a53d8f8f5abbbbd07ec297e98e9e0): Install iocsh files.

#### Bug fixes

* Commit [e9615c7](https://github.com/epics-motor/motorAcsMotion/commit/e9615c7a59f0683761b57c112780f47c925e7229): Initialized variables to prevent random memory from being freed when SPiiPlusCreate is called.
* Commit [e537b51](https://github.com/epics-motor/motorAcsMotion/commit/e537b51baa182cc7ad3046b897c7e53f888d64f0): Added locks around writeReadController to prevent conficts between the poller thread and the profile move thread.
* Commit [187f609](https://github.com/epics-motor/motorAcsMotion/commit/187f6095f70ed0b459980c690dc70f88ed10d8c7): Initialized status in the axis poll method to avoid errors when toggling the CNEN field or changing a limit.
* Commit [d442ffa](https://github.com/epics-motor/motorAcsMotion/commit/d442ffa3f711377495de9390102b6d8eee6293ba): Initialized status in the getMaxParams.
* Commit [9e48329](https://github.com/epics-motor/motorAcsMotion/commit/9e48329927849ca802c89d17c900ca7985e9a219): Corrected the name of a dbd file in the example IOC.

#### Notes

* Triggering has not been implemented yet.
* Homing to a dedicated home switch requires writing custom code to run on the controller, since there isn't a predefined homing routine for that.

## __R1-0 (2020-08-06)__
Initial release

### Notes

* Only point-to-point moves are supported.

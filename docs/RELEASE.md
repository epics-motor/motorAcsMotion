# motorAcsMotion Releases

## __R2-3-1 (2026-01-14)__
R2-3-1 is a bugfix release based on the master branch.

### Changes since R2-3

#### Bug fixes

* Pull request [#79](https://github.com/epics-motor/motorAcsMotion/pull/79): Fix for binary communication error false positives, first reported by [J. Lewis Muir](https://github.com/jlmuir) in issue [#72](https://github.com/epics-motor/motorAcsMotion/issues/72)

## __R2-3 (2025-12-16)__
R2-3 is a release based on the master branch.

### Important Notes

It is highly recommended to save asynReport output for the ACS port to a file before upgrading IOCs to this version, due to the changes to the resolution detection.  The MRES/ERES fields of the motor record may need to be corrected.

This is the recommended motor record configuration based on the mode:

| Mode                                                         | MRES  | ERES  |
| :----------------------------------------------------------- | :---: | :---: |
| Open loop stepper                                            | STEPF | STEPF |
| Open loop stepper (with enc verification)                    | STEPF | EFAC  |
| Servo Processor Stepper Algo (closed-loop microstep control) | EFAC  | EFAC  |
| Closed-loop brushless motor (or brushless stepper algo)      | EFAC  | EFAC  |

### Changes since R2-2

#### New features
* Pull request [#60](https://github.com/epics-motor/motorAcsMotion/pull/60): Allow disabling setPosition (for axes with absolute encoders)
* Pull request [#73](https://github.com/epics-motor/motorAcsMotion/pull/73): Added pulse generation (PEG) for profile moves
* Pull request [#75](https://github.com/epics-motor/motorAcsMotion/pull/75): Added records for MFLAGS and MFLAGSX

#### Modifications to existing features
* Pull request [#61](https://github.com/epics-motor/motorAcsMotion/pull/61): Improved resolution detection logic based on empirical data collected by [Max Wyman](https://github.com/mdwyman)
* Commit [f87f14d](https://github.com/epics-motor/motorAcsMotion/commit/f87f14d65ec60b9f9a3d1dd518b4ec10ece87e84): Improved asynReport interest level output

#### Bug fixes
* Pull request [#52](https://github.com/epics-motor/motorAcsMotion/pull/52): Fix for homing failure when MaxDistance is zero, a problem reported in issue [#51](https://github.com/epics-motor/motorAcsMotion/issues/51) by [Tim Speight](https://github.com/tim-speight)
* Commit [de7c903](https://github.com/epics-motor/motorAcsMotion/commit/de7c90326da19262f295c5c762f5581295b44389): homingMaxDist, homingOffsetPos, and homingOffsetNeg are now autosaved
* Pull request [#55](https://github.com/epics-motor/motorAcsMotion/pull/55): Allow specifying the homingCurrLimit to avoid problems caused by the default value. See [#54](https://github.com/epics-motor/motorAcsMotion/issues/54) for more info.
* Pull request [#59](https://github.com/epics-motor/motorAcsMotion/pull/59): Use the correct axis index for profileMove calculations
* Commit [3f690cb](https://github.com/epics-motor/motorAcsMotion/commit/3f690cbbddcfd08a6260daf862faef75984b3f5a): [Mark Rivers](https://github.com/MarkRivers) fixed Windows dynamic build problems
* Pull request [#66](https://github.com/epics-motor/motorAcsMotion/pull/66): [J. Lewis Muir](https://github.com/jlmuir) fixed a bug that prevented the use of a non-standard TCP port
* Pull request [#65](https://github.com/epics-motor/motorAcsMotion/pull/65) and [#67](https://github.com/epics-motor/motorAcsMotion/pull/67): [J. Lewis Muir](https://github.com/jlmuir) added support for virtual axes, which are specified by a comma-separated list
* Pull request [#69](https://github.com/epics-motor/motorAcsMotion/pull/69): Improved the default polling period configuration in the example IOC
* Pull request [#74](https://github.com/epics-motor/motorAcsMotion/pull/74): Exit EPICS threads when the IOC is shutting down to prevent error messages
* Pull request [#76](https://github.com/epics-motor/motorAcsMotion/pull/76): Set the MSTA direction bit based on the feedback velocity

#### Continuous integration
* Commit [b6de1b3](https://github.com/epics-motor/motorAcsMotion/commit/b6de1b3f740f1d033860dd1e2acd26cc183b8f41): Updated ci-scripts to v3.4.1
* Commit [c2d67e7](https://github.com/epics-motor/motorAcsMotion/commit/c2d67e708fd8b91154ccba4469362179b45974fc): Switched to v4 of artifact actions

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

# BMCU370
BMCU Xing-C modified version latest (BMCU-C Hall V0.1-0020) source code. Includes some minor personal optimizations.


# Links
- english wiki: https://wiki.yuekai.fr/
- 中文wiki：https://bmcu.wanzii.cn/


# Changelog
Copied from group files

### 25-7月17日-0020；
Fixed lighting logic error, causing some statuses not to light up.
Fixed channel accidental online
Corrected anti-disconnection, previously not effective
Rewrote lighting system, fixed flickering issue, reduced refresh frequency.
When channel error occurs, attempt to update red every 3 seconds to avoid BMCU entering working state after inserting channels that are not lit.


### 25-7月6日-0019 modified version；
Dual microswitch Hall version is also available.

First, the changes from 0019 original to 0013 original;
According to different firmware flashed, P1X1 can support 16 colors now
Fixed the issue where filament information could not be saved after P1X1 printer firmware upgrade (latest 00.01.06.62), or latest slicer software (latest 2.1.1.52).
Modified online logic judgment to prevent erroneous channel online in certain states.
Modified motor control logic to use different calls for high and low voltage positions.

Then, changes from the previous 0013 version with lamp anti-overheat;
Mainboard lights, red breathing when not connected to printer, white breathing during normal work.
Further reduced the brightness of buffer lights and mainboard lights.
Retraction part, abandoned control of A1.
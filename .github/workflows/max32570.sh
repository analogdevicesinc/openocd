# SPDX-License-Identifier: GPL-2.0-or-later

TARGET=max32570
TARGET_CFG=${TARGET}.cfg

if [[ $# -ne 1 ]]
then
    echo "Need to pass in CMSIS_DAP_ID as argument"
    exit 1
fi

CMSIS_DAP_ID=$1 # Get from arguments
HEXGEN=~/Tools/hexgen.pl

# Reset any tcl file modifications
git checkout tcl/target/${TARGET_CFG}

# Enable the QSPI driver
sed -i "s/set QSPI_ENABLE 0/set QSPI_ENABLE 1/g" tcl/target/${TARGET_CFG}

# Test 1: Start a simple OpenOCD session
./src/openocd -f tcl/interface/cmsis-dap.cfg -f tcl/target/${TARGET_CFG} -s tcl  \
    -c "gdb_port 3334" -c "telnet_port 4445" -c "tcl_port 6665" \
    -c "adapter serial ${CMSIS_DAP_ID}" -c "init; reset halt; exit"

# Test 2: Mass erase
./src/openocd -f tcl/interface/cmsis-dap.cfg -f tcl/target/${TARGET_CFG} -s tcl  \
-c "gdb_port 3334" -c "telnet_port 4445" -c "tcl_port 6665" \
-c "adapter serial ${CMSIS_DAP_ID}" -c "init; reset halt; max32xxx mass_erase 0; flash erase_check 0; exit"
./src/openocd -f tcl/interface/cmsis-dap.cfg -f tcl/target/${TARGET_CFG} -s tcl  \
-c "gdb_port 3334" -c "telnet_port 4445" -c "tcl_port 6665" \
-c "adapter serial ${CMSIS_DAP_ID}" -c "init; reset halt; max32xxx mass_erase 1; flash erase_check 1; exit"
./src/openocd -f tcl/interface/cmsis-dap.cfg -f tcl/target/${TARGET_CFG} -s tcl  \
-c "gdb_port 3334" -c "telnet_port 4445" -c "tcl_port 6665" \
-c "adapter serial ${CMSIS_DAP_ID}" -c "init; reset halt; max32xxx_qspi mass_erase 2; flash erase_check 2; exit"

# Test 3: Program and verify a file to the main flash bank
export ADDR_BASE=0x10000000
export ADDR_MOD=0x30000
export LEN=0x10000
export LEN_MOD=0x10000
$HEXGEN $ADDR_BASE $ADDR_MOD $LEN $LEN_MOD ${TARGET}_rand.hex
./src/openocd -f tcl/interface/cmsis-dap.cfg -f tcl/target/${TARGET_CFG} -s tcl  \
    -c "gdb_port 3334" -c "telnet_port 4445" -c "tcl_port 6665" \
    -c "adapter serial ${CMSIS_DAP_ID}" -c "init; program ${TARGET}_rand.hex verify exit "

# Test 4: Program and verify a file to the second flash bank
export ADDR_BASE=0x10080000
export ADDR_MOD=0x30000
export LEN=0x10000
export LEN_MOD=0x10000
$HEXGEN $ADDR_BASE $ADDR_MOD $LEN $LEN_MOD ${TARGET}_rand.hex
./src/openocd -f tcl/interface/cmsis-dap.cfg -f tcl/target/${TARGET_CFG} -s tcl  \
    -c "gdb_port 3334" -c "telnet_port 4445" -c "tcl_port 6665" \
    -c "adapter serial ${CMSIS_DAP_ID}" -c "init; program ${TARGET}_rand.hex verify exit "

# Test 5: Program and verify a file to the QSPI bank
export ADDR_BASE=0x08000000
export ADDR_MOD=0x30000
export LEN=0x1000
export LEN_MOD=0x1000
$HEXGEN $ADDR_BASE $ADDR_MOD $LEN $LEN_MOD ${TARGET}_rand.hex
./src/openocd -f tcl/interface/cmsis-dap.cfg -f tcl/target/${TARGET_CFG} -s tcl  \
    -c "gdb_port 3334" -c "telnet_port 4445" -c "tcl_port 6665" \
    -c "adapter serial ${CMSIS_DAP_ID}" -c "init; program ${TARGET}_rand.hex verify exit "

# Test 6: Program across multiple internal flash banks
export ADDR_BASE=0x10000000
export ADDR_MOD=0x0
export LEN=0x100000
export LEN_MOD=0x0
$HEXGEN $ADDR_BASE $ADDR_MOD $LEN $LEN_MOD ${TARGET}_rand.hex
./src/openocd -f tcl/interface/cmsis-dap.cfg -f tcl/target/${TARGET_CFG} -s tcl  \
    -c "gdb_port 3334" -c "telnet_port 4445" -c "tcl_port 6665" \
    -c "adapter serial ${CMSIS_DAP_ID}" -c "init; program ${TARGET}_rand.hex verify exit "

# Test 7: Program and verify a file to the QSPI bank with encryption
export ADDR_BASE=0x08000000
export ADDR_MOD=0x30000
export LEN=0x1000
export LEN_MOD=0x1000
$HEXGEN $ADDR_BASE $ADDR_MOD $LEN $LEN_MOD ${TARGET}_rand.hex

# Program the flash encrypted
sed -i "s/set QSPI_OPTIONS 0x0/set QSPI_OPTIONS 0x22/g" tcl/target/${TARGET_CFG}
./src/openocd -f tcl/interface/cmsis-dap.cfg -f tcl/target/${TARGET_CFG} -s tcl  \
    -c "gdb_port 3334" -c "telnet_port 4445" -c "tcl_port 6665" \
    -c "adapter serial ${CMSIS_DAP_ID}" -c "init; reset halt; program ${TARGET}_rand.hex verify exit "

# Revert the options to disable encryption
sed -i "s/set QSPI_OPTIONS 0x22/set QSPI_OPTIONS 0x0/g" tcl/target/${TARGET_CFG}

# Verify unencrypted, expect this to fail
set +e
./src/openocd -f tcl/interface/cmsis-dap.cfg -f tcl/target/${TARGET_CFG} -s tcl  \
    -c "gdb_port 3334" -c "telnet_port 4445" -c "tcl_port 6665" \
    -c "adapter serial ${CMSIS_DAP_ID}" -c "init; reset halt; flash verify_image ${TARGET}_rand.hex; exit "

# Check the return value to make sure we received an error
if [ "$?" -ne "1" ]
then
    echo "Error: Verify should have failed"
    exit 1
fi
set -e

# Test 8: Program and verify a file to the QSPI bank with encryption and authentication
export ADDR_BASE=0x08000000
export ADDR_MOD=0x100
export LEN=0x10000
export LEN_MOD=0x1000
$HEXGEN $ADDR_BASE $ADDR_MOD $LEN $LEN_MOD ${TARGET}_rand.hex

# Program the flash encrypted and authenticated
sed -i "s/set QSPI_OPTIONS 0x0/set QSPI_OPTIONS 0x26/g" tcl/target/${TARGET_CFG}
./src/openocd -f tcl/interface/cmsis-dap.cfg -f tcl/target/${TARGET_CFG} -s tcl  \
  -c "gdb_port 3334" -c "telnet_port 4445" -c "tcl_port 6665" \
  -c "adapter serial ${CMSIS_DAP_ID}" -c "init; reset halt; program ${TARGET}_rand.hex verify exit "

# Revert the options to disable authentication
sed -i "s/set QSPI_OPTIONS 0x26/set QSPI_OPTIONS 0x22/g" tcl/target/${TARGET_CFG}

# Verify unencrypted, expect this to fail
set +e
./src/openocd -f tcl/interface/cmsis-dap.cfg -f tcl/target/${TARGET_CFG} -s tcl  \
  -c "gdb_port 3334" -c "telnet_port 4445" -c "tcl_port 6665" \
  -c "adapter serial ${CMSIS_DAP_ID}" -c "init; reset halt; flash verify_image ${TARGET}_rand.hex; exit "

# Check the return value to make sure we received an error
if [ "$?" -ne "1" ]
then
    exit 1
fi
set -e

# Revert the options to disable encryption
sed -i "s/set QSPI_OPTIONS 0x22/set QSPI_OPTIONS 0x0/g" tcl/target/${TARGET_CFG}

# Verify unencrypted, expect this to fail
set +e
./src/openocd -f tcl/interface/cmsis-dap.cfg -f tcl/target/${TARGET_CFG} -s tcl  \
  -c "gdb_port 3334" -c "telnet_port 4445" -c "tcl_port 6665" \
  -c "adapter serial ${CMSIS_DAP_ID}" -c "init; reset halt; flash verify_image ${TARGET}_rand.hex; exit "

# Check the return value to make sure we received an error
if [ "$?" -ne "1" ]
then
    echo "Error: Verify should have failed"
    exit 1
fi
set -e


# Test 9: Program small flash sections
export ADDR_BASE=0x10000000
export ADDR_MOD=0x100
export LEN=0x1
export LEN_MOD=0x0
$HEXGEN $ADDR_BASE $ADDR_MOD $LEN $LEN_MOD ${TARGET}_rand.hex
./src/openocd -f tcl/interface/cmsis-dap.cfg -f tcl/target/${TARGET_CFG} -s tcl  \
    -c "gdb_port 3334" -c "telnet_port 4445" -c "tcl_port 6665" \
    -c "adapter serial ${CMSIS_DAP_ID}" -c "init; program ${TARGET}_rand.hex verify exit "

export ADDR_BASE=0x10000000
export ADDR_MOD=0x1000
export LEN=0x3
export LEN_MOD=0x5
$HEXGEN $ADDR_BASE $ADDR_MOD $LEN $LEN_MOD ${TARGET}_rand.hex
./src/openocd -f tcl/interface/cmsis-dap.cfg -f tcl/target/${TARGET_CFG} -s tcl  \
    -c "gdb_port 3334" -c "telnet_port 4445" -c "tcl_port 6665" \
    -c "adapter serial ${CMSIS_DAP_ID}" -c "init; program ${TARGET}_rand.hex verify exit "

# Test 10: Program small flash sections in QSPI
sed -i "s/set QSPI_OPTIONS 0x0/set QSPI_OPTIONS 0x26/g" tcl/target/${TARGET_CFG}
export ADDR_BASE=0x08000000
export ADDR_MOD=0x100
export LEN=0x1
export LEN_MOD=0x0
$HEXGEN $ADDR_BASE $ADDR_MOD $LEN $LEN_MOD ${TARGET}_rand.hex
./src/openocd -f tcl/interface/cmsis-dap.cfg -f tcl/target/${TARGET_CFG} -s tcl  \
    -c "gdb_port 3334" -c "telnet_port 4445" -c "tcl_port 6665" \
    -c "adapter serial ${CMSIS_DAP_ID}" -c "init; program ${TARGET}_rand.hex verify exit "

export ADDR_BASE=0x08000000
export ADDR_MOD=0x1000
export LEN=0x3
export LEN_MOD=0x5
$HEXGEN $ADDR_BASE $ADDR_MOD $LEN $LEN_MOD ${TARGET}_rand.hex
./src/openocd -f tcl/interface/cmsis-dap.cfg -f tcl/target/${TARGET_CFG} -s tcl  \
    -c "gdb_port 3334" -c "telnet_port 4445" -c "tcl_port 6665" \
    -c "adapter serial ${CMSIS_DAP_ID}" -c "init; program ${TARGET}_rand.hex verify exit "

# Reset any tcl file modifications
git checkout tcl/target/${TARGET_CFG}

echo "Success!"

#!/bin/bash

OW_TYPE=0xA0

function UpdateCRC() {
	local i
	CRC=$(($CRC ^ $1))
	for ((i=0;i<8;i++))
	do
		if (($CRC & 0x01))
		then
			CRC=$(( ($CRC>>1) ^ 0x8c))
		else
			CRC=$(($CRC>>1))
		fi
	done
}

function MakeSerial() {
	CRC=0x00
	SUM=0x08

	SUM=$((SUM+$OW_TYPE))
	UpdateCRC $OW_TYPE
	printf ":08000000"
	printf "%02X" $OW_TYPE

	# Generate a highly-random serial number.
	for ((i=0;i<5;i++))
	do
		byte=$(openssl rand -hex 1)
		UpdateCRC 0x$byte
		SUM=$((SUM+0x$byte))
		printf "%02X" 0x"$byte"
	done

	# Make randomly-generated serial numbers
	# have 0xFF for the most significant byte.
	UpdateCRC 0xFF
	printf "%02X" 0xFF
	SUM=$((SUM+0xFF))

	printf "%02X" $CRC
	SUM=$((SUM+CRC))

	printf "%02X\n" $((((~SUM+1) & 0xFF)))
}

# Record type 4, size 2 bytes... 0x00 0x08
echo ":02000004008179"

# Record type 0, data
MakeSerial

# Record type 1, EOF
echo ":00000001FF"


/*---------------------------------------------------------*\
| ENESMBusInterface_i2c_smbus.cpp                           |
|                                                           |
|   ENE SMBus interface for I2C/SMBus                       |
|                                                           |
|   Adam Honse (CalcProgrammer1)                21 Nov 2021 |
|                                                           |
|   This file is part of the OpenRGB project                |
|   SPDX-License-Identifier: GPL-2.0-or-later               |
\*---------------------------------------------------------*/

#include <chrono>
#include <thread>
#include "ENESMBusInterface_i2c_smbus.h"

using namespace std::chrono_literals;

/*---------------------------------------------------------*\
| Number of attempts made for each transaction when checked |
| I/O is enabled.  DDR5 platforms share the SMBus with the  |
| SPD hubs and PMICs, which firmware polls in the           |
| background, so ENE transactions are occasionally NAKed.   |
\*---------------------------------------------------------*/
#define ENE_CHECKED_IO_ATTEMPTS         5
#define ENE_CHECKED_IO_RETRY_DELAY      1ms

ENESMBusInterface_i2c_smbus::ENESMBusInterface_i2c_smbus(i2c_smbus_interface* bus, bool checked_io)
{
    this->bus           = bus;
    this->checked_io    = checked_io;
    this->read_error    = false;
}

ENESMBusInterface_i2c_smbus::~ENESMBusInterface_i2c_smbus()
{

}

ene_interface_type ENESMBusInterface_i2c_smbus::GetInterfaceType()
{
    return(ENE_INTERFACE_TYPE_I2C_SMBUS);
}

std::string ENESMBusInterface_i2c_smbus::GetLocation()
{
    std::string return_string(bus->info.device_name);
    return("I2C: " + return_string);
}

int ENESMBusInterface_i2c_smbus::GetMaxBlock()
{
    return(3);
}

int ENESMBusInterface_i2c_smbus::SetRegisterPointer(ene_dev_id dev, ene_register reg)
{
    return(bus->i2c_smbus_write_word_data(dev, 0x00, ((reg << 8) & 0xFF00) | ((reg >> 8) & 0x00FF)));
}

unsigned char ENESMBusInterface_i2c_smbus::ENERegisterRead(ene_dev_id dev, ene_register reg)
{
    if(!checked_io)
    {
        //Write ENE register
        SetRegisterPointer(dev, reg);

        //Read ENE value
        return(bus->i2c_smbus_read_byte_data(dev, 0x81));
    }

    /*-----------------------------------------------------*\
    | Checked read: retry the pointer write and the read as |
    | a pair.  If the read never succeeds, flag the error   |
    | and return 0x00 rather than 0xFF, which would         |
    | otherwise show up as 255 LEDs in the config table.    |
    \*-----------------------------------------------------*/
    for(int attempt = 0; attempt < ENE_CHECKED_IO_ATTEMPTS; attempt++)
    {
        if(SetRegisterPointer(dev, reg) >= 0)
        {
            int res = bus->i2c_smbus_read_byte_data(dev, 0x81);

            if(res >= 0)
            {
                return((unsigned char)res);
            }
        }

        std::this_thread::sleep_for(ENE_CHECKED_IO_RETRY_DELAY);
    }

    read_error = true;

    return(0x00);
}

bool ENESMBusInterface_i2c_smbus::TakeReadError()
{
    bool error = read_error;

    read_error = false;

    return(error);
}

void ENESMBusInterface_i2c_smbus::ENERegisterWrite(ene_dev_id dev, ene_register reg, unsigned char val)
{
    if(!checked_io)
    {
        //Write ENE register
        SetRegisterPointer(dev, reg);

        //Write ENE value
        bus->i2c_smbus_write_byte_data(dev, 0x01, val);
        return;
    }

    for(int attempt = 0; attempt < ENE_CHECKED_IO_ATTEMPTS; attempt++)
    {
        if((SetRegisterPointer(dev, reg) >= 0) && (bus->i2c_smbus_write_byte_data(dev, 0x01, val) >= 0))
        {
            return;
        }

        std::this_thread::sleep_for(ENE_CHECKED_IO_RETRY_DELAY);
    }
}

void ENESMBusInterface_i2c_smbus::ENERegisterWriteBlock(ene_dev_id dev, ene_register reg, unsigned char * data, unsigned char sz)
{
    //Write ENE register
    SetRegisterPointer(dev, reg);

    //Write ENE block data
    if(bus->i2c_smbus_write_block_data(dev, 0x03, sz, data) == -1)
    {
        //Fall back to individual byte operations if the block operation fails
        for(unsigned int block_byte = 0; block_byte < sz; block_byte++)
        {
            bus->i2c_smbus_write_byte_data(dev, 0x01, data[block_byte]);
        }
    }
}

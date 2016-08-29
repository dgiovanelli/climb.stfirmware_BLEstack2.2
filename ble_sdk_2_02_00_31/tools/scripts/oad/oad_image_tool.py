'''
/*
 * Filename:    oad_image_tool.py
 *
 * Description: This tool is used to generate OAD/production images for OAD
 * Enabled projects using the TI-BLE SDK.
 *
 *
 * Copyright (C) 2016 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/
'''

# Needs python 2.7.10
from __future__ import print_function
import __builtin__
import argparse
import crcmod # pip -[--proxy <addr>] install crcmod
from intelhex import IntelHex # pip [--proxy <addr>] install intelhex, needs latest version
import struct
import textwrap
import sys
import math
import ntpath
from collections import namedtuple

#tool version number
tool_version = "1.0"
#CRC related data
# CRC Polynomial used by OAD for CC254x
#crc16 = crcmod.mkCrcFun(0x18005, rev=False, initCrc=0x0000, xorOut=0x0000)
# CRC Poly used by OAD for CC26xx
crc16 = crcmod.mkCrcFun(0x11021, rev=False, initCrc=0x0000, xorOut=0x0000)


OAD_HDR_FMT = 'HHHH4sHBB'
OadHdr = namedtuple('OadImgHdr', 'crc crcShdw imgVer imgLen usrId imgAddr imgType status')
#the below dictionary contains the mapping from imgType string to an integeer
imgTypes = {'app': 1, 'stack': 2, 'np': 3, 'production': 4 }

#Meta data status field measures success as 0xFF
META_STATUS_SUCCESS = 0xFF
META_STATUS_PREPENDED = 0XFE
#External flash layout (this info is in ext_flash_layout.h)
EXT_FL_SECTOR_1_SECTOR = "APP: 0x00000 - 0x1FFFF"
EXT_FL_SECTOR_2_SECTOR = "STACK/NP: 0x20000 - 0x3FFFF"
EXT_FL_SECTOR_3_SECTOR = "Factory: 0x40000 - 0x5FFFF"
EXT_FL_PG_SIZE = 4096

#OAD defines (this info is in oad_target.h)
OAD_BLOCK_SIZE        =   16        #this value is in bytes
OAD_METADATA_SIZE     =   16        #this value is in bytes
INT_FL_PG_SIZE        =   4096      #this value is in bytes
INT_FL_RSVD_PG1       =   0x1000
INT_FL_RSVD_PG7       =   0x7000
INT_FL_RSVD_PG31      =   0x1F000
#note that only internal flash pages between 6 and 30 (inclusive) can be updated OTA for int flash OAD
INT_FL_OAD_IMG_B_META_BEGIN = 0x9000      #First addr of on chip OAD img B
INT_FL_OAD_IMG_B_END        = 0x12FFF
INT_FL_OAD_IMG_A_META_BEGIN = 0x600
INT_FL_OAD_IMG_A_BEGIN      = 0x100
INT_FL_OAD_IMG_A_END        = 0x8FFF
#note that only internal flash pages between 1-30 (inclusive) can be updated OTA for ext flash OAD
EXT_FL_OAD_META_BEGIN =   0x1000      #First addr of app space for ext flash OAD
EXT_FL_OAD_META_END   =   EXT_FL_OAD_META_BEGIN + OAD_METADATA_SIZE




#Argparse is a class that helps to make friendlier command line interfaces.
#the text you see below is what is printed as a form of documentation on how
#to use the tool, similar to a man command
if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        prog="Texas Instruments OAD Image Tool",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description=textwrap.dedent('''\
           Merges Intel Hex format files provided as command line arguments or via stdin.
           Default output is Intel-Hex to stdout. Input via stdin and/or missing output
           file specifiers implies --quiet.

           Generates and inserts or appends metadata needed for Over the Air Download.

           Unless the location for the metadata structure is explicitly given as an
           argument with -m <addr>, one of the following will happen:
             1. The lowest 16 bytes of the merged image is all 0xFF. In this case the meta-
                data is inserted here, and status is set to 0xFF.
             2. The metadata is pre-pended to the hex and binary images, and status is set
                to 0xFE to indicate an OAD Host should not transmit this as part of the
                image data.

           The generated meta-data structure looks like this:

           [ 0 - 2 ][  2 - 4   ][ 4 - 6  ][ 6 - 8  ][  8-12    ][ 12 - 14 ][   15    ][  16  ]
           [  CRC  ][ CRC-SHDW ][ imgVer ][ imgLen ][  usrId   ][ imgAddr ][ imgType ][ stat ]
           [ calc' ][  0xFFFF  ][   (0)  ][ calc'd ][ ("EEEE") ][ calc'd  ][  (APP)  ][ 0xFF ]

        '''),
        epilog=textwrap.dedent('''\

            Usage examples:
              %(prog)s app.hex stack.hex > merged_oad.hex
                  Merges app.hex and stack.hex, filling in metadata from defaults.
              %(prog)s app.hex -ob app_oad.bin -m 0x1000 -r :0xE000
                  Place metadata at 0x1000, and fill or cut off image at 0xE000, starting from
                  first detected data location, output binary app.bin.
        '''))

    def auto_int(x):
        return int(x, 0)

    def auto_range(r):
        r = r.split(':')
        if len(r) == 1: r.append('')
        return [int(x, 0) if x != '' else None for x in r]

    def strip_path_filetype(*args, **kwargs):
        ft = argparse.FileType(*args, **kwargs)
        def inner(filename):
            if type(filename) is type(''):
                filename = filename.strip()
                if filename == '': filename = '-' # Blank means stdin/out
            return ft(filename)
        return inner

    #helper functions to aide in printing to the console
    def print_metadata(metaVector):
        metaDataDispStr = "Data : | 0x%04X |  0x%04X  | 0x%04X | 0x%04X |  " %  (metaVector[0], metaVector[1],metaVector[2], metaVector[3]) + metaVector[4] + \
                        "    | 0x%04X  |  0x%02X   | 0x%02X |\n"  % (metaVector[5], metaVector[6], metaVector[7])
        print("\n")
        print("The script has calculated the 16 Byte OAD Metadata vector below\n")
        print("Bytes: | 0 - 2  |  2 - 4   | 4 - 6  | 6 - 8  |  8-12    | 12 - 14 |   15    |  16  |")
        print("Desc : |  CRC   | CRC-SHDW | imgVer | imgLen |  usrId   | imgAddr | imgType | stat |")
        print("------------------------------------------------------------------------------------")
        print(metaDataDispStr)
        print("******************************************************************************************")
        return

    def print_console_header():
        print("******************************************************************************************")
        print(parser.prog)
        print("Version: " + tool_version)
        print("******************************************************************************************")
        return

    def print_args_info(inputFileList, outHex, outBin, oadType, imgType):
        inputFileStr = ""
        for f in inputFileList:
            inputFileStr += (", " + ntpath.basename(f))

        if outHex is not None:
            outHexStr = ntpath.basename(outHex.name)
        else:
            outHexStr = "none"

        if outBin is not None:
            outBinStr = ntpath.basename(outBin.name)
        else:
            outBinStr = "none"

        print("OAD Type: " + oadType)
        print("Img Type: " + imgType.upper())
        print("Input file(s): " + inputFileStr)
        print("Output Hex file: " + outHexStr)
        print("Output Bin file: " + outBinStr)
        print("******************************************************************************************")
        print("Runtime Output:\n")

        return

    def addr_is_in_oad_imgspace(addr):
        #check if the provided address lies within the OAD image space
        #for on or offchip
        if vargs.oadtype == 'offchip':
            return addr >= INT_FL_RSVD_PG1 and addr < INT_FL_RSVD_PG31
        elif vargs.imgtype == 'production':
            #oad imgA on chip app space is between end of pg0-9
            return addr >= INT_FL_OAD_IMG_A_BEGIN and addr <= INT_FL_OAD_IMG_A_END
        else:
            #right now on chip OAD considers App space to be between pg9-30, which includes the stack.
            #assume for now that the user doesn't try to udpate the stack
            return addr >= INT_FL_OAD_IMG_B_META_BEGIN and addr <= INT_FL_OAD_IMG_B_END

    def argument_sanity_check(vargs, mergedHex):
        #onchip OAD only supports app or production images
        if vargs.oadtype == 'onchip' and (vargs.imgtype == 'stack' or vargs.imgtype == 'np'):
            print("Fatal Error: -- Cannot perform " + vargs.oadtype +" OAD of " + vargs.imgtype + ". Exiting.")
            sys.exit(1)

        #off chip production images are a different beast, they need don't metadata
        #they simply need to merge (optionally convert to bin) and output
        #We need to make sure there is a BIM image included  (a quick check of this is to see if any of the images start at 0)
        #alternatively, non-production images cannot contain data in page zero or page 31

        #Network processor images are not OAD aware, but are expected to start at pg0 (include int vects)
        #this is because the serial bootloader will start loading data at addr0
        if vargs.imgtype == 'production' or vargs.imgtype == 'np':
            if mergedHex.minaddr() != 0x00000:
                print("Fatal Error: -- "+ vargs.imgtype.upper() + " image must include must have data(intVects) at addr 0. Exiting.")
                sys.exit(1)
            else:
                print("Data at Addr0, assume BIM/OAD Target App is present")
        elif vargs.imgtype == 'app' or vargs.imgtype == 'stack':
            #else we are inspecting an OAD ready that is targeted for internal flash of SoC/AP
            #check to ensure that no data is placed invalid sectors pg 0-6,31 for onchip, pg0,31 for offchip
            addrSegmentsList = mergedHex.segments()
            for seg in  addrSegmentsList:
                #note that the end addr seems to not be inclusive so we need to do a minus 1
                if not addr_is_in_oad_imgspace(int(seg[0])) or not addr_is_in_oad_imgspace(int(seg[1] - 1)):
                    print("Fatal Error: -- Non Production/NP image cannot have data in reserved pages. Exiting.")
                    sys.exit(1)
                #otherwise input hex ranges look valid
        #do checking on meta parameter
        if vargs.meta is not None:
            if vargs.oadtype == 'onchip' and vargs.imgtype == 'production':
                #on chip OAD target still has metadata
                #by default it resides at 0x0600, however, just be sure it doesn't reside in BIM
                if not (vargs.meta >= INT_FL_RSVD_PG1 and vargs.meta < INT_FL_RSVD_PG31):
                    print("Fatal Error: -- OAD target image must reside in pg1. Exiting.")
                    sys.exit(1)
            elif vargs.oadtype == 'offchip' and vargs.imgtype == 'production':
                print("Info: -- Metadata isn't needed for production images for off chip OAD, skipping meta gen")
            elif vargs.imgtype == 'app' or vargs.imgtype == 'stack':
                if not addr_is_in_oad_imgspace(vargs.meta):
                    print("Warning: -- metadata specified within resident image space, attempting to place at app start.")
            elif vargs.imgType == 'np':
                print("Warning: -- meta data cannot be embedded in NP image, ignoring meta generation")
        #do range checking
        if vargs.range is not None:
            if vargs.imgtype == 'production' or vargs.imgtype == 'np':
                #production/np images should start at 0 include intvecs
                if vargs.range[0] != 0x0:
                    print("Warning: -- production/np images should contain int vects...attempting to override")
            else:
                #other images should have range within their app space
                #might have a potential off by one here
                if not addr_is_in_oad_imgspace(vargs.range[0]) or not addr_is_in_oad_imgspace(vargs.range[1]):
                    print("Warning: --  range should be within oad app space...attempting to override")








    #setup the command line argument options
    parser.add_argument('hexfile', help="Path(s) to input image(s)", nargs='*', type=strip_path_filetype('r'), default=[sys.stdin])
    parser.add_argument('-t', '--oadtype', help="Whether to generate hex files for on or off chip OAD", choices=['onchip','offchip'], default='offchip')
    parser.add_argument('-i', '--imgtype', help="Defines the img type. For Onchip: only app and production are valid. For Offchip: app, stack, prodcution, np are valid",
                        choices=['app', 'stack', 'np', 'production'], default='app')
    parser.add_argument('-v', '--imgVer', help="Defines the version of the application to be downloaded", type=auto_int, default=0)
    parser.add_argument('-o', '--out', help='Path to output hex file. Missing -o and -ob implies -q', type=strip_path_filetype('w'), default=sys.stdout)
    parser.add_argument('-ob', '--outbin', help='Path to output bin file. Missing -o and -ob implies -q', type=strip_path_filetype('wb'), nargs='?', const=sys.stdout)
    parser.add_argument('-f', '--fill', help='Filler data in output [0xff]', type=auto_int, default=0xff)
    parser.add_argument('-m', '--meta', help='Override calculated location of metadata', type=auto_int)
    #parser.add_argument('-c', '--crc', help='Override generated CRC value', type=auto_int)
    parser.add_argument('-r', '--range', help='Range of addresses included in output', type=auto_range)
    #parser.add_argument('-l', '--len', help='Override calculated image length [bytes]', type=auto_int)
    parser.add_argument('-n', '--dry-run', action='store_true', help='Do not produce output, only show info', default=False)
    parser.add_argument('-q', '--quiet', action='store_true', help='Do not produce diagnostic and informational output', default=False)
    parser.add_argument('--round', help='Round up end-address to fill nearest X, for example 4096 if sectors are 4kB. Ignored if end-range given', type=auto_int)
    parser.add_argument('--version', action='version', version=(parser.prog + ' ' + tool_version))

    #parse the user's command line arguments
    vargs = parser.parse_args()

    # Determine if normal output should be presented
    if vargs.quiet or (vargs.hexfile[0] is sys.stdin) or (vargs.out is sys.stdout) or (vargs.outbin is sys.stdout):
        oldPrint = __builtin__.print
        def myprint(*args, **kwargs):
            pass
        __builtin__.print = myprint

    # first, print a neat header
    print_console_header()

    # Parse and merge hex files
    #iterate over input hexfiles, intelhexify them
    #if we cannot merge or open the hex files, the script is hosed, print and abort
    ihs = []
    inputFileNames = []
    mergedHex = IntelHex()

    for f in vargs.hexfile:
        try:
            ih = IntelHex(f)
            inputFileNames.append(f.name)
            ihs.append(ih)
            try:
                mergedHex.merge(ih, overlap='replace')
            except:
                print("Fatal Error: -- FAILED merge due to overlap when merging " + f.name)
                sys.exit(1)
        except:
            print("Fatal Error: -- FAILED parsing input hex file(s)")
            sys.exit(1)

    #print information about the input hex files
    print_args_info(inputFileNames, vargs.out, vargs.outbin, vargs.oadtype, vargs.imgtype)

    #Now that we have a merged hex image, lets do a bunch of arg checking
    #since mergedHex is an merge of all input hexes, it can be treated as an argument to the script
    argument_sanity_check(vargs, mergedHex)


    # Cut off / fill with --fill.
    startAddr = mergedHex.minaddr()
    endAddr = mergedHex.addresses()[-1] + 1  # Inclusive address

    if startAddr % OAD_BLOCK_SIZE:
        print("Fatal Error: -- Start address 0x%X is not divisible by 16. Exiting")
        sys.exit(1)

    # DevMon rounds up to nearest sector. Why not, if they want to waste time and space.
    if vargs.round is not None:
        endAddr = ((endAddr + INT_FL_PG_SIZE) & ~(INT_FL_PG_SIZE-1))
        print ('endAddr round', hex(endAddr))

    if vargs.range is not None:
        if vargs.range[0] is not None: startAddr = vargs.range[0]
        if vargs.range[1] is not None: endAddr = vargs.range[1]

    # Make sure the last address is divisible by 16
    remainder = endAddr % OAD_BLOCK_SIZE
    if remainder:
        print("Last address was 0x%0X. Expanded to 0x%0X to be divisible by OAD block size" % (endAddr, endAddr+(OAD_BLOCK_SIZE-remainder)) )
        endAddr += OAD_BLOCK_SIZE - remainder

    #if specified, the script will pad the image with the pad value
    fillHex = IntelHex()
    fillHex.puts(startAddr, struct.pack('B', vargs.fill) * (endAddr))
    mergedHex.merge(fillHex, 'ignore')
    mergedHex = mergedHex[startAddr:endAddr] # +1 since the last address is not inclusive
    mergedHex.padding = vargs.fill


    #if we are calculating metadata
    #Offchip OAD production images and NP images don't have embedded metadata headers, skip meta data calc for these
    #All onchip OAD images need metdata placement
    if not(vargs.oadtype == 'offchip' and (vargs.imgtype == 'production' or vargs.imgtype == 'np')):
        # Place metadata, onchip production image expects header at 0x600
        if vargs.oadtype == 'onchip' and vargs.imgtype == 'production':
            residentHdr = OadHdr._make(struct.unpack(OAD_HDR_FMT, mergedHex.tobinstr(INT_FL_OAD_IMG_A_META_BEGIN, INT_FL_OAD_IMG_A_META_BEGIN+15)))
        else:
            residentHdr = OadHdr._make(struct.unpack(OAD_HDR_FMT, mergedHex.tobinstr(startAddr, startAddr+15)))
        metaAddr = vargs.meta
        if metaAddr is None:
            # Try first address, ideally there should be free space at the beginning of the image
            #in the reserved image metadata sector
            #if onchip production image, we must start at imgA start with is 0x600
            if vargs.oadtype == 'onchip' and vargs.imgtype == 'production':
                attemptedMetaStart = INT_FL_OAD_IMG_A_META_BEGIN
            else:
                attemptedMetaStart = mergedHex.minaddr()

            beginning = mergedHex.gets(attemptedMetaStart, OAD_METADATA_SIZE)


            blank16 = struct.pack('B', 0xff)*OAD_METADATA_SIZE
            if beginning == blank16:
                metaAddr = attemptedMetaStart
                print ("Found free area at start of address range, metalocation: 0x%08X" % metaAddr)

            else:
                # there are two cases to be checked by this else:
                # 1.Is there already metadata embedded in the image
                # - we verify this by checking if imgLen <= actual size and if crc == crc
                # 2.There is no metadata in the image, and no metadata addr is specified
                # - We will look to see if there room for metadata based on start addr
                # - remember that for current on OAD solution metadata must reside
                # - in one of two places defined by EXT_FL_OAD_META_BEGIN, INT_FL_OAD_META_BEGIN

                #first lets assume the first 16B are meta data and see that proves correct
                if vargs.oadtype == 'onchip' and vargs.imgtype == 'production':
                    hexHdr = OadHdr._make(struct.unpack(OAD_HDR_FMT, mergedHex.tobinstr(INT_FL_OAD_IMG_A_META_BEGIN, INT_FL_OAD_IMG_A_META_BEGIN+15)))
                else:
                    hexHdr = OadHdr._make(struct.unpack(OAD_HDR_FMT, mergedHex.tobinstr(startAddr, startAddr+15)))

                if vargs.oadtype == 'onchip' and vargs.imgtype == 'production':
                    crcBin = mergedHex.tobinstr(INT_FL_OAD_IMG_A_META_BEGIN+4, INT_FL_OAD_IMG_A_END-1)
                    localLen = int(math.ceil((INT_FL_OAD_IMG_A_END - INT_FL_OAD_IMG_A_META_BEGIN) / 4.0))
                else:
                    crcBin = mergedHex.tobinstr(startAddr+4, endAddr-1)
                    localLen = int(math.ceil((mergedHex.maxaddr() - mergedHex.minaddr()) / 4.0))


                localCrc = crc16(crcBin)
                #if the resident header checks out, then we will assume metaAddr==startAddr
                #note that the script will still over-write what is currently there
                if localCrc == hexHdr.crc and hexHdr.crcShdw == 0xFFFF \
                    and hexHdr.imgLen <= localLen:
                    metaAddr = startAddr
                    print("Resident metadata detected, using metaAddr = startAddr")
                    print("Note: Resident metadata will still be overwritten")
                else:
                    # See if the range leaves room
                    if vargs.range is None or vargs.range[0] is None:
                        # First address was not specified, expand backwards
                        if vargs.oadtype == 'onchip' and vargs.imgtype == 'production':
                            #TODO, remember imgA start and try to start with 16B offset from here, until then assume it is here
                            metaAddr = INT_FL_OAD_IMG_A_META_BEGIN
                            startAddr = INT_FL_OAD_IMG_A_META_BEGIN
                        else:
                            metaAddr = startAddr - 0x10
                            startAddr = metaAddr
                        if addr_is_in_oad_imgspace(metaAddr):
                            print("Expanded address range. Placed metadata at 0x%08X" % metaAddr)
                        else:
                            print("Fatal Error: -- Could not find free area for metadata before 0x%08X. Exiting." % startAddr)
                            sys.exit(1)
                    else:
                        #maybe improve this comment
                        print("Fatal Error: -- Could not find free area for metadata in --range specified to start at 0x%08X. Exiting.")
                        sys.exit(1)
        else:
            # User provided metadata location
            # Given current OAD method there is a fixed location for metadata given the OAD method (on or off chip)
            if vargs.oadtype == 'onchip' and vargs.imgtype == 'production':
                imgStartAddr = INT_FL_OAD_IMG_A_META_BEGIN
            else:
                imgStartAddr = startAddr

            if metaAddr > imgStartAddr or not addr_is_in_oad_imgspace(metaAddr):
                print("Fatal Error: -- Metadata must be at the start, additionally, must be within APP+STACK bounds. Exiting." % metaAddr)
                sys.exit(1)
            else:
                # Trust the user
                print("Placing metadata at 0x%08X" % metaAddr)
                startAddr = metaAddr

        # Calculate metadata
        if vargs.oadtype == 'onchip' and vargs.imgtype == 'production':
            imgLen = int(math.ceil((INT_FL_OAD_IMG_A_END - INT_FL_OAD_IMG_A_META_BEGIN) / 4.0)) # Image length in words
            imgVer = 0 << 1 # LSB means A/B .... TODO
        elif vargs.oadtype == 'onchip':
            imgLen = int(math.ceil((endAddr - startAddr) / 4.0)) # Image length in words
            imgVer = 1 # LSB means A/B .... TODO
        else:
            imgLen = int(math.ceil((endAddr - startAddr) / 4.0)) # Image length in words
            imgVer = vargs.imgVer # LSB means A/B .... TODO


        usrId = struct.pack('B', ord('E')) * 4 # Well.. TODO.
        imgAddr = startAddr / 4  # In words
        imgType = imgTypes[vargs.imgtype]
        crcShdw = 0xffff

        meta = struct.pack('HHHH4sHBB', crcShdw, crcShdw, imgVer, imgLen, usrId, imgAddr, imgType, META_STATUS_SUCCESS)
        mergedHex.puts(metaAddr, meta)
        #production image only calculates over imgA region
        if vargs.oadtype == 'onchip' and vargs.imgtype == 'production':
            asBin = mergedHex.tobinstr(INT_FL_OAD_IMG_A_META_BEGIN, INT_FL_OAD_IMG_A_END)
        else:
            asBin = mergedHex.tobinstr(startAddr, endAddr-1)
        crc = crc16(asBin[4:])
        mergedHex.puts(metaAddr, struct.pack('H', crc))

        metaVector = [crc, crcShdw, imgVer, imgLen, usrId, imgAddr, imgType, META_STATUS_SUCCESS]

        print_metadata(metaVector)

    #mergedHex.puts(metaAddr, struct.pack('B', 0xff)*16)

    # Output merged hex file unless both hex and bin is stdout, in which case bin wins
    if not (vargs.out is sys.stdout and vargs.outbin is sys.stdout):
        print("Writing to:\n", vargs.out.name)
        mergedHex.write_hex_file(vargs.out)
        vargs.out.flush()

    # Output binary
    if vargs.outbin is not None:
        print("Writing to:\n", vargs.outbin.name)
        for ch in mergedHex.tobinstr(startAddr, endAddr-1):
            vargs.outbin.write(ch)
        vargs.outbin.flush()

    print("******************************************************************************************")
    print("Success")
    print("******************************************************************************************")
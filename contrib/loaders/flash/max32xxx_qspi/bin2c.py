# Convert binary file to a hex encoded array for inclusion in C projects
 
import os
import struct
 
import logging
 
class BinToArray:
    def __init__(self):
        pass
 
    def ConvertFileToArray(self, strInFile, strOutFile, strArrayName, integerSize, ignoreBytes, endianNess):
        """ Reads binary file at location strInFile and writes out a C array of hex values
            Parameters - 
                strInFile - Path and filename of binary file to convert
                strOutFile - Path and filename of output. Suggested extension is .c or .cpp
                integerSize - Size in bytes of output array elements. Array generated is always
                    of type uint8, uint16, uint32. These types would need to be defined using
                    typedef if they don't exist, or the user can replace the type name with the
                    appropriate keyword valid for the compiler size conventions
                ignoreBytes - Number of bytes to ignore at the beginning of binary file. Helps
                    strip out file headers and only encode the payload/data.
                endianNess - Only used for integerSize of 2 or 4. 'l' for Little Endian, 'b' for
                    Big Endian
        """
        # Check integerSize value
        if integerSize not in (1, 2, 4):
            logging.debug("Integer Size parameter must be 1, 2 or 4")
            return
        # endif
        # Open input file
        try:
            fileIn = open(strInFile, 'rb')
        except IOError, err:
            logging.debug("Could not open input file %s" % (strInFile))
            return
        # end try
        # Open output file
        try:
            fileOut = open(strOutFile, 'w')
        except IOError, err:
            logging.debug("Could not open output file %s" % (strOutFile))
            return
        # end try
        # Start array definition preamble
        inFileName = os.path.basename(strInFile)
        strVarType = "static uint%d_t" % (integerSize * 8)
        fileOut.write("%s " % strVarType)
        fileOut.write("%s[] = {\n" % strArrayName)
        # Convert and write array into C file
        fileIn.seek(ignoreBytes)
        if integerSize == 1:
            bufChunk = fileIn.read(20)
            while bufChunk != '':
                fileOut.write("  ")
                for byteVal in bufChunk:
                    fileOut.write("0x%02x, " % ord(byteVal))
                # end for
                fileOut.write("\n")
                bufChunk = fileIn.read(20)
            # end while
        else:
            if   endianNess == 'l' and integerSize == 2:
                endianFormatter = '<H'
                maxWordsPerLine = 10
            elif endianNess == 'l' and integerSize == 4:
                endianFormatter = '<L'
                maxWordsPerLine = 6
            elif endianNess == 'b' and integerSize == 2:
                endianFormatter = '>H'
                maxWordsPerLine = 10
            elif endianNess == 'b' and integerSize == 4:
                endianFormatter = '>L'
                maxWordsPerLine = 6
            # endif
            bufChunk = fileIn.read(integerSize)
            i = 0
            fileOut.write("  ")
            while bufChunk != '':
                if len(bufChunk) < integerSize:
                  tempVal = 0
                  for byteVal in bufChunk:
                    tempVal = (tempVal << 8) + ord(byteVal)
                  # end for
                  tempVal = (tempVal << (8 * (integerSize - len(bufChunk))))
                  if endianNess == 'l':
                    wordVal = (tempVal & 0xFF) << 24
                    wordVal += (tempVal & 0xFF00) << 8
                    wordVal += (tempVal & 0xFF0000) >> 8
                    wordVal += (tempVal & 0xFF000000) >> 24
                  else:
                    wordVal = tempVal
                  # endif
                else:
                  wordVal = struct.unpack(endianFormatter, bufChunk)
                # endif
                if integerSize == 2:
                    fileOut.write("0x%04x, " % wordVal)
                else:
                    fileOut.write("0x%08x, " % wordVal)
                # endif
                i += 1
                if i == maxWordsPerLine:
                    fileOut.write("\n  ")
                    i = 0
                # endif
                bufChunk = fileIn.read(integerSize)
            # end while
        # end if
        # Close array definition
        fileOut.write("};\n")
        fileIn.close()
        fileOut.close()
 

if __name__ == '__main__':
    import getopt
    import os
    import sys

    usage = '''Bin2C python converting utility.

Usage:
    python bin2c.py [options] file.bin [out.c]

Arguments:
    file.bin                name of binary file to process
    out.c                   name of output file
                            If omitted then output write to file.c

Options:
    -h, --help              this help message
    -s, --size=X            size in bytes of output array elements
    -i, --ignore=X          number of bytes to ignore at the beginning of binary file
    -l, --little-endian     endianness (only used for integerSize of 2 or 4)
    -b, --big-endian        endianness (only used for integerSize of 2 or 4)
'''

    size = 1
    ignore = 0
    endianness = 'l'

    try:
        opts, args = getopt.getopt(sys.argv[1:], "hlbs:i:", ["help", "size=", "ignore="])

        for o, a in opts:
            if o in ("-h", "--help"):
                print usage
                sys.exit(0)
            elif o in ("-s", "--size"):
                try:
                    size = int(a, 10)
                except:
                    raise getopt.GetoptError, 'Bad size value'
            elif o in ("-i", "--ignore"):
                try:
                    ignore = int(a, 10)
                except:
                    raise getopt.GetoptError, 'Bad size value'
            if o in ("-l", "--little-endian"):
                endianness = 'l'
            if o in ("-b", "--big-endian"):
                endianness = 'b'

        if not args:
            raise getopt.GetoptError, 'Input binary file is not specified'

        if len(args) > 2:
            raise getopt.GetoptError, 'Too many arguments'

    except getopt.GetoptError, msg:
        print msg
        print usage
        sys.exit(2)

    fin = args[0]
    import os.path
    if len(args) == 1:
        name, ext = os.path.splitext(fin)
        fout = name + ".c"
    else:
        fout = args[1]

    arrayname, ext = os.path.splitext(fout)
    arrayname = os.path.basename(arrayname)

    if not os.path.isfile(fin):
        print "File not found"
        sys.exit(1)

    logging.basicConfig(level=logging.DEBUG)
    converter = BinToArray()
    converter.ConvertFileToArray(fin, fout, arrayname, size, ignore, endianness)

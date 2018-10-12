class RTCM_msg(object):

    def __init__(self, array_of_bytes):
        self.complete_frame = array_of_bytes
        self.preamble   = array_of_bytes[0]
        self.header     = array_of_bytes[1:3]
        self.payload    = array_of_bytes[3:-3]
        self.crc        = array_of_bytes[-3:]

    def write_to_file(self,filename):
        with open(filename, "a") as myfile:
            myfile.write(str(self))
            myfile.close()

    def get_bytearray(self):
        return self.complete_frame

    def __len__(self):
        return len(self.complete_frame)

    def __str__(self):
        printout = "Preamble: " + hex(self.preamble) + "\n"
        printout += "Header: "
        for byte in self.header:
            printout += hex(byte) + " "
        printout += "\n"

        printout += "Payload: "
        for byte in self.payload:
            printout += hex(byte) + " "
        printout += "\n"

        printout += "CRC: "
        for byte in self.crc:
            printout += hex(byte) + " "
        printout += "\n"
        printout += "\n"

        return printout
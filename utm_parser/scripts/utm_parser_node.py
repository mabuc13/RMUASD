#!/usr/bin/env python
# -*- coding: utf-8 -*-
import signal
import sys
import rospy
import time
import struct

"""
Descriptors: MB = Mark Buch (mabuc13@student.sdu.dk)
2018-10-06 MB created file heavily inspired by example from Tobias Lundby @ SDU
"""

"""
Description:
This class will handle post and get to the UTM from the GCS. 
The idea is to have each request as a seperate method.

License: BSD 3-Clause
"""

import sys
import requests
import json
#import sh
import csv
import simplekml
from termcolor import colored
from kml_reader import kml_no_fly_zones_parser
from time import gmtime, strftime
# Disable warning
from requests.packages.urllib3.exceptions import InsecureRequestWarning
requests.packages.urllib3.disable_warnings(InsecureRequestWarning)

class utm_parser(object):
    def __init__(self):
        self.payload = {
            ''
        }
        self.kml = kml_no_fly_zones_parser()
        self.static_filename = ''
    #ROS STUFF
    def shutdownHandler(self):
        # shutdown services
        print("Shutting down")
    #Helper methods
    def create_kml(self, file_name, data): #Formats special characters out of a string and saves it as a kml file with file_name

        f = open(file_name,"w+")
        line_array = data.split("\n")
        for i in line_array:
            i = i.replace('æ', 'ae')
            i = i.replace('ø', 'oo')
            i = i.replace('å', 'aa')

            i = i.replace('Æ', 'AE')
            i = i.replace('Ø', 'OO')
            i = i.replace('Å', 'AE')

            i = i.replace('é', 'e')
            i = i.replace('ö', 'oo')

            f.write(i)
            f.write("\n")

            #f.write("\n")

        f.close()

    def format_string(self, string): #Formats special characters out of a string and returns it

        line_array = string.split("\n")
        superstring = ''
        for i in line_array:
            i = i.replace('æ', 'ae')
            i = i.replace('ø', 'oo')
            i = i.replace('å', 'aa')

            i = i.replace('Æ', 'AE')
            i = i.replace('Ø', 'OO')
            i = i.replace('Å', 'AE')

            i = i.replace('é', 'e')
            i = i.replace('ö', 'oo')

            superstring += i
            superstring += '\n'

        return superstring

    def englify(self, string): #This method will remove the danish characters
        string.replace('æ', "ae")
        string.replace('ø', "oo")
        string.replace('å', "aa")
        return string

    def get_static_nfz(self):
        """
        input: none
        output: the output will be a string with the filename / location of the file and an int with the response code from the UTM
        """

        self.payload = {
            'data_type': 'static_no_fly'
        }
        r = ''
        try:
            r = requests.get(url = 'https://droneid.dk/rmuasd/utm/data.php', params = self.payload, timeout=2)
            r.raise_for_status()
        except requests.exceptions.Timeout:
            # Maybe set up for a retry, or continue in a retry loop
            print colored('Request has timed out', 'red')
        except requests.exceptions.TooManyRedirects:
            # Tell the user their URL was bad and try a different one
            print colored('Request has too many redirects', 'red')
        except requests.exceptions.HTTPError as err:
            print colored('HTTP error', 'red')
            print colored(err, 'yellow')
            #sys.exit(1) # Consider the exit since it might be unintentional in some cases
        except requests.exceptions.RequestException as err:
            # Catastrophic error; bail.
            print colored('Request error', 'red')
            print colored(err, 'yellow')
            sys.exit(1)
        else:
            print colored('Status code: %i' % r.status_code, 'yellow')
            print colored('Content type: %s' % r.headers['content-type'], 'yellow')

            data_dict = ''
            try:
                string = r.content
                string = string[:-7]
                #filename = str(time.now())
                self.static_filename = strftime("%a, %d %b %Y %H:%M:%S +0000", gmtime())
                self.create_kml(self.static_filename, string)

            except:
                print colored('Error in parsing to ktml parser', 'red')
            else:
                self.kml.parse_file(self.static_filename)
                self.kml.print_zones()

                #for i in line_array:
                   # pos = i.find("\t")
                    #print pos
                #f.close()
                #rospy.sleep(1) # MAYBE this is unnescarry




                #string = self.format_string(string)
                #print string
                #self.kml.parse_data(string)

                #print string
                #superstring = superstring[:-1]
                #self.kml.parse_data(string)

                #print superstring
                #self.kml.parse_data(superstring)
                #print r.text # Print the raw body data
                #for entry in data_dict: # The loop could be omitted since there should only be 1 entry and the header exception should catch a request for a UAV ID which does not exist.
                #    print entry
    def print_zones(self):
        dummy = 2

def main():
        rospy.init_node('utm_parser')#, anonymous=True)
        rospy.sleep(1)

        par = utm_parser()
        rospy.on_shutdown(par.shutdownHandler)



        par.get_static_nfz()
        par.print_zones()

        rospy.spin()

if __name__ == "__main__":
    main()
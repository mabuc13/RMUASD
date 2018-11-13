#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Descriptors: TL = Tobias Lundby (tolu@mmmi.sdu.dk)
2018-09-21 TL Created file
"""

"""
Description:
Example of how to register a UAV and operator using the 'Register UAV' API in order to get a UAV ID and authentication key.
License: BSD 3-Clause
"""

import sys
import requests
import json
from termcolor import colored
import time

# Disable warning
from requests.packages.urllib3.exceptions import InsecureRequestWarning
requests.packages.urllib3.disable_warnings(InsecureRequestWarning)

def ask_user():
    check = str(raw_input("Register UAV and operator ? (Y/N): ")).lower().strip()
    try:
        if check[0] == 'y':
            return True
        elif check[0] == 'n':
            return False
        else:
            print('Invalid Input')
            return ask_user()
    except Exception as error:
        print("Please enter valid inputs")
        print(error)
        return ask_user()

if __name__ == '__main__':
    payload = {
        'uav_name': 'SDU RM-UASDE18 Group 2',
        'uav_weight_kg': 2,
        'uav_max_vel_mps': 15,
        'uav_max_endurance_s': 600,
        'gdpr_compliance': 'yes',
        'operator_name': 'Kristian Hansen',
        'operator_phone': '+45 5184 0967',
        'operator_drone_cert': 5099205
    }

    print colored('UAV and operator information', 'yellow')
    print '  UAV name: %s' % payload['uav_name']
    print '  UAV weight [kg]: %.02f' % payload['uav_weight_kg']
    print '  UAV max. vertical velocity [m/s]: %.02f' % payload['uav_max_vel_mps']
    print '  UAV max. endurance [s]: %i' % payload['uav_max_endurance_s']
    print '  GDPR compliance: %s' % payload['gdpr_compliance']
    print '  Operator phone: %s' % payload['operator_phone']
    print '  Operator drone certificate: %i' % payload['operator_drone_cert']

    if ask_user(): #Make the user verify that the information is correct
        print colored('\nTrying to register UAV and operator...', 'yellow')
        r = ''
        try:
            r = requests.post(url = 'https://droneid.dk/rmuasd/utm/uav.php', data = payload, timeout=2)
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
            print colored('Success!\n', 'green')
            print colored('Status code: %i' % r.status_code, 'yellow')
            print colored('Content type: %s' % r.headers['content-type'], 'yellow')

            data_dict = ''
            try:
                data_dict = json.loads(r.text) # convert to json
            except:
                print colored('Error in parsing of data to JSON', 'red')
            else:
                #print r.text # Print the raw body data
                print colored( 'Issued UAV ID: %i, authentication key: %s' % (data_dict['uav_id'], data_dict['uav_auth_key']), 'green' )
                print colored('\nTrying to save registration information to file...', 'yellow')
                try:
                    f = open('uav_registration.txt', 'w')
                    f.write('time=%f,uav_id=%i,uav_auth_key=%s' % (time.time(), data_dict['uav_id'], data_dict['uav_auth_key']))
                except:
                    print colored('Could not save registration information', 'red')
                else:
                    print colored('Saved!', 'green')

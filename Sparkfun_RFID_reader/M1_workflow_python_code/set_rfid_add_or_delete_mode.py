'''
This workflow sets the operation mode: Add/Delete/Verify RFID card key

Last Updated: June 19, 2018

Author: Medium One
'''

import MQTT
import Store

adduser = IONode.get_input('in1')['event_data']['value']
deluser = IONode.get_input('in2')['event_data']['value']

if (adduser == 'on' and deluser == 'on'):  # illegal case
    new_value = "verify"  
    msg = "Verify card ID: Please scan your card."
elif (adduser == 'on'):  
    new_value = "add"
    msg = "Add card ID: Please scan your card."
elif (deluser == 'on'):
    new_value = "delete"
    msg = "Delete card ID: Please scan your card."
else:                  # add off or del off case
    new_value = "verify"  
    msg = "Verify card ID: Please scan your card."
    
Store.set_data("rfid_mode", new_value, -1)

# push (bool) to send this notification to push (iOS or Android)
IONode.set_output('out1', {"message": msg, "sms":False,"email":False,"push":True})

# for debug purpose
log('new value =' )
log(new_value)


'''
This workflow will process the scanned card key according to the following modes: Add/Delete/Verify RFID card key

Last Updated: June 19, 2018

Author: Medium One
'''

import MQTT
import Store

cardkey =  IONode.get_input('in1')['event_data']['value']

secretcardkey = Store.get("cardkey")   # Get the stored secret card key to compare for verify card key mode.

verifyflag = True                      # Set verify pass.

mode = Store.get("rfid_mode")          # Get the stored card key operation mode.
if mode == "add":                      # Add card key mode.
    log('Add Mode')
    sms_msg = "Add card"
    if (secretcardkey == None or secretcardkey == ""):    # Allow to add a new key if the previous secret card key has been deleted first.   
        Store.set_data("cardkey", cardkey, -1)
        verifyflag = True   # Set Pass to indicate that the new secret card key is added successfully.
    else:    
        verifyflag = False  # Set Fail to indicate that the new secret card key is not added.
elif mode == "delete":
    log('Delete Mode')
    sms_msg = "Delete card"      
    if (secretcardkey == cardkey):   # Delete the current secret card key only if the scanned card key matches it.
        Store.set_data("cardkey", "", -1)  
        verifyflag = True   # Set Pass to indicate that the new secret card key is deleted successfully.
    else:    
        verifyflag = False  # Set Fail to indicate that the new secret card key is not deleted.      
elif mode == "verify": 
    sms_msg = "Verify card"
    log('Verify Mode')
    if (secretcardkey == cardkey):
        verifyflag = True   # Set Pass to indicate that they are matched.
    else:    # not matched or secretcardkey is None or ""
        verifyflag = False  # Set Pass to indicate that they are matched.
else:
    sms_msg = "Error Invalid Mode"   #unknown mode
    log('Invalid Mode')
    verifyflag = False  # Show fail to indicate that there is a problem.

# show the card ID    
sms_msg = sms_msg + ' (' + cardkey + ').  '

#send the pass/fail flag to MCU, so the MCU can blink the green led or red led to indicate that the operation fails or pass respectively.
if (verifyflag):        
    sms_msg = sms_msg + " Pass!"
    MQTT.publish_event_to_client('s5d9', 'V1')   
else:    
    sms_msg = sms_msg + " Fail!"
    MQTT.publish_event_to_client('s5d9', 'V0')                 

# push (bool) to send this notification to push (iOS or Android)
IONode.set_output('out1', {"message": sms_msg, "sms":False,"email":False,"push":True})

# For debug purpose
log('mode =' )
log(verifyflag)    
log('verify =' )
log(verifyflag)    
log('secret card key =' )
log(secretcardkey)  
newsecretcardkey = Store.get("cardkey")        
log('new secret card key =' )
log(newsecretcardkey)  

import os
import json
import random
import subprocess
import speech_recognition as sr

import rospy
from std_msgs.msg import String

from utils.coba_trt import infer_text, init



WAKE = 'hello nico'

def rec(device, model, pub):
    command = String()
    rec = sr.Recognizer()
    mic = sr.Microphone(device_index=device)
    
    with mic as s:
        rec.adjust_for_ambient_noise(s, duration=1)
        rospy.loginfo('Ready')
        
        while not rospy.is_shutdown():
            try:
                v = rec.listen(s)
                try:
                    w = rec.recognize_google(v).lower()
                    rospy.loginfo(f'Hear:{w}')
                    
                    if WAKE in w:
                        rospy.loginfo('Listening to user command')
                        speak_callback('wake')

                        w = rec.listen(s)
                        c = rec.recognize_google(w).lower()
                        
                        if 'stop' in c:
                            pub.publish('stop')
                            speak_callback('stop')
                            continue
                        
                        command.data = infer_text(c, model)
                        rospy.loginfo(f'Command: [{command}] {c}')
                        pub.publish(command)
                        speak_callback(command.data)
                        
                except sr.UnknownValueError:
                    rospy.logwarn('Cannot understand audio')
                except sr.RequestError as e:
                    rospy.logerr(f'Cannot solve request results: {e}')
            except Exception as e:
                rospy.logerr(f'Error: {e}')

def speak_callback(msg):
    cat = 'conversation'

    if 'wake' in msg:
        r = get_r(cat, 'wake')
    elif 'stop' in msg:
        r = get_r(cat, 'stop')
    else:
        r = get_r(cat, 'stop', msg)
    text_to_speech(r)

def speech_callback(msg):
    cat = 'info'
    
    script_dir = os.path.dirname(os.path.abspath(__file__))
    nico_path = os.path.join(script_dir, "nico.wav")
    jar_path = os.path.join(script_dir, "jarvis.wav")
    if 'ready' in msg.data:
        play_audio(jar_path, d=False)
        r = ''
    elif 'scan' in msg.data:
        p = msg.data.split()
        r = get_r(cat, 'scan', p={'scan_obj' : p[1]}) 
    elif 'done' in msg.data:
        r = get_r(cat, 'calib')
    elif 'nfound' in msg.data:
        p = msg.data.split()
        r = get_r(cat, 'nfound', p={'nfound_obj' : p[1]})
    elif 'success' in msg.data:
        p = msg.data.split()
        r = get_r(cat, 'success', p={'obj' : p[1]})
    else:
        p = msg.data.split()
        r = get_r(cat, 'dir', p={'dir' : p[0], 'obj' : p[1]})

    text_to_speech(r)

def main():
    rospy.init_node('com', anonymous=True)
    model = init()
    pub = rospy.Publisher('/user_command', String, queue_size=10)
    sub = rospy.Subscriber('/out_speech', String, speech_callback)
    
    rec(device, model, pub)

    rospy.spin()

def get_r(cat, s_cat, p=None):
    r = random.choice(response[cat][s_cat])

    if p:
        r = r.format(**p)

    return r

def text_to_speech(text, language="en-US", output_file="output.wav"):
    try:
        # Use pico2wave to generate the WAV file
        subprocess.run(["pico2wave", "-l", language, "-w", output_file, text], check=True)
        rospy.loginfo(f'Said: {text}')
        play_audio(output_file)
    except subprocess.CalledProcessError as e:
        rospy.logerr(f"Error: {e}")

def play_audio(file_path, d=True):
    try:
        subprocess.run(["aplay", file_path], check=True)
    except subprocess.CalledProcessError as e:
        rospy.logerr(f"Error: {e}")

    if d:
        os.remove(file_path)

if __name__ == "__main__":
    global response
    global device
    
    script_dir = os.path.dirname(os.path.abspath(__file__))
    json_path = os.path.join(script_dir, "response.json")
    with open(json_path, "r", encoding="utf-8") as file:
        response = json.load(file)

    for index, name in enumerate(sr.Microphone.list_microphone_names()):
        if 'AB13X' in name:
            device = index
            print(name)
            break
    
    main()
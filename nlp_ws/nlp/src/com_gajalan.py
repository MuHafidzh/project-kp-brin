import os
import json
from pathlib import Path
import random
import queue
import subprocess
import pyaudio
import rospy
from std_msgs.msg import String
from vosk import Model, KaldiRecognizer
from utils.coba_trt import infer_text, init

# Define wake word
WAKE = 'hello'

# Load Vosk model
FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]
MODEL_PATH = str(ROOT) + "/" + "vosk_en_small"
model = Model(MODEL_PATH)

# Audio streaming parameters
SAMPLE_RATE = 48000
CHUNK_SIZE = 4000  # Adjust buffer size if necessary

# Initialize PyAudio
p = pyaudio.PyAudio()

def get_microphone_index():
    """Finds the microphone index for the specified device."""
    for index in range(p.get_device_count()):
        info = p.get_device_info_by_index(index)
        if "AB13X" in info["name"]:  # Adjust for your specific microphone name
            print(f"Using microphone: {info['name']}")
            return index
    return None  # If no matching microphone is found

device_index = get_microphone_index()
if device_index is None:
    raise ValueError("Microphone AB13X not found! Check your input device.")

def rec(model_, pub):
    """Recognizes voice commands from the microphone in real-time."""
    command = String()
    
    stream = p.open(format=pyaudio.paInt16,
                    channels=1,
                    rate=SAMPLE_RATE,
                    input=True,
                    frames_per_buffer=CHUNK_SIZE,
                    input_device_index=device_index)

    recognizer = KaldiRecognizer(model, SAMPLE_RATE)
    recognizer.SetWords(True)  # Enable word-by-word output

    rospy.loginfo("🎤 Ready. Listening for wake word...")

    while not rospy.is_shutdown():
        try:
            data = stream.read(CHUNK_SIZE, exception_on_overflow=False)
            
            if recognizer.AcceptWaveform(data):
                result = json.loads(recognizer.Result())
                transcript = result.get("text", "").lower()
                rospy.loginfo(f"🎤 Heard: {transcript}")

                if WAKE in transcript:
                    rospy.loginfo("🔊 Wake word detected! Listening for command...")
                    speak_callback("wake")

                    # Listen for command
                    data = stream.read(CHUNK_SIZE, exception_on_overflow=False)
                    if recognizer.AcceptWaveform(data):
                        result = json.loads(recognizer.Result())
                        command_text = result.get("text", "").lower()

                        # if not command_text:
                        #     rospy.logwarn("⚠ No command detected.")
                        #     continue

                        if "stop" in command_text:
                            pub.publish("stop")
                            speak_callback("stop")
                            continue

                        # Process the command
                        command.data = infer_text(command_text, model_)
                        rospy.loginfo(f"📢 Command: [{command.data}] {command_text}")
                        pub.publish(command)
                        speak_callback(command.data)
                            
        except Exception as e:
            rospy.logerr(f"Error: {e}")

def speak_callback(msg):
    """Handles text-to-speech responses."""
    cat = "conversation"

    if "wake" in msg:
        r = get_r(cat, "wake")
    elif "stop" in msg:
        r = get_r(cat, "stop")
    else:
        r = get_r(cat, "stop", msg)

    text_to_speech(r)

def speech_callback(msg):
    """Handles speech-related responses."""
    cat = "info"
    
    script_dir = os.path.dirname(os.path.abspath(__file__))
    nico_path = os.path.join(script_dir, "nico.wav")
    jar_path = os.path.join(script_dir, "jarvis.wav")

    if "ready" in msg.data:
        play_audio(jar_path, d=False)
        r = ""
    elif "scan" in msg.data:
        p = msg.data.split()
        r = get_r(cat, "scan", p={"scan_obj": p[1]})
    elif "done" in msg.data:
        r = get_r(cat, "calib")
    elif "nfound" in msg.data:
        p = msg.data.split()
        r = get_r(cat, "nfound", p={"nfound_obj": p[1]})
    elif "success" in msg.data:
        p = msg.data.split()
        r = get_r(cat, "success", p={"obj": p[1]})
    else:
        p = msg.data.split()
        r = get_r(cat, "dir", p={"dir": p[0], "obj": p[1]})

    text_to_speech(r)

def main():
    """ROS Node Initialization."""
    rospy.init_node("com", anonymous=True)
    model = init()
    pub = rospy.Publisher("/user_command", String, queue_size=10)
    sub = rospy.Subscriber("/out_speech", String, speech_callback)
    
    rec(model, pub)

    rospy.spin()

def get_r(cat, s_cat, p=None):
    """Fetches a response from JSON."""
    r = random.choice(response[cat][s_cat])

    if p:
        r = r.format(**p)

    return r

def text_to_speech(text, language="en-US", output_file="output.wav"):
    """Converts text to speech using pico2wave."""
    try:
        subprocess.run(["pico2wave", "-l", language, "-w", output_file, text], check=True)
        rospy.loginfo(f"Said: {text}")
        play_audio(output_file)
    except subprocess.CalledProcessError as e:
        rospy.logerr(f"Error: {e}")

def play_audio(file_path, d=True):
    """Plays an audio file."""
    try:
        subprocess.run(["aplay", file_path], check=True)
    except subprocess.CalledProcessError as e:
        rospy.logerr(f"Error: {e}")

    if d:
        os.remove(file_path)

if __name__ == "__main__":
    global response
    
    script_dir = os.path.dirname(os.path.abspath(__file__))
    json_path = os.path.join(script_dir, "response.json")
    with open(json_path, "r", encoding="utf-8") as file:
        response = json.load(file)

    main()

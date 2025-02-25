import rospy
from std_msgs.msg import String
import speech_recognition as sr
from utils.coba_trt import infer_text, init

WAKE = "hello nico"  # Define wake word

pub = rospy.Publisher('/user_command', String, queue_size=10)

def rec(device, model):
    """ Continuously listens for the wake word and publishes recognized commands to a ROS topic. """
    rec = sr.Recognizer()
    mic = sr.Microphone(device_index=device)
    # mic = sr.Microphone()
    
    with mic as source:
        rec.adjust_for_ambient_noise(source, duration=1)
        print(f"Listening for wake word '{WAKE}'...")

        while rospy.is_shutdown() == False:
            try:
                # Listen for wake word
                print("Waiting for wake word...")
                audio = rec.listen(source)

                try:
                    detected_text = rec.recognize_google(audio).lower()
                    print(f"Hear: {detected_text}")

                    if WAKE in detected_text:
                        print("Wake word detected. Listening for user command...")

                        # Listen for user command
                        audio = rec.listen(source)
                        command_text = rec.recognize_google(audio).lower()
                        print(f'Hear: {command_text}')

                        if "stop" in command_text:
                            print("Stopping voice recognition...")
                            break
                        user_msg = String()
                        user_command = infer_text(command_text, model)
                        user_msg.data = user_command
                        pub.publish(user_msg)
                        
                        print(f"User command: {user_command}")

                except sr.UnknownValueError:
                    print("Could not understand the audio.")
                except sr.RequestError as e:
                    print(f"Request error: {e}")

            except Exception as e:
                print(f"Error: {e}")

if __name__ == "__main__":
    rospy.init_node("voice_recognition")

    for i, name in enumerate(sr.Microphone.list_microphone_names()):
        if 'AB13X' in name:
            device = i
            break
    model = init()
    rec(device, model)

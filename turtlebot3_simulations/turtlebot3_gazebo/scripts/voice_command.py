#!/usr/bin/env python3

import rospy
import speech_recognition as sr
from std_msgs.msg import String
import noisereduce as nr
import numpy as np
import sys
import select
import termios
import tty

def recognize_speech():
    recognizer = sr.Recognizer()
    mic = sr.Microphone()  # Use the default microphone

    with mic as source:
        print("Please say the desired location...")
        recognizer.adjust_for_ambient_noise(source, duration=2)  # Adjust for ambient noise
        print("Adjusted for ambient noise")
        audio = recognizer.listen(source, timeout=5)  # Listen for up to 5 seconds
        print("Audio captured")

    # Convert audio data to numpy array for noise reduction
    audio_data = np.frombuffer(audio.get_raw_data(), np.int16)
    reduced_noise_audio = nr.reduce_noise(y=audio_data, sr=audio.sample_rate)
    reduced_noise_audio_bytes = reduced_noise_audio.tobytes()

    # Create a new AudioData instance with the noise-reduced data
    reduced_audio = sr.AudioData(reduced_noise_audio_bytes, audio.sample_rate, audio.sample_width)

    try:
        command = recognizer.recognize_google(reduced_audio)
        print(f"You said: {command}")
        return command
    except sr.UnknownValueError:
        print("Sorry, I did not understand that.")
        return None
    except sr.RequestError:
        print("Could not request results; check your network connection.")
        return None

def is_spacebar_pressed():
    # Check if the spacebar has been pressed
    dr, dw, de = select.select([sys.stdin], [], [], 0)
    if dr:
        key = sys.stdin.read(1)
        return key == ' '
    return False

def main():
    rospy.init_node('voice_command_publisher', anonymous=True)
    pub = rospy.Publisher('/user_command', String, queue_size=10)

    # Configure the terminal to read single characters (non-blocking input)
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    try:
        print("Press spacebar to activate the microphone and speak your command.")
        while not rospy.is_shutdown():
            if is_spacebar_pressed():  # Wait for spacebar press
                command = recognize_speech()
                if command:
                    print(f"Publishing command: {command}")
                    pub.publish(command)
                else:
                    print("No command recognized")
            rospy.sleep(0.1)  # Sleep briefly to avoid busy-waiting
    finally:
        # Reset terminal to original settings when done
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

if __name__ == "__main__":
    main()

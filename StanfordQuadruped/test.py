from gen2emotionrecognition import main as emotion

current_emotion = emotion.Emotion_Detection
while True:
    current_emotion = emotion.Emotion_Detection
    if current_emotion == "neutral":
        print("Neutral")
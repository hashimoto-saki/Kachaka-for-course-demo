#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import time
import openai
sys.path.append(os.path.expanduser("~/Kachaka_hashimoto_2024/kachaka-api/python"))
import kachaka_api
import speech_recognition as sr

# ======= 設定 =======
KACHAKA_IP = "192.168.43.143:26400"
API_KEY_PATH = os.path.join(os.path.dirname(__file__), "API_key.txt")
PROMPT_PATH = os.path.join(os.path.dirname(__file__), "task_planning.txt")
DEFAULT_SHELF_ID = "S01"

# ======= 初期化 =======
client = kachaka_api.KachakaApiClient(KACHAKA_IP)

# ======= 音声認識 =======
def listen_from_microphone():
    recognizer = sr.Recognizer()
    with sr.Microphone() as source:
        print("マイクを調整中...")
        time.sleep(1)  # Kachakaがしゃべる前に少し待つ
        recognizer.adjust_for_ambient_noise(source, duration=1)

        while True:
            print("マイクが起動しました。何か話してください。")
            client.speak("マイクが起動しました。何か話してください。")
            time.sleep(4)  # Kachakaの発話が終わるのを待ってから録音開始

            try:
                audio = recognizer.listen(source, timeout=5, phrase_time_limit=10)
                text = recognizer.recognize_google(audio, language="ja-JP")
                print("認識したテキスト：", text)
                return text

            except sr.WaitTimeoutError:
                print("タイムアウト: 音声が検出されませんでした。")
                client.speak("音声が聞こえませんでした。もう一度お願いします。")

            except sr.UnknownValueError:
                print("音声を認識できませんでした。もう一度話してください。")
                client.speak("音声を認識できませんでした。もう一度話してください。")

            except sr.RequestError as e:
                sys.exit(f"[エラー] Google Speech Recognition の接続に失敗しました: {e}")



# ======= GPTを使ってタスクと場所を決定 =======
def task_planning(user_instruction):
    """
    GPTを使ってuser_instructionからタスクとロケーションを抽出する。
    """
    with open(API_KEY_PATH, "r") as f:
        openai.api_key = f.read().strip()
    with open(PROMPT_PATH, "r", encoding="utf-8") as f:
        prompt = f.read()
    full_prompt = prompt.replace("USER_INSTRUCTION", user_instruction)
    messages = [{'role': 'user', 'content': full_prompt}]
    response = openai.ChatCompletion.create(
        model='gpt-4',
        messages=messages,
        temperature=0.0
    )
    reply = response['choices'][0]['message']['content']

    print(f"[GPT応答] {reply}")
    output_data = reply.split('=')[1].strip().strip('()')
    task, location = [s.strip().strip('"') for s in output_data.split(',')]
    print(f"[抽出結果] タスク: {task}, ロケーション: {location}")
    return task, location


# ======= メイン処理 =======
def main():
    print("[INFO] Kachakaの起動待機中（10秒）...")
    time.sleep(5)  # Kachaka起動を待つ

    user_question = listen_from_microphone()
    # user_question = "トイレ行って"
    task, location = task_planning(user_question)

    print(f"[INFO] タスク = {task}, 場所 = {location}")

    if task == 'move':
        client.speak("目的地に移動します")
        client.move_to_location(location)

    elif task == 'bring':
        client.speak("棚を使って運びます")
        client.move_shelf(DEFAULT_SHELF_ID, location)
        time.sleep(5)
        client.return_shelf()

    elif task == 'return':
        client.speak("ホームに帰ります")
        client.return_home()

    elif task == 'speak':
        client.speak(location)

    else:
        client.speak("タスクが認識できませんでした")
        print(f"[WARN] 未知のタスクです: {task}")


if __name__ == "__main__":
    main()
